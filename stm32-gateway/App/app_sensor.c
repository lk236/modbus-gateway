/*
 * app_sensor.c — RS-485 总线健康检查任务(HealthCheck)
 *
 * 每 60 秒通过 RS-485 向温湿度传感器发一次 Modbus 请求, 借这次读写探测总线
 * 是否还通。连续 3 次失败判定"总线故障", 恢复后再记一条日志。
 *
 * 本任务即网关的自检探针: 无人值守现场传感器掉线、线缆松动、模块损坏时,
 * 网关须能自行发现并写进 Flash 日志供上位机事后查询。
 *
 * 只是"体检", 结果不对外提供; 真正的数据采集由 app_modbus.c 在处理
 * Modbus-TCP 请求时现场转成 485 请求。两者共用同一条总线, 靠 modbus_rtu.c
 * 里的互斥锁错开。
 */
#include "app_sensor.h"   /* 本任务对外接口 APP_Sensor_Init() */
#include "bsp_rs485.h"    /* RS-485 底层初始化 */
#include "modbus_rtu.h"   /* 阻塞式 Modbus 主站收发 */
#include "modbus_map.h"   /* 寄存器地址映射(0x0300 温度) */
#include "app_task.h"     /* Wdg_Heartbeat 上报心跳 + Log_Push 写日志 */
#include <stdio.h>        /* printf */

#define HEALTH_CHECK_INTERVAL_MS    60000   /* 每 60s 巡检一次(毫秒) */
#define HEALTH_CHECK_INTERVAL_SEC   60      /* 同上, 秒 —— 心跳循环用这个 */
#define HEALTH_FAIL_THRESHOLD       3       /* 连续失败 3 次 → 判定总线故障 */

static osThreadId_t g_sensor_task_handle;   /* 任务句柄 */
static uint32_t g_rs485_ok_count;           /* 累计成功次数(统计用) */

static void APP_HealthCheck_Task(void *argument)
{
    uint32_t start_tick = osKernelGetTickCount();  /* 启动时刻, 用来算开机时长 */

    uint8_t  fail_streak = 0;   /* 当前连续失败次数 */
    uint8_t  bus_was_down = 0;  /* 是否处于故障态(保证判降/恢复各只打一条日志, 不刷屏) */

    for (;;) {
        uint32_t uptime = (osKernelGetTickCount() - start_tick) / 1000;  /* 开机秒数 */
        uint16_t reg;   /* 读回的寄存器原始值 */

        /* 读从机 1 的温度寄存器(0x0300), 读 1 个。阻塞式完整 485 事务:
         * 发请求 → 等响应 → 校验 → 返回; 内部自动拿总线锁, 不与 TCP 转发冲突。 */
        Modbus_Status status = Modbus_ReadHoldingRegs(1, MODBUS_REG_TEMPERATURE, 1, &reg);

        if (status == MODBUS_OK) {
            g_rs485_ok_count++;

            if (bus_was_down) {
                /* 由故障转正常 → 打印 + 写日志, 状态置回正常 */
                printf("[RS485] BUS RECOVERED, uptime=%lus T=%.1fC [OK=%lu]\r\n",
                       uptime, (int16_t)reg / 10.0f, g_rs485_ok_count);
                       /* (int16_t) 必须: 温度有符号, 零下要按 int16 解释;
                        * 除 10.0f 是因为寄存器单位是 0.1℃ */
                Log_Push(W25Q64_LOG_485_OK, 0);
                bus_was_down = 0;
            } else {
                /* 一直正常 → 只打印不写日志, 避免刷屏磨损 Flash */
                printf("[RS485] BUS OK, uptime=%lus T=%.1fC [OK=%lu]\r\n",
                       uptime, (int16_t)reg / 10.0f, g_rs485_ok_count);
            }
            fail_streak = 0;

        } else {
            fail_streak++;

            /* 连续失败达阈值且此前未判过故障 → 判定总线故障并告警 */
            if (fail_streak >= HEALTH_FAIL_THRESHOLD && !bus_was_down) {
                printf("[RS485] BUS DOWN, uptime=%lus [%s]\r\n",
                       uptime, Modbus_StatusStr(status));   /* 打印失败原因(超时/CRC) */
                Log_Push(W25Q64_LOG_485_ERROR, fail_streak);
                bus_was_down = 1;
            }
        }

        /* 等 60s 再巡检下一轮。拆成 60 个 1s 小循环而不是直接 osDelay(60000),
         * 是因为看门狗门限 15s, 60s 不报心跳会被误判为死任务并触发整机复位;
         * 每 1s 报一次心跳既保 60s 周期, 又让看门狗知道任务活着。 */
        for (uint8_t w = 0; w < HEALTH_CHECK_INTERVAL_SEC; w++) {
            Wdg_Heartbeat(WDG_TASK_ID_SENSOR);
            osDelay(1000);
        }
    }
}

/* 创建 RS-485 健康检查任务(由 defaultTask 在初始化 485 / RTU 之后调用) */
void APP_Sensor_Init(void)
{
    g_rs485_ok_count = 0;

    BSP_RS485_Init();

    const osThreadAttr_t attr = {
        .name       = "HealthCheck",
        .stack_size = 256 * 4,       /* 1KB */
        .priority   = osPriorityLow, /* 低优先级: 60s 才跑一次, 不抢关键任务 CPU */
    };
    g_sensor_task_handle = osThreadNew(APP_HealthCheck_Task, NULL, &attr);
}
