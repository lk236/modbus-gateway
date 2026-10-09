#ifndef __APP_TASK_H
#define __APP_TASK_H

/*
 * app_task.h — 系统服务接口: 软件看门狗 + 异步日志
 *
 * 软件看门狗 (WdgSupervisor):
 *   监控各任务心跳, 失效时不喂硬件 IWDG, 触发 MCU 复位
 *
 * 异步日志 (LogTask):
 *   通过消息队列将日志异步写入 W25Q64 SPI Flash
 */

#include "cmsis_os.h"
#include "bsp_w25q64.h"

/* 看门狗任务 ID 枚举 */
#define WDG_TASK_ID_DEFAULT   0  /* 主导航任务 (defaultTask) */
#define WDG_TASK_ID_TCP       1  /* TCP 服务器任务 */
#define WDG_TASK_ID_SENSOR    2  /* 传感器健康检查任务 */
#define WDG_TASK_ID_LOG       3  /* 日志写入任务 */
#define WDG_TASK_COUNT        4  /* 被监控任务总数 */

/**
 * @brief 将日志事件入队 (可从中断调用)
 * @param type  日志类型 (W25Q64_LogType 枚举)
 * @param data  关联数据
 */
void Log_Push(uint8_t type, uint16_t data);

/**
 * @brief 初始化异步日志系统: 创建消息队列 + LogTask
 */
void Log_TaskInit(void);

/**
 * @brief 任务上报存活心跳 (每个任务在循环中定期调用)
 * @param task_id  WDG_TASK_ID_xxx
 */
void Wdg_Heartbeat(uint8_t task_id);

/**
 * @brief 创建 WdgSupervisor 任务 (在 IWDG 启用之前调用)
 */
void Wdg_TaskInit(void);

#endif
