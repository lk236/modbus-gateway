/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_sensor.h"   /* RS-485 健康检查任务 */
#include "app_modbus.h"   /* Modbus TCP 多 Socket 服务 */
#include "app_task.h"     /* 看门狗监控 + 异步日志写入 */
#include "modbus_rtu.h"   /* Modbus RTU 主站协议 */
#include "bsp_w25q64.h"   /* W25Q64 SPI Flash (配置持久化 + 事件日志) */
#include "bsp_ds3231.h"   /* DS3231 RTC (日志时间戳) */
#include "bsp_iwdg.h"     /* 独立看门狗 (IWDG) */
#include "ota_receive.h"  /* OTA 远程固件升级 */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask: 系统启动引导任务
 * - 从 W25Q64 加载配置 (IP/网关/端口)
 * - 初始化 RTC、RS-485、Modbus RTU
 * - 启动传感器轮询、TCP Server、日志、看门狗四个子任务
 * - 之后每 1s 喂一次看门狗 (心跳) */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,  /* 512 字节 */
  .priority = (osPriority_t) osPriorityNormal,
};

/* 全局网关配置 (从 W25Q64 加载, 所有模块共用) */
W25Q64_Config g_gateway_config;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* ============================================================================
 * MX_FREERTOS_Init() — FreeRTOS 任务创建入口
 *
 * 这个函数在 main.c 的 osKernelInitialize() 之后被调用。
 * 此时 FreeRTOS 内核已经初始化好了, 但还没开始运行 (调度器还没启动)。
 *
 * 这里只做一件事: 创建 defaultTask (即 StartDefaultTask)。
 * 其他所有任务 (TcpServerTask、HealthCheck、WdgSupervisor、LogTask)
 * 都不是在这里创建的, 而是在 StartDefaultTask 启动后, 由它按顺序创建。
 *
 * 为什么这样设计?
 *   因为 StartDefaultTask 需要先做一堆初始化 (Flash→RTC→配置加载),
 *   初始化完成后才创建其他任务。这样可以保证依赖关系正确。
 * ============================================================================
 */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* 可以在这里创建互斥锁 (但本项目在各自的模块里创建) */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 可以在这里创建信号量 */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* 可以在这里创建软件定时器 */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* 可以在这里创建消息队列 (但 Log 队列在 Log_TaskInit 中创建) */
  /* USER CODE END RTOS_QUEUES */

  /* ---- 创建唯一一个任务: defaultTask (StartDefaultTask) ------------------ */
  /* 注意: 这个任务会做所有初始化工作, 然后变成 "保姆任务" 每 1s 喂狗 */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 其他任务不是在这里创建的! 它们在 StartDefaultTask 内部创建 */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* 可以在这里创建事件标志 */
  /* USER CODE END RTOS_EVENTS */

}

/* ============================================================================
 * StartDefaultTask() — 系统启动引导任务
 *
 * 这是系统通电后执行的第一个也是最重要的任务。它像 "开机启动脚本" 一样,
 * 按固定顺序完成所有初始化, 然后启动各个子任务, 最后变成看门狗监控的一部分。
 *
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║  执行顺序一览:                                                  ║
 * ║                                                                  ║
 * ║  阶段1: 检查复位原因 (看门狗复位还是正常上电)                    ║
 * ║  阶段2: 初始化 W25Q64 SPI Flash, 加载网络配置参数               ║
 * ║  阶段3: 初始化 DS3231 RTC 时钟, 启动日志写入任务                ║
 * ║  阶段4: 初始化 Modbus RTU, 启动传感器和 TCP 任务                ║
 * ║  阶段5: 启动软件看门狗监控, 使能硬件看门狗 IWDG                 ║
 * ║  最后: 每 1 秒喂一次狗 (证明自己还活着)                         ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * @param argument: 未使用 (FreeRTOS 任务参数)
 * ============================================================================
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* =========================================================================
   * 阶段 1: 检查上次复位原因
   *
   * 如果系统是因为看门狗超时复位的, 打印告警信息。
   * 然后清除复位标志, 避免下次开机误报。
   * ========================================================================= */
  if (BSP_IWDG_IsResetByIWDG()) {
      /* 如果上次是看门狗把系统复位了, 说明有任务死掉了 */
      printf("[IWDG] System was reset by watchdog!\r\n");
  }
  BSP_IWDG_ClearResetFlag();  /* 清除复位标志, 下次开机重新判断 */

  /* =========================================================================
   * 阶段 2: SPI Flash 初始化 + 加载配置
   *
   * W25Q64 是一个 8MB 的 SPI Flash 芯片, 里面存了两类数据:
   *   1) 配置数据 (网关的 IP 地址、子网掩码、端口等)
   *   2) 事件日志 (启动记录、连接断开、错误信息等)
   *
   * 这里的 BSP_W25Q64_ConfigLoad() 会先去 Flash 里读配置,
   * 如果读出来的配置无效 (CRC 校验失败), 就自动用默认配置并写回 Flash。
   * 默认配置 (与 BSP_W25Q64_ConfigDefault 一致, 见 bsp_w25q64.c:444):
   *   IP = 192.168.10.100, 子网掩码 = 255.255.255.0
   *   网关 = 192.168.10.1, TCP 端口 = 502
   * ========================================================================= */
  BSP_W25Q64_Init();             /* 初始化 W25Q64: 校验 JEDEC ID (0xEF4017) */
  BSP_W25Q64_ConfigLoad(&g_gateway_config);  /* 从 Flash 加载配置, 失败用默认值 */
  /* 打印加载到的配置, 方便调试时确认参数 */
  printf("[CFG] IP: %d.%d.%d.%d GW: %d.%d.%d.%d Port: %d\r\n",
         g_gateway_config.gateway_ip[0], g_gateway_config.gateway_ip[1],
         g_gateway_config.gateway_ip[2], g_gateway_config.gateway_ip[3],
         g_gateway_config.default_gw[0], g_gateway_config.default_gw[1],
         g_gateway_config.default_gw[2], g_gateway_config.default_gw[3],
         g_gateway_config.tcp_port);

  /* =========================================================================
   * 阶段 3: RTC 时钟 + 日志系统
   *
   * DS3231 是一个高精度 RTC 时钟芯片 (通过 I2C 通信)。
   * 它的内部有电池, 断电后时间不会丢失。
   *
   * 这里有个自动校准机制:
   *   如果 RTC 里的年份小于 2024 (说明电池掉电了),
   *   就用 C 编译器的 __DATE__ 和 __TIME__ 宏把当前编译时间写进去。
   *
   * Log_TaskInit() 启动一个专门的日志写入任务:
   *   任何模块要写日志时, 调用 Log_Push() 把日志扔进消息队列,
   *   LogTask 从队列里取出, 异步写入 W25Q64 Flash。
   *   这样写日志就不会阻塞主流程 (Flash 擦写要 ~10ms)。
   * ========================================================================= */
  BSP_DS3231_Init();             /* 初始化 DS3231 RTC, 获取 Unix 时间戳 */
  Log_TaskInit();                /* 创建异步日志任务 (消息队列 + LogTask) */

  /* ---- 打印最近的 Flash 日志 (最多 10 条), 方便排查历史问题 ------------ */
  {
      uint16_t log_cnt = BSP_W25Q64_LogGetCount();   /* Flash 里有多少条日志 */
      printf("[LOG] Flash log count: %u\r\n", log_cnt);

      uint16_t show = (log_cnt > 10) ? 10 : log_cnt;  /* 最多打印 10 条 */
      /* index 0 = 最新的一条, index 1 = 次新... 以此类推 */
      for (uint16_t i = 0; i < show; i++) {
          W25Q64_LogEntry e;
          if (BSP_W25Q64_LogRead(i, &e) == W25Q64_OK) {
              const char *type_str;
              switch (e.type) {
                  case W25Q64_LOG_SYSTEM_BOOT:    type_str = "BOOT";    break;  /* 系统启动 */
                  case W25Q64_LOG_TCP_CONNECT:    type_str = "TCP_ON";  break;  /* 有客户端连 TCP */
                  case W25Q64_LOG_TCP_DISCONNECT: type_str = "TCP_OFF"; break;  /* 客户端断开 */
                  case W25Q64_LOG_485_ERROR:      type_str = "485_ERR"; break;  /* RS-485 总线故障 */
                  case W25Q64_LOG_485_OK:         type_str = "485_OK";  break;  /* RS-485 总线恢复 */
                  case W25Q64_LOG_CFG_UPDATED:    type_str = "CFG";     break;  /* 配置被修改 */
                  case W25Q64_LOG_SENSOR_DATA:    type_str = "SENSOR"; break;   /* 传感器数据 */
                  default:                        type_str = "?";       break;  /* 未知类型 */
              }
              printf("[LOG] #%u ts=%lu type=%s data=%u\r\n",
                     i, e.timestamp, type_str, e.data);
          }
      }
  }

  /* 记录本次启动事件到 Flash (以后重启可以在历史日志里看到) */
  Log_Push(W25Q64_LOG_SYSTEM_BOOT, 0);

  /* =========================================================================
   * 阶段 4: 业务模块初始化
   *
   * 这里按顺序启动三个关键模块:
   *
   *   1) Modbus_RTU_Init()
   *      创建 Modbus RTU 互斥锁 (防止多个任务同时发 RS-485 数据)
   *
   *   2) APP_Sensor_Init()
   *      创建 HealthCheck 任务。这个任务每 60 秒通过 RS-485 读一次传感器,
   *      监控总线是否正常。如果连续 3 次读不到数据, 记录故障日志。
   *
   *   3) APP_Modbus_Init()
   *      创建 TcpServerTask。这个任务管理 4 个 TCP Socket,
   *      监听 502~505 端口, 把收到的 Modbus-TCP 请求转成 Modbus-RTU
   *      发到 RS-485 总线上, 再把响应转回 TCP 回复给客户端。
   * ========================================================================= */
  Modbus_RTU_Init();             /* 创建 Modbus RTU 互斥锁 */
  OTA_Init();                    /* 初始化 OTA 固件接收模块 */
  APP_Sensor_Init();             /* 创建 HealthCheck 任务 (RS-485 总线监控) */
  APP_Modbus_Init();             /* 创建 TcpServerTask (4 个 Socket, 协议转换 + OTA Socket4) */

  /* =========================================================================
   * 阶段 5: 看门狗系统
   *
   * 这里有两层看门狗:
   *
   *   软件看门狗 (WdgSupervisor):
   *     一个 FreeRTOS 任务, 每 2 秒检查所有任务的心跳。
   *     如果某个任务超过 15 秒没报告心跳, 打印 "XX任务挂了"。
   *     只有当所有任务都活着, 才去喂硬件看门狗。
   *
   *   硬件看门狗 (IWDG):
   *     STM32 内部的独立看门狗, 一旦启动就不能关闭。
   *     超时时间约 6.4 秒。如果没人按时喂它, 它会直接复位整个系统。
   *
   *     这个顺序很重要: 先启动软件监控, 再使能硬件看门狗。
   *     否则刚使能硬件看门狗时, 还没人喂它, 6.4 秒后系统就复位了。
   * ========================================================================= */
  Wdg_TaskInit();                /* 先创建 WdgSupervisor 任务 (软件看门狗) */
  BSP_IWDG_Init();               /* 再使能硬件 IWDG (超时 6.4s) */

  /* =========================================================================
   * defaultTask 主循环: 每 1 秒报告一次心跳
   *
   * 注意: 这个任务不再做其他工作。
   * 它唯一的职责就是每 1 秒调用 Wdg_Heartbeat(WDG_TASK_ID_DEFAULT),
   * 告诉软件看门狗 "我 defaultTask 还活着"。
   *
   * 为什么把它放在这里而不是退出?
   *   因为 FreeRTOS 任务不能退出 (退出会触发 configASSERT)。
   *   所以每个任务都需要一个无限循环。
   * ========================================================================= */
  for(;;)
  {
    Wdg_Heartbeat(WDG_TASK_ID_DEFAULT);  /* 向软件看门狗报告: defaultTask 存活 */
    osDelay(1000);                        /* 睡 1 秒, 让 CPU 给其他任务用 */
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

