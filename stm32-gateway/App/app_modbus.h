#ifndef __APP_MODBUS_H
#define __APP_MODBUS_H

/*
 * app_modbus.h — Modbus TCP 多 Socket 服务器接口
 *
 * 对外暴露:
 *   APP_Modbus_Init()   创建 TcpServerTask (在 freertos.c StartDefaultTask 中调用)
 *
 * 该任务管理 W5500 的 4 个 TCP Socket, 实现:
 *   Modbus-TCP (端口 502+) ↔ RS-485 Modbus-RTU 协议转换
 */

#include "cmsis_os.h"

/**
 * @brief 初始化 W5500 并创建 TCP Server FreeRTOS 任务
 *        任务管理 4 个并发 Socket, 监听配置的端口 (默认 502~505)
 */
void APP_Modbus_Init(void);

#endif
