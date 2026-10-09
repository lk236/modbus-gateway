#ifndef __APP_SENSOR_H
#define __APP_SENSOR_H

/*
 * app_sensor.h — RS-485 总线健康检查任务接口
 *
 * 对外暴露:
 *   APP_Sensor_Init()   创建 HealthCheck 任务 (在 freertos.c StartDefaultTask 中调用)
 */

#include "cmsis_os.h"

/**
 * @brief 初始化 RS-485 并创建健康检查 FreeRTOS 任务
 *        任务每 60s 读取温湿度传感器一次, 监控总线状态
 */
void APP_Sensor_Init(void);

#endif
