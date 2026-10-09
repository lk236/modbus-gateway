#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

#include "main.h"

#define IWDG_PRESCALER_4    0
#define IWDG_PRESCALER_8    1
#define IWDG_PRESCALER_16   2
#define IWDG_PRESCALER_32   3
#define IWDG_PRESCALER_64   4
#define IWDG_PRESCALER_128  5
#define IWDG_PRESCALER_256  6

/* IWDG 超时时间 (预分频 64 + 重装载 4000 → 约 6.4s) */
#define IWDG_TIMEOUT_MS     6400

/**
 * @brief 初始化并启动独立看门狗 (使能后不可关闭)
 */
void BSP_IWDG_Init(void);

/**
 * @brief 喂狗: 重装载计数器 (由 WdgSupervisor 任务调用)
 */
void BSP_IWDG_Reload(void);

/**
 * @brief 检查上次复位是否由 IWDG 触发
 * @return 1=看门狗复位, 0=其他原因
 */
uint8_t BSP_IWDG_IsResetByIWDG(void);

/**
 * @brief 清除所有复位标志
 */
void BSP_IWDG_ClearResetFlag(void);

#endif
