/* ============================================================================
 * hw_init.h — 硬件初始化接口
 *
 * 【提供什么】
 *   系统时钟、GPIO、SPI1、IWDG 初始化函数 (基于 StdPeriph 标准库),
 *   以及 SysTick 毫秒时基 (g_tick)、简易延时 delay_ms、LED 控制宏。
 *
 * 【时钟概览】HSE 8MHz → PLL x9 → 72MHz; AHB=72M, APB2=72M, APB1=36M。
 * 【看门狗】IWDG 使能后无法关闭, 超时约 26s, 长耗时循环必须插 IWDG_ReloadCounter()。
 * ============================================================================
 */
#ifndef __HW_INIT_H
#define __HW_INIT_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 全局变量 ======================== */
extern volatile uint32_t g_tick;    // 毫秒计数器 (SysTick 每 1ms +1)

/* ======================== SysTick ======================== */
void SysTick_Handler(void);         // SysTick 中断 (1ms 节拍)
uint32_t GetTick(void);             // 取当前毫秒时基

/* ======================== 简易延时 ======================== */
void delay_ms(uint32_t ms);         // 忙等延时 (内部每 10ms 喂狗)

/* ======================== 硬件初始化 ======================== */
void SystemClock_Config(void);      // 配 72MHz 时钟
void MX_GPIO_Init(void);            // 初始化 CS/RST/LED 等 GPIO
void MX_USART1_UART_Init(void);     // 初始化调试串口 (printf 出口)
void MX_SPI1_Init(void);            // 初始化 SPI1
void MX_IWDG_Init(void);            // 初始化独立看门狗

/* ======================== LED 控制 (StdPerip) ======================== */
/* 蓝丸板 PC13 的 LED 是低电平点亮, 所以 ON 用 ResetBits(输出低), OFF 用 SetBits。

   LED_TOGGLE() 是一个"翻转"宏: 读 PC13 当前输出电平, 亮则灭、灭则亮。
   注意它是多行宏 —— 每行末尾的 '\' 是"行续接符", 告诉预处理器这几行属于
   同一条宏定义。续接符必须是该行最后一个字符 (后面不能跟空格或注释)。 */
#define LED_ON()    GPIO_ResetBits(GPIOC, GPIO_Pin_13)      // 点亮
#define LED_OFF()   GPIO_SetBits(GPIOC, GPIO_Pin_13)        // 熄灭
/* 翻转: 当前输出为高(灭) 就拉低点亮, 否则拉高熄灭 */
#define LED_TOGGLE() do{ if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)) \
                            GPIO_ResetBits(GPIOC, GPIO_Pin_13); \
                        else \
                            GPIO_SetBits(GPIOC, GPIO_Pin_13); \
                     }while(0)

#ifdef __cplusplus
}
#endif

#endif /* __HW_INIT_H */
