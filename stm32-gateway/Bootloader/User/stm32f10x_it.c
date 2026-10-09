/* ============================================================================
 * stm32f10x_it.c — 中断/异常服务函数 (STM32F10x StdPeriph 模板)
 *
 * 【本文件在做什么】
 *   这是 ST 官方模板, 里面是 Cortex-M3 的"内核异常"处理函数, 名字固定,
 *   由启动文件 (startup_stm32f10x_xx.s) 的向量表指向这里。改名会导致
 *   链接时找不到符号, 中断触发就跳到默认死循环。
 *
 * 【为什么这些函数大多是空的】
 *   内核异常 (HardFault/BusFault/...) 平时不该发生; 一旦发生说明程序跑飞,
 *   模板默认"死循环", 便于调试器停在那里定位。本工程用到的真正中断
 *   (SysTick) 被搬到 hw_init.c 里实现 (见下方 SysTick_Handler 的说明)。
 *
 * 【HardFault 排查思路】
 *   1) 让调试器停在 HardFault_Handler 的死循环, 查看 LR 判断是从哪进来的
 *      (MSP/PSP);
 *   2) 读栈帧里的 PC 值 (出错指令地址), 在反汇编里定位;
 *   3) 读 SCB->CFSR (0xE000ED28) 看具体原因: 非法访问/非对齐/除零/取指错误等;
 *   4) 常见诱因: 空指针解引用、数组越界、函数指针未初始化、栈溢出、
 *      中断里用了不安全的操作。
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  NMI 异常处理 (不可屏蔽中断)。
  *        通常由硬件看门狗或时钟安全系统(CSS)触发。这里空实现。
  */
void NMI_Handler(void)
{
}

/**
  * @brief  HardFault 异常处理 —— 最严重的异常, 程序"跑飞"时最终会落到这里。
  *         保持死循环, 方便调试器停下来排查 (排查方法见文件头)。
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  MemManage 异常 (内存管理错误) —— 需要一个 MPU 才会触发。
  *        本工程未启用 MPU, 正常不会进入。
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  BusFault 异常 (总线错误) —— 访问了不存在的地址/外设时钟没开就想读写寄存器,
  *        都会触发它。同样死循环便于定位。
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  UsageFault 异常 (用法错误) —— 如除零、非对齐访问、非法状态切换等。
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  SVCall 异常 —— 由 SVC 指令触发, 主要用于 RTOS 的系统调用。
  *        本工程 (Bootloader) 无 RTOS, 空实现。
  */
void SVC_Handler(void)
{
}

/**
  * @brief  DebugMon 异常 —— 调试监视器异常, 一般不用。
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  PendSV 异常 —— 挂起的可屏蔽异常, 专用于 RTOS 任务上下文切换。
  *        Bootloader 无 RTOS, 空实现。
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  SysTick 中断处理 (系统节拍定时器)。
  *         注意: 本工程把它的实现放在了 hw_init.c (做 g_tick++), 这里不再重复定义,
  *         否则会与 hw_init.c 中的定义冲突 (重复符号)。
  */
/* SysTick_Handler 定义在 hw_init.c 中, 此处删除重复定义 */

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  外设中断服务函数添加处 (模板示例)。
  *         本工程 Bootloader 未使用外设中断 (串口/SPI 都用轮询), 故留空。
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
