/* ============================================================================
 * bkp_fault.h — BKP 备份寄存器故障标志
 *
 * 【BKP 是什么】
 *   STM32 的"备份域"里有 10 个 16 位备份寄存器 (BKP_DR1~DR10), 由 VBAT
 *   (纽扣电池/主电)供电。特点: 主电源断了、系统复位了, 只要 VBAT 还在,
 *   里面的内容就不丢。所以适合存"掉电也要记住"的少量状态。
 *
 * 【本工程用它干什么】
 *   记录一个"上次 OTA 写控制块失败"的标志。为什么需要:
 *   如果 OTA 把固件收完了, 但写控制块 (告知 Bootloader "有待安装固件")
 *   这一步失败了, 那这次升级就白费了 —— 存个标志好让后续逻辑/调试知道。
 *
 * 【为什么直接操作寄存器地址, 而不用库函数】
 *   本文件 Bootloader 和 App 共用。App 用的是 HAL 库, Bootloader 用的是
 *   StdPeriph 库, 两边的库函数名不同。为了"一份代码两边都能编译", 这里
 *   干脆绕开库, 直接按寄存器物理地址操作。为此需要:
 *     1) 使能 PWR 和 BKP 时钟 (否则寄存器写不进去);
 *     2) 解除备份域写保护 (PWR_CR 的 DBP 位)。
 *
 * 【地址推导】BKP 外设基址 = 0x40006C00, DR1 偏移 0x04 → 0x40006C04。
 * ============================================================================
 */
#ifndef __BKP_FAULT_H
#define __BKP_FAULT_H

#include "stm32f10x.h"                  // 提供 RCC/PWR 寄存器定义 (含 PWR_CR_DBP)

#ifdef __cplusplus
extern "C" {
#endif

#define BKP_DR1_ADDR        (0x40006C04UL)  // BKP_DR1 寄存器地址
#define FAULT_MAGIC         0xFAUL          // 故障标志魔数 (0xFA 表示"写控制块失败")

/* RCC APB1 时钟使能位 */
#define RCC_APB1ENR_PWREN   ((uint32_t)0x10000000)  /* bit 28 — PWR 时钟使能 */
#define RCC_APB1ENR_BKPEN   ((uint32_t)0x08000000)  /* bit 27 — BKP 时钟使能 */

/* 使能 PWR+BKP 时钟 + 取消备份域写保护 — 所有 BKP 读写前的必备步骤。
   做成 static inline 放在头文件里, 避免为这 3 行再单开一个 .c 文件。 */
static inline void BKP_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;  // 开 PWR+BKP 时钟
    PWR->CR |= PWR_CR_DBP;              // DBP=1 解除备份域写保护 (否则写 BKP 无效)
}

/* 写故障标志 — OTA 写控制块失败时调用。 */
static inline void BKP_WriteFaultFlag(void)
{
    BKP_Init();                         // 先开时钟+解保护
    *(__IO uint16_t*)BKP_DR1_ADDR = FAULT_MAGIC;    // 往 DR1 写 0xFA (__IO=volatile, 防止被优化)
}

/* 读故障标志 — 开机时读一次。 */
static inline uint16_t BKP_ReadFaultFlag(void)
{
    BKP_Init();
    return *(__IO uint16_t*)BKP_DR1_ADDR;
}

/* 清故障标志 — 读完就清, 避免影响下次判断。 */
static inline void BKP_ClearFaultFlag(void)
{
    BKP_Init();
    *(__IO uint16_t*)BKP_DR1_ADDR = 0;  // 写 0 清除
}

#ifdef __cplusplus
}
#endif

#endif /* __BKP_FAULT_H */
