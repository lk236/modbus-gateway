/* ============================================================================
 * stm32f1xx_hal.h — HAL 兼容头 (用于 OtaCommon 共享代码)
 *
 * 本 Bootloader 使用 StdPerip 标准库, 但 OtaCommon 下的共享文件
 * (ota_protocol_def.h) 需要 #include "stm32f1xx_hal.h" 来获取
 * __REV / __REV16 宏。此处提供兼容桩。
 * ============================================================================
 */
#ifndef __STM32F1XX_HAL_H
#define __STM32F1XX_HAL_H

#include "stm32f10x.h"      /* 通过 core_cm3.h 提供 __REV / __REV16 */

#endif /* __STM32F1XX_HAL_H */
