/* ============================================================================
 * utils_crc.c — CRC 历史占位文件 (已废弃, 无实现)
 *
 * 本文件当前没有任何 CRC 实现。CRC 计算已统一迁移到共享文件:
 *   Bootloader/OtaCommon/ota_crc.c / ota_crc.h
 *   (提供 CRC16_Modbus 查表法 + CRC32_ZIP 查表法, Bootloader 与 App 共用)
 *
 * 同时存在两份 CRC16 来源, 以代码为准:
 *   - modbus_rtu.c 内的 static Modbus_CRC16() (RTU 帧校验, 自带 hi/lo 双表)
 *   - ota_crc.c 的 CRC16_Modbus() (OTA 分块校验, 与 Bootloader/上位机共用)
 *   两者多项式同为 0x8005、初值 0xFFFF, 结果一致。
 * ============================================================================
 */
#include "utils_crc.h"

/* utils_crc.c 无实现, 仅保留头文件以兼容旧 include 路径。 */
