/* ============================================================================
 * ota_crc.h — CRC32-ZIP + CRC16-Modbus
 * Bootloader 与 App 共用，禁止复制粘贴。
 * ============================================================================
 */
#ifndef __OTA_CRC_H
#define __OTA_CRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CRC32-ZIP (多项式 0x04C11DB7, 初值 0xFFFFFFFF, 出异或 0xFFFFFFFF)
 * @param data 数据指针
 * @param len  数据长度
 * @return CRC32 校验值
 */
uint32_t CRC32_ZIP(const uint8_t *data, uint32_t len);

/**
 * @brief CRC32-ZIP 增量更新 (用于流式计算)
 * @param crc  当前 CRC 值 (首次调用传 0xFFFFFFFF)
 * @param data 数据块指针
 * @param len  数据块长度
 * @return 更新后的 CRC 值 (末次调用后需 ^ 0xFFFFFFFF)
 */
uint32_t CRC32_ZIP_Update(uint32_t crc, const uint8_t *data, uint32_t len);

/**
 * @brief CRC16-Modbus (多项式 0x8005, 初值 0xFFFF, 出异或 0x0000)
 * @param data 数据指针
 * @param len  数据长度
 * @return CRC16 校验值
 */
uint16_t CRC16_Modbus(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_CRC_H */
