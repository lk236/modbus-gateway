/* ============================================================================
 * ota_ctrl_block.h — OTA 控制块读写接口
 *
 * 本文件为 Bootloader 与 App 共用，禁止复制粘贴。
 * 平台相关的 W25Q64 读写通过函数指针或宏适配。
 * ============================================================================
 */
#ifndef __OTA_CTRL_BLOCK_H
#define __OTA_CTRL_BLOCK_H

#include "ota_crc.h"
#include "ota_protocol_def.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 平台适配宏 ========================
 * 使用前必须在 c 文件中定义以下宏指向实际 W25Q64 读写函数:
 *
 *   #define OTA_FLASH_READ(addr, buf, len)  BSP_W25Q64_Read(addr, buf, len)
 *   #define OTA_FLASH_WRITE(addr, buf, len) BSP_W25Q64_Write(addr, buf, len)
 *   #define OTA_FLASH_SECTOR_ERASE(addr)    BSP_W25Q64_SectorErase(addr)
 *   #define OTA_FLASH_WAIT_BUSY()           BSP_W25Q64_WaitBusy(1000)
 *   #define OTA_FLASH_IS_BUSY()             BSP_W25Q64_IsBusy()
 */

/* ======================== 控制块操作 ======================== */

/**
 * @brief 读取并校验 OTA 控制块
 * @param ctrl 输出缓冲区 (≥19 字节)
 * @return 0=有效, -1=magic 不匹配, -2=CRC 不匹配
 */
static inline int OTA_ReadCtrlBlock(OTACtrlBlock *ctrl)
{
    OTA_FLASH_READ(CTRL_BLOCK_ADDR, (uint8_t*)ctrl, sizeof(OTACtrlBlock));

    if (ctrl->magic != OTA_CTRL_MAGIC) return -1;

    uint16_t calc = CRC16_Modbus((uint8_t*)ctrl, CTRL_CRC_LEN);
    if (calc != ctrl->ctrl_crc16) return -2;

    return 0;
}

/**
 * @brief 擦除控制块扇区并校验空白
 * @return 0=成功, -1=超时/非全0xFF
 */
static inline int OTA_EraseAndVerifyCtrlSector(void)
{
    OTA_FLASH_SECTOR_ERASE(CTRL_BLOCK_ADDR);

    /* 等待 BUSY 完成, 每次迭代延时喂狗 (平台通过 OTA_FLASH_WAIT_BUSY 实现) */
    uint32_t timeout = 2000;
    while (OTA_FLASH_IS_BUSY() != 0) {
        if (--timeout == 0) return -1;
        OTA_FLASH_WAIT_BUSY();
    }

    uint8_t check[sizeof(OTACtrlBlock)];
    OTA_FLASH_READ(CTRL_BLOCK_ADDR, check, sizeof(OTACtrlBlock));
    for (uint32_t i = 0; i < sizeof(OTACtrlBlock); i++) {
        if (check[i] != 0xFF) return -1;
    }

    return 0;
}

/**
 * @brief 写入 OTA 控制块 (先擦后写 + 回读校验)
 * @param ctrl 待写入的控制块
 * @return 0=成功, -1=擦失败, -2=空白错, -3=回读错
 */
static inline int OTA_WriteCtrlBlock(const OTACtrlBlock *ctrl)
{
    if (OTA_EraseAndVerifyCtrlSector() != 0) {
        return -1;
    }

    /* 先写 [4~18] = 15 字节 (firmware_size ~ ctrl_crc16) */
    OTA_FLASH_WRITE(CTRL_BLOCK_ADDR + 4, (uint8_t*)&ctrl->firmware_size,
                    sizeof(OTACtrlBlock) - 4);

    /* 最后写 [0~3] = magic */
    OTA_FLASH_WRITE(CTRL_BLOCK_ADDR + 0, (uint8_t*)&ctrl->magic, 4);

    /* 回读校验 */
    OTACtrlBlock verify;
    OTA_FLASH_READ(CTRL_BLOCK_ADDR, (uint8_t*)&verify, sizeof(OTACtrlBlock));
    if (memcmp(ctrl, &verify, sizeof(OTACtrlBlock)) != 0) {
        /* 回读失败: 清 magic 确保控制块无效 */
        uint32_t zero = 0;
        OTA_FLASH_WRITE(CTRL_BLOCK_ADDR + 0, (uint8_t*)&zero, 4);
        return -3;
    }

    return 0;
}

/* ======================== 失败标记 ======================== */

/**
 * @brief 统计 fail_flag 中失败次数 (≠0xFF 的字节数)
 */
static inline uint8_t GetFailCount(const OTACtrlBlock *ctrl)
{
    uint8_t cnt = 0;
    for (int i = 0; i < 3; i++) {
        if (ctrl->fail_flag[i] != 0xFF) cnt++;
    }
    return cnt;
}

/**
 * @brief 内存中标记一次失败 (首个 0xFF→0x00)
 */
static inline void MarkOneFail(OTACtrlBlock *ctrl)
{
    for (int i = 0; i < 3; i++) {
        if (ctrl->fail_flag[i] == 0xFF) {
            ctrl->fail_flag[i] = 0x00;
            return;
        }
    }
}

/**
 * @brief 标记一次升级失败 (写入 W25Q64)
 *
 * 规则:
 *   有有效控制块 → 仅写 fail_flag[3] (1→0, 不改 last_error)
 *   无有效控制块 → 创建全新 (含错误码)
 *
 * @param err     错误码 (ERR_xxx)
 * @param fw_size 固件大小 (仅新建时使用)
 * @param fw_crc  固件 CRC (仅新建时使用)
 * @return 0=成功
 */
static inline int OTA_MarkFail(uint8_t err,
                                uint32_t fw_size, uint32_t fw_crc)
{
    OTACtrlBlock ctrl;
    int ret = OTA_ReadCtrlBlock(&ctrl);

    if (ret != 0) {
        /* 无有效控制块 → 全新创建 */
        memset(&ctrl, 0xFF, sizeof(ctrl));
        ctrl.magic = OTA_CTRL_MAGIC;
        ctrl.firmware_size = fw_size;
        ctrl.firmware_crc32 = fw_crc;
        ctrl.fail_flag[0] = 0xFF;
        ctrl.fail_flag[1] = 0xFF;
        ctrl.fail_flag[2] = 0xFF;
        ctrl.last_error = err;      /* 首次写入, 0xFF→err, 合法 */
        ctrl.reserved = 0xFF;
        ctrl.ctrl_crc16 = CRC16_Modbus((uint8_t*)&ctrl, CTRL_CRC_LEN);
        return OTA_WriteCtrlBlock(&ctrl);
    }

    /* 有有效控制块: 只改 fail_flag, 不改 last_error */
    MarkOneFail(&ctrl);
    /* 仅写 bytes[12~14], 1→0, 无需擦, 不更新 CRC */
    OTA_FLASH_WRITE(CTRL_BLOCK_ADDR + 12, (uint8_t*)&ctrl.fail_flag, 3);
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* __OTA_CTRL_BLOCK_H */
