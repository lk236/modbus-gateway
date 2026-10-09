/* ============================================================================
 * rollback.h — OTA 升级回滚接口
 *
 * 【回滚机制一句话】
 *   升级前把"当前能跑的 App"整份备份到 W25Q64 的备份区 (0x01C000),
 *   升级失败时再把它搬回内部 Flash —— 相当于给升级上了"后悔药"。
 *
 * 【W25Q64 分区 (对照 ota_protocol_def.h)】
 *   0x007000  OTA 控制块 (19B) : 是否待安装 / 失败次数
 *   0x007020  Rollback 控制块   : 备份是否有效 (magic "RBBK" + size + crc)
 *   0x010000  固件暂存区        : 新固件先到这
 *   0x01C000  固件备份区        : 旧 App 备份
 *
 * 【内部 Flash 三条铁律】只能 1→0; 擦除按页(1KB), 编程按半字(2B); 操作前后要 Unlock/Lock。
 * ============================================================================
 */
#ifndef __ROLLBACK_H
#define __ROLLBACK_H

#include <stdint.h>
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Rollback 控制块 ======================== */
/* 记录"备份区里那份固件"的信息。magic="RBBK" 表示有效。
   #pragma pack(1) 禁止结构体对齐填充 —— 因为它要原样写进 Flash 并被
   Bootloader/App/上位机三方按字节解析, 一旦有填充字节, 布局就错位了。 */
#pragma pack(1)
typedef struct {
    uint32_t magic;         // "RBBK" (0x5242424B)
    uint32_t fw_size;       // 备份固件大小
    uint32_t fw_crc;        // 备份固件 CRC32
    uint32_t reserved;      // 保留
} RollbackCtrlBlock;
#pragma pack()

/* ======================== Flash 操作 (寄存器直操) ======================== */
/* 内部 Flash 操作的薄封装 (对 ST 标准库的转发), 统一返回 0 成功 / -1 失败。 */
void Flash_Unlock(void);                                // 解写保护
void Flash_Lock(void);                                  // 上写保护
int  Flash_ErasePage(uint32_t page_addr);               // 擦一页 (1KB)
int  Flash_ProgramHalfWord(uint32_t addr, uint16_t data);// 写半字 (2B)

/* ======================== 公开 API ======================== */
int Rollback_IsValid(void);                             // 备份是否有效 (查 magic)
void Rollback_MarkValid(uint32_t fw_size, uint32_t fw_crc);  // 登记备份有效
void Rollback_Invalidate(void);                         // 使备份失效 (清 magic)
int  Rollback_BackupCurrentApp(uint32_t fw_size);       // 备份当前 App
int  Rollback_Restore(void);                            // 从备份恢复

#ifdef __cplusplus
}
#endif

#endif /* __ROLLBACK_H */
