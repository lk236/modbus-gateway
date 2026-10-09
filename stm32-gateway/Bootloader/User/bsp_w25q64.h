/* ============================================================================
 * bsp_w25q64.h — W25Q64 SPI Flash 驱动接口
 *
 * 与 W5500 共用 SPI1 总线 (CS=PA4), 单线程无需互斥。
 *
 * 【相对底层 Hardware 库的增强】
 *   BSP_W25Q64_Write        : 支持任意长度 (自动按页拆分成多次页编程)
 *   BSP_W25Q64_SendEraseCmd : 只发擦除命令不等待 (配合 IsBusy 做异步擦除)
 *   BSP_W25Q64_IsBusy       : 查询芯片忙状态
 * ============================================================================
 */
#ifndef __BSP_W25Q64_H
#define __BSP_W25Q64_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 容量常量 ======================== */
#define W25Q64_CAPACITY     8388608U   /* 8MB — 编址 0x000000~0x7FFFFF */
#define W25Q64_PAGE_SIZE    256U       /* 页 256B: 页编程上限, 不可跨页 */
#define W25Q64_SECTOR_SIZE  4096U      /* 扇区 4KB: 擦除的最小单位 */

/* ======================== 公开 API ======================== */
int  BSP_W25Q64_Init(void);                                     // 初始化 + 校验 JEDEC ID
int  BSP_W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);        // 读 (无限制)
int  BSP_W25Q64_Write(uint32_t addr, const uint8_t *buf, uint32_t len); // 写 (自动跨页拆分)
int  BSP_W25Q64_SectorErase(uint32_t addr);                     // 擦除 4KB 扇区 (阻塞)
void BSP_W25Q64_SendEraseCmd(uint32_t addr);                    // 只发擦除命令 (非阻塞)
int  BSP_W25Q64_IsBusy(void);                                   // 查询 BUSY (1=忙)

#ifdef __cplusplus
}
#endif

#endif /* __BSP_W25Q64_H */
