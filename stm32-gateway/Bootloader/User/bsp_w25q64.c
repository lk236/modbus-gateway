/* ============================================================================
 * bsp_w25q64.c — W25Q64 SPI Flash 驱动 (基于 Hardware 库)
 *
 * 【本文件在 Hardware/W25Q64.c 之上做了什么】
 *   Hardware/W25Q64.c 提供的是"阻塞式"基础操作 (擦除/编程会一直等芯片忙完)。
 *   本文件把它包一层, 并额外实现两件 Bootloader 专需要的能力:
 *     1) 跨页写: 一次可以写任意长度, 自动拆成多次页编程 (Flash 页=256B, 不能跨页写)
 *     2) 非阻塞擦除: 只发擦除命令立刻返回, 由调用者轮询 IsBusy —— 避免长时间卡住
 *
 * 【W25Q64 关键参数】
 *   容量 8MB / 页 256B / 扇区 4KB / 块 64KB。
 *   写入粒度: 页(256B)内可任意写, 但一条编程命令不能跨页 —— 这是最容易踩的坑。
 *   JEDEC ID: MID=0xEF (Winbond), DID=0x4017 (8MB)。
 * ============================================================================
 */
#include "bsp_w25q64.h"
#include "../Hardware/MySPI.h"          // 底层 SPI 收发 (MySPI_Start/Stop/SwapByte)
#include "../Hardware/W25Q64.h"         // Hardware 库: W25Q64_Init/ReadData/PageProgram/...
#include "../Hardware/W25Q64_Ins.h"     // Flash 指令码定义 (W25Q64_WRITE_ENABLE 等)

/* ======================== 容量常量 ======================== */
#define W25Q64_CAPACITY     8388608U   /* 8MB — 地址范围 0x000000 ~ 0x7FFFFF */

/* ======================== 初始化 ======================== */

/* 初始化: 调底层初始化, 再读 JEDEC ID 确认芯片在位。 */
int BSP_W25Q64_Init(void)
{
    W25Q64_Init();                      // Hardware 库的初始化 (内部会调 MySPI_Init)

    /* 读 JEDEC ID 校验 — 每颗 Flash 有固定的厂商/型号 ID, 读到才算通信正常 */
    uint8_t mid;
    uint16_t did;
    W25Q64_ReadID(&mid, &did);          // 读厂商 ID 和设备 ID

    /* W25Q64: MID=0xEF, DID=0x4017 */
    if (mid != 0xEF) return -1;         // 厂商 ID 不是 Winbond(0xEF) → 通信异常
    return 0;
}

/* ======================== 读数据 ======================== */

/* 读任意长度数据 (Flash 读是"随机访问", 不需要擦除, 也没有页限制)。 */
int BSP_W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (addr + len > W25Q64_CAPACITY || len == 0) return -1;    // 越界或空操作 → 拒绝
    W25Q64_ReadData(addr, buf, len);    // 底层读 (内部会发 READ_DATA 指令)
    return 0;
}

/* ======================== 写数据 (跨页) ========================
 * W25Q64 一条 Page Program 命令只能写在同一个 256B 页内。若一次要写的
 * 数据跨越页边界, 必须拆成多次。本函数就是自动拆分。
 * 例: 从地址 0x00F0 写 300 字节 →
 *   第1段: 0x00F0, 长 16  (写到 0x0100 页边界为止)
 *   第2段: 0x0100, 长 256 (整页)
 *   第3段: 0x0200, 长 28  (剩余)
 */
int BSP_W25Q64_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    if (addr + len > W25Q64_CAPACITY || len == 0) return -1;    // 越界保护

    uint32_t remain = len;              // 还剩多少字节要写
    uint32_t cur = addr;                // 当前写入地址
    const uint8_t *p = buf;             // 当前源数据指针

    while (remain > 0) {
        uint32_t page_off = cur & 0xFF;             // 在当前 256B 页内的偏移 (低 8 位)
        uint32_t chunk = (256 - page_off) < remain ? (256 - page_off) : remain;
        /* chunk = min(本页剩余空间, 还要写的字节数) —— 保证不跨页 */

        W25Q64_PageProgram(cur, (uint8_t*)p, (uint16_t)chunk);  // 写这一段 (底层含写使能+等忙)

        cur += chunk;                   // 地址前进
        p += chunk;                     // 数据指针前进
        remain -= chunk;                // 剩余减少
    }

    return 0;
}

/* ======================== 扇区擦除 (阻塞) ======================== */

/* 擦除一个 4KB 扇区 (阻塞: 内部会等芯片擦完才返回, 约 45ms)。
   注意: 擦除后整扇区变 0xFF。写数据前必须先擦。 */
int BSP_W25Q64_SectorErase(uint32_t addr)
{
    if (addr >= W25Q64_CAPACITY) return -1;     // 越界
    W25Q64_SectorErase(addr);                   // 底层阻塞擦除
    return 0;
}

/* ======================== 非阻塞擦除命令 ========================
 * 只发命令不等 Busy, 由 BSP_W25Q64_IsBusy 轮询完成状态。
 * 为什么需要它: 擦 48KB 要 12 个扇区、约 0.5s; 若阻塞等待, 会拖垮 TCP 收包。
 * 于是"发命令"和"查完成"分开, 交给 ota_handler 的异步擦除状态机调度。
 * ============================================================ */

void BSP_W25Q64_SendEraseCmd(uint32_t addr)
{
    /* 写使能 — Flash 规定: 任何写/擦操作前必须先发 Write Enable(0x06) */
    MySPI_Start();                              // CS 拉低
    MySPI_SwapByte(W25Q64_WRITE_ENABLE);        // 发 0x06
    MySPI_Stop();                               // CS 拉高

    /* 发扇区擦除命令 (0x20) + 24 位地址 */
    MySPI_Start();
    MySPI_SwapByte(W25Q64_SECTOR_ERASE_4KB);    // 发擦除指令
    MySPI_SwapByte(addr >> 16);                 // 地址高 8 位
    MySPI_SwapByte(addr >> 8);                  // 地址中 8 位
    MySPI_SwapByte(addr);                       // 地址低 8 位
    MySPI_Stop();
    /* 命令已发出, 芯片开始擦除; 立即返回, 不等它完成 */
}

/* ======================== 查询 BUSY ======================== */

/* 读状态寄存器 1 的 bit0 (BUSY): 1=芯片正忙, 0=空闲。 */
int BSP_W25Q64_IsBusy(void)
{
    uint8_t sr;
    MySPI_Start();
    MySPI_SwapByte(W25Q64_READ_STATUS_REGISTER_1);  // 发 0x05
    sr = MySPI_SwapByte(W25Q64_DUMMY_BYTE);         // 读回状态字节
    MySPI_Stop();
    return (sr & 0x01) ? 1 : 0;                     // 取 bit0
}
