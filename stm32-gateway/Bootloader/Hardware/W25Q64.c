/* ============================================================================
 * W25Q64.c — W25Q64 Flash 基础操作库 (阻塞式)
 *
 * 【本文件与 User/bsp_w25q64.c 的关系】
 *   这里是"最底层、最原始"的 Flash 操作: 每条命令都严格按 SPI 时序发,
 *   并阻塞等待芯片忙完。上层 bsp_w25q64.c 在此基础上加"跨页写/非阻塞擦除"。
 *
 * 【W25Q64 常用指令 (见 W25Q64_Ins.h)】
 *   0x06 WriteEnable     写使能 (每次写/擦前必发)
 *   0x20 SectorErase     扇区擦除 4KB
 *   0x02 PageProgram     页编程 (最多 256B, 不可跨页)
 *   0x03 ReadData        读数据 (随机访问, 无限制)
 *   0x9F JEDEC_ID        读厂商+设备 ID
 *   0x05 ReadStatusReg1  读状态寄存器 (bit0 = BUSY)
 * ============================================================================
 */
#include "stm32f10x.h"                  // Device header
#include "MySPI.h"                      // SPI 底层收发
#include "W25Q64_Ins.h"                 // W25Q64 指令码定义

/* 初始化: 就是初始化 SPI 总线 (仅 SPI, 不涉及 Flash 命令)。 */
void W25Q64_Init(void)
{
	MySPI_Init();
}

/* 读 JEDEC ID: 返回厂商 ID(MID) 和设备 ID(DID)。
   时序: 发 0x9F → 读 3 字节 (厂商 ID + 设备 ID 高 8 位 + 低 8 位)。 */
void W25Q64_ReadID(uint8_t *MID, uint16_t *DID)
{
	MySPI_Start();                                  // CS 拉低
	MySPI_SwapByte(W25Q64_JEDEC_ID);                // 发 0x9F
	*MID = MySPI_SwapByte(W25Q64_DUMMY_BYTE);       // 第 1 字节 = 厂商 ID
	*DID = MySPI_SwapByte(W25Q64_DUMMY_BYTE);       // 第 2 字节 = 设备 ID 高字节
	*DID <<= 8;                                     // 左移 8 位, 放到高字节
	*DID |= MySPI_SwapByte(W25Q64_DUMMY_BYTE);      // 第 3 字节 = 设备 ID 低字节, 或上去
	MySPI_Stop();                                   // CS 拉高
}

/* 发写使能命令 (0x06)。
   Flash 规定: 每次 PageProgram / SectorErase 之前必须发一次 WriteEnable,
   否则芯片忽略该写/擦命令。这是新手最常漏的一步。 */
void W25Q64_WriteEnable(void)
{
	MySPI_Start();
	MySPI_SwapByte(W25Q64_WRITE_ENABLE);            // 发 0x06
	MySPI_Stop();
}

/* 阻塞等待 Flash 空闲: 循环读状态寄存器 bit0(BUSY), 直到变 0。
   带超时(10 万次), 防止 Flash 坏掉时死等卡死整个程序。 */
void W25Q64_WaitBusy(void)
{
	uint32_t Timeout;
	MySPI_Start();
	MySPI_SwapByte(W25Q64_READ_STATUS_REGISTER_1);                  // 发 0x05
	Timeout = 100000;                                              // 超时计数
	while ((MySPI_SwapByte(W25Q64_DUMMY_BYTE) & 0x01) == 0x01)     // 读状态, 只要 bit0=1 就一直等
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;                                                 // 超时退出, 避免死循环
		}
	}
	MySPI_Stop();
}

/* 页编程: 在 Address 处写入 Count 个字节 (Count ≤ 256, 且不能跨越页边界)。
   时序: 写使能 → 发 0x02 + 24 位地址 + 数据 → 等忙完。 */
void W25Q64_PageProgram(uint32_t Address, uint8_t *DataArray, uint16_t Count)
{
	uint16_t i;
	
	W25Q64_WriteEnable();                           // 第一步: 写使能 (必须!)
	
	MySPI_Start();
	MySPI_SwapByte(W25Q64_PAGE_PROGRAM);            // 发 0x02
	MySPI_SwapByte(Address >> 16);                  // 地址高 8 位
	MySPI_SwapByte(Address >> 8);                   // 地址中 8 位
	MySPI_SwapByte(Address);                        // 地址低 8 位
	for (i = 0; i < Count; i ++)
	{
		MySPI_SwapByte(DataArray[i]);               // 依次发送数据字节
	}
	MySPI_Stop();
	
	W25Q64_WaitBusy();                              // 等写入完成 (约 0.7ms/页)
}

/* 扇区擦除 (4KB): 把 Address 所在扇区整片擦成 0xFF。
   时序: 写使能 → 发 0x20 + 24 位地址 → 等忙完 (约 45ms)。 */
void W25Q64_SectorErase(uint32_t Address)
{
	W25Q64_WriteEnable();                           // 写使能
	
	MySPI_Start();
	MySPI_SwapByte(W25Q64_SECTOR_ERASE_4KB);        // 发 0x20
	MySPI_SwapByte(Address >> 16);                  // 地址高 8 位
	MySPI_SwapByte(Address >> 8);                   // 地址中 8 位
	MySPI_SwapByte(Address);                        // 地址低 8 位
	MySPI_Stop();
	
	W25Q64_WaitBusy();                              // 等擦除完成
}

/* 读数据: 从 Address 读 Count 个字节到 DataArray。
   读是唯一"不需要写使能、不需要等忙、无页限制"的操作。 */
void W25Q64_ReadData(uint32_t Address, uint8_t *DataArray, uint32_t Count)
{
	uint32_t i;
	MySPI_Start();
	MySPI_SwapByte(W25Q64_READ_DATA);               // 发 0x03
	MySPI_SwapByte(Address >> 16);                  // 地址高 8 位
	MySPI_SwapByte(Address >> 8);                   // 地址中 8 位
	MySPI_SwapByte(Address);                        // 地址低 8 位
	for (i = 0; i < Count; i ++)
	{
		DataArray[i] = MySPI_SwapByte(W25Q64_DUMMY_BYTE);   // 连续读出 Count 个字节
	}
	MySPI_Stop();
}
