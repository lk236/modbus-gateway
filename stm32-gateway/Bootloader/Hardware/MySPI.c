/* ============================================================================
 * MySPI.c — SPI1 底层收发封装 (STM32F103 标准外设库)
 *
 * 【这个文件干什么】
 *   把 SPI1 的初始化、片选控制、以及"收发一个字节"封装成最简函数,
 *   给上层 Flash / W5500 驱动复用。
 *
 * 【本工程 SPI 配置要点 (SPI 四要素)】
 *   主从:   Master (STM32 是主机)
 *   模式:   CPOL=0, CPHA=0 → 即 SPI Mode 0
 *           CPOL=0: 空闲时时钟为低; CPHA=0: 在第一个时钟边沿(上升沿)采样。
 *   位序:   MSB First (高位先出)
 *   速率:   预分频 128 → 72MHz/128 ≈ 562.5kHz (保守速度, 保证长线/杜邦线也稳定)
 *   片选:   软件 NSS (SPI_NSS_Soft), 片选脚由代码用 GPIO 手动控制。
 *
 * 【引脚 (SPI1 默认复用)】
 *   PA5 = SCK(时钟)  PA6 = MISO(主机入)  PA7 = MOSI(主机出)  PA4 = CS(软件片选)
 * ============================================================================
 */
#include "stm32f10x.h"                  // Device header

/* 软件片选控制: 把 PA4 当普通 GPIO 用, BitValue=0 选中, =1 释放。
   为什么用软件片选而不用硬件 NSS: 一条 SPI 总线可能挂多个从机(Flash+W5500),
   由代码分别控制各自的 CS, 更灵活。 */
void MySPI_W_SS(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)BitValue);   // 写 PA4
}

/* SPI1 初始化 */
void MySPI_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    // 开 GPIOA 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);     // 开 SPI1 时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	/* PA4 = CS: 普通推挽输出 (软件控制片选) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* PA5(SCK) + PA7(MOSI): 复用推挽输出 (由 SPI 硬件控制电平) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* PA6(MISO): 上拉输入 (从机驱动, 空闲时上拉保证电平确定) */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                       // 主机模式
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // 全双工 (2 线)
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                   // 每次 8 位
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                  // 高位先出
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;// 分频 128 ≈ 562.5kHz
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                          // 空闲低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                        // 第 1 个边沿采样 → Mode 0
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                           // 软件 NSS
	SPI_InitStructure.SPI_CRCPolynomial = 7;                            // CRC 多项式 (未用)
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);                                   // 使能 SPI1
	
	MySPI_W_SS(1);                                           // CS 默认拉高 (不选中)
}

/* 开始一次 SPI 事务: 拉低片选, 选中从机。 */
void MySPI_Start(void)
{
	MySPI_W_SS(0);                                           // CS = 0
}

/* 结束一次 SPI 事务: 拉高片选, 释放总线。 */
void MySPI_Stop(void)
{
	MySPI_W_SS(1);                                           // CS = 1
}

/* SPI 全双工交换一个字节: 发送 ByteSend, 同时返回从机发来的字节。
   步骤: 等发送寄存器空 → 写入 → 等接收寄存器非空 → 读走。 */
uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != SET);   // 等 TXE=1 (发送缓冲空)
	
	SPI_I2S_SendData(SPI1, ByteSend);                              // 写入待发数据 (触发时钟)
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET);  // 等 RXNE=1 (收到一个字节)
	
	return SPI_I2S_ReceiveData(SPI1);                              // 读出收到的字节
}
