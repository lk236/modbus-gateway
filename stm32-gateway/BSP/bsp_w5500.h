#ifndef __BSP_W5500_H
#define __BSP_W5500_H

/*
 * bsp_w5500.h — W5500 以太网控制器寄存器定义与 API
 *
 * W5500 内部寄存器通过 SPI 访问, 编址方式:
 *   偏移地址 (16-bit) + 块选择 (BSB, 5-bit) + 读写控制 (1-bit)
 *
 * BSB 编码:
 *   块 0:   通用寄存器 (Common)
 *   块 1+4n: Socket n 寄存器 (n=0~7)
 *   块 2+4n: Socket n TX 缓冲区
 *   块 3+4n: Socket n RX 缓冲区
 *
 * SPI 帧: [ADDR_H][ADDR_L][BSB<<3 | OM][DATA]
 *   OM BIT2=1 写, BIT2=0 读
 */

#include "main.h"

/* ======================== 硬件接口引脚 ======================== */
#define W5500_CS_PORT       GPIOB
#define W5500_CS_PIN        GPIO_PIN_12
#define W5500_RST_PORT      GPIOB
#define W5500_RST_PIN       GPIO_PIN_1

#define W5500_CS_LOW()      HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_RESET)
#define W5500_CS_HIGH()     HAL_GPIO_WritePin(W5500_CS_PORT, W5500_CS_PIN, GPIO_PIN_SET)
#define W5500_RST_LOW()     HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_RESET)
#define W5500_RST_HIGH()    HAL_GPIO_WritePin(W5500_RST_PORT, W5500_RST_PIN, GPIO_PIN_SET)

/* ======================== BSB 块选择编码 ======================== */
#define W5500_BLOCK_COMMON  0           /* 通用寄存器块 */
#define W5500_SOCK_REG(n)   (1 + (n) * 4)  /* Socket n 寄存器块 (BSB) */
#define W5500_BLOCK_S0      1
#define W5500_BLOCK_S1      5
#define W5500_BLOCK_S2      9
#define W5500_BLOCK_S3      13
#define W5500_BLOCK_S4      17
#define W5500_BLOCK_S5      21
#define W5500_BLOCK_S6      25
#define W5500_BLOCK_S7      29

#define W5500_TX_BLOCK(n)   (2 + (n) * 4)  /* Socket n TX 缓冲区块 */
#define W5500_RX_BLOCK(n)   (3 + (n) * 4)  /* Socket n RX 缓冲区块 */

/* ======================== 通用寄存器地址 (Block 0) ======================== */
#define W5500_MR            0x0000  /* 模式寄存器 */
#define W5500_GAR0          0x0001  /* 网关地址 (GW) */
#define W5500_GAR1          0x0002
#define W5500_GAR2          0x0003
#define W5500_GAR3          0x0004
#define W5500_SUBR0         0x0005  /* 子网掩码 (SUB) */
#define W5500_SUBR1         0x0006
#define W5500_SUBR2         0x0007
#define W5500_SUBR3         0x0008
#define W5500_SHAR0         0x0009  /* MAC 地址 (SHAR) */
#define W5500_SHAR1         0x000A
#define W5500_SHAR2         0x000B
#define W5500_SHAR3         0x000C
#define W5500_SHAR4         0x000D
#define W5500_SHAR5         0x000E
#define W5500_SIPR0         0x000F  /* IP 地址 (SIPR) */
#define W5500_SIPR1         0x0010
#define W5500_SIPR2         0x0011
#define W5500_SIPR3         0x0012
#define W5500_PHYCFGR       0x002E  /* PHY 配置寄存器 */
#define W5500_VERSIONR      0x0039  /* 版本寄存器 (应 = 0x04) */

/* ======================== Socket n 寄存器地址 (Block 1+4n) ======================== */
#define W5500_Sn_MR         0x0000  /* Socket n 模式寄存器 */
#define W5500_Sn_CR         0x0001  /* Socket n 命令寄存器 */
#define W5500_Sn_IR         0x0002  /* Socket n 中断寄存器 */
#define W5500_Sn_SR         0x0003  /* Socket n 状态寄存器 */
#define W5500_Sn_PORT0      0x0004  /* Socket n 端口 (高字节) */
#define W5500_Sn_PORT1      0x0005  /* Socket n 端口 (低字节) */
#define W5500_Sn_DHAR0      0x0006  /* 目标 MAC 地址 */
#define W5500_Sn_DHAR1      0x0007
#define W5500_Sn_DHAR2      0x0008
#define W5500_Sn_DHAR3      0x0009
#define W5500_Sn_DHAR4      0x000A
#define W5500_Sn_DHAR5      0x000B
#define W5500_Sn_DIPR0      0x000C  /* 目标 IP 地址 */
#define W5500_Sn_DIPR1      0x000D
#define W5500_Sn_DIPR2      0x000E
#define W5500_Sn_DIPR3      0x000F
#define W5500_Sn_DPORT0     0x0010  /* 目标端口 (高字节) */
#define W5500_Sn_DPORT1     0x0011  /* 目标端口 (低字节) */
#define W5500_Sn_MSSR0      0x0012  /* MSS 最大值 */
#define W5500_Sn_MSSR1      0x0013
#define W5500_Sn_TX_FSR0    0x0020  /* TX 空闲空间 (读取前需先读 FSR1) */
#define W5500_Sn_TX_FSR1    0x0021
#define W5500_Sn_TX_RD0     0x0022  /* TX 读指针 (硬件维护) */
#define W5500_Sn_TX_RD1     0x0023
#define W5500_Sn_TX_WR0     0x0024  /* TX 写指针 (软件控制) */
#define W5500_Sn_TX_WR1     0x0025
#define W5500_Sn_RX_RSR0    0x0026  /* RX 已接收字节数 */
#define W5500_Sn_RX_RSR1    0x0027
#define W5500_Sn_RX_RD0     0x0028  /* RX 读指针 (软件控制) */
#define W5500_Sn_RX_RD1     0x0029
#define W5500_Sn_RX_WR0     0x002A  /* RX 写指针 (硬件维护) */
#define W5500_Sn_RX_WR1     0x002B
#define W5500_Sn_TXBUF_SIZE 0x001F  /* TX 缓冲区大小 (单位 KB) */
#define W5500_Sn_RXBUF_SIZE 0x001E  /* RX 缓冲区大小 (单位 KB) */

#define W5500_SOCKET_TX_BASE     0x0000
#define W5500_SOCKET_RX_BASE     0x0000

/* ======================== MR 模式寄存器位 ======================== */
#define W5500_MR_RST         0x80  /* 软件复位 */
#define W5500_MR_WOL         0x20  /* 网络唤醒 */
#define W5500_MR_PB          0x10  /* 广播阻塞 */
#define W5500_MR_PPPOE       0x08  /* PPPoE 模式 */
#define W5500_MR_FARP        0x02  /* 强制 ARP */

/* ======================== Sn_MR Socket 模式 ======================== */
#define W5500_Sn_MR_CLOSE    0x00  /* 关闭 */
#define W5500_Sn_MR_TCP      0x01  /* TCP */
#define W5500_Sn_MR_UDP      0x02  /* UDP */
#define W5500_Sn_MR_MACRAW   0x04  /* MAC RAW */

/* ======================== Sn_CR Socket 命令 ======================== */
#define W5500_Sn_CR_OPEN     0x01  /* 打开 */
#define W5500_Sn_CR_LISTEN   0x02  /* 监听 (TCP 服务器) */
#define W5500_Sn_CR_CONNECT  0x04  /* 连接 (TCP 客户端) */
#define W5500_Sn_CR_DISCON   0x08  /* 断开 */
#define W5500_Sn_CR_CLOSE    0x10  /* 关闭 */
#define W5500_Sn_CR_SEND     0x20  /* 发送 */
#define W5500_Sn_CR_SEND_MAC 0x21  /* MAC 发送 */
#define W5500_Sn_CR_SEND_KEEP 0x22 /* 发送保活 */
#define W5500_Sn_CR_RECV     0x40  /* 接收完成 (释放 RX 缓冲) */

/* ======================== Sn_IR Socket 中断标志 ======================== */
#define W5500_Sn_IR_SEND_OK  0x10  /* 发送完成 */
#define W5500_Sn_IR_TIMEOUT  0x08  /* 超时 */
#define W5500_Sn_IR_RECV     0x04  /* 收到数据 */
#define W5500_Sn_IR_DISCON   0x02  /* 断开 */
#define W5500_Sn_IR_CON      0x01  /* 连接建立 */

/* ======================== Sn_SR Socket 状态 ======================== */
#define W5500_Sn_SR_CLOSED      0x00  /* 关闭 */
#define W5500_Sn_SR_INIT        0x13  /* 初始化 */
#define W5500_Sn_SR_LISTEN      0x14  /* 监听中 */
#define W5500_Sn_SR_SYNSENT     0x15  /* SYN 已发送 */
#define W5500_Sn_SR_SYNRECV     0x16  /* SYN 已接收 (三次握手进行中) */
#define W5500_Sn_SR_ESTABLISHED 0x17  /* 连接已建立 */
#define W5500_Sn_SR_FIN_WAIT    0x18  /* 等待 FIN */
#define W5500_Sn_SR_CLOSING     0x1A  /* 关闭中 */
#define W5500_Sn_SR_TIME_WAIT   0x1B  /* 等待超时 */
#define W5500_Sn_SR_CLOSE_WAIT  0x1C  /* 等待关闭 */
#define W5500_Sn_SR_LAST_ACK    0x1D  /* 等待最后 ACK */
#define W5500_Sn_SR_UDP         0x22  /* UDP */
#define W5500_Sn_SR_MACRAW      0x42  /* MAC RAW */

/* ======================== PHYCFGR PHY 配置位 ======================== */
#define W5500_PHYCFGR_RST    0x80  /* PHY 复位 */
#define W5500_PHYCFGR_OPMD   0x40  /* 操作模式 */
#define W5500_PHYCFGR_OPMDC  0x38  /* 操作模式配置 */
#define W5500_PHYCFGR_DPX    0x04  /* 双工: 1=全双工 */
#define W5500_PHYCFGR_SPD    0x02  /* 速率: 1=100Mbps, 0=10Mbps */
#define W5500_PHYCFGR_LNK    0x01  /* 链路状态: 1=已连接 */

/* ======================== 默认参数 ======================== */
#define W5500_SOCK_TCP       0     /* Socket 类型 TCP */
#define W5500_TCP_PORT       502   /* Modbus TCP 标准端口 */
#define W5500_TX_BUF_SIZE    2048  /* TX 缓冲区大小 */
#define W5500_RX_BUF_SIZE    2048  /* RX 缓冲区大小 */

void BSP_W5500_Init(const uint8_t *ip, const uint8_t *subnet, const uint8_t *gateway);
void BSP_W5500_Reset(void);
uint8_t BSP_W5500_ReadReg(uint8_t block, uint16_t offset);
void BSP_W5500_WriteReg(uint8_t block, uint16_t offset, uint8_t data);
void BSP_W5500_ReadBuf(uint8_t block, uint16_t offset, uint8_t *data, uint16_t len);
void BSP_W5500_WriteBuf(uint8_t block, uint16_t offset, const uint8_t *data, uint16_t len);
uint16_t BSP_W5500_ReadReg16(uint8_t block, uint16_t offset);
void BSP_W5500_WriteReg16(uint8_t block, uint16_t offset, uint16_t data);
void BSP_W5500_SetNetwork(const uint8_t *mac, const uint8_t *ip,
                          const uint8_t *subnet, const uint8_t *gateway);
void BSP_W5500_SocketInit(uint8_t sn, uint8_t protocol, uint16_t port);
void BSP_W5500_SocketListen(uint8_t sn);
void BSP_W5500_SocketClose(uint8_t sn);
void BSP_W5500_SocketDisconnect(uint8_t sn);
uint8_t BSP_W5500_SocketStatus(uint8_t sn);
uint16_t BSP_W5500_SocketRxAvail(uint8_t sn);
uint16_t BSP_W5500_SocketTxFree(uint8_t sn);
int8_t BSP_W5500_SocketSend(uint8_t sn, const uint8_t *data, uint16_t len);
uint16_t BSP_W5500_SocketRecv(uint8_t sn, uint8_t *buf, uint16_t len);

#endif
