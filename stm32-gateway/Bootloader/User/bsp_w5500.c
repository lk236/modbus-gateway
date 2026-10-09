/* ============================================================================
 * bsp_w5500.c — W5500 以太网控制器驱动 (基于 StdPerip + MySPI)
 *
 * 【W5500 是什么】
 *   一颗"硬件 TCP/IP 协议栈"芯片: MCU 只要通过 SPI 读写它的寄存器/缓冲区,
 *   就能收发以太网数据, 不必在 MCU 里跑 lwIP。代价是 socket 数量有限(8个)。
 *
 * 【和 App 侧驱动的区别 (重要, 联调必看)】
 *   本文件是 Bootloader 侧: W5500 走 SPI1 (CS=PB12, RST=PB1), 底层复用 MySPI_SwapByte。
 *   而 App 侧 (stm32-gateway/BSP/bsp_w5500.c) 走 SPI2。同一块板子物理接线必须一致,
 *   联调 failsafe 前要按实际接线核对两端工程, 否则一边 SPI1 一边 SPI2 会互相打架。
 *
 * 【W5500 的 SPI 帧格式 (所有读写的基础)】
 *   一次寄存器访问 = 4 字节头 + 数据:
 *     [ADDR_H][ADDR_L][BSB<<3 | RWB][DATA...]
 *   - ADDR_H/ADDR_L: 16 位寄存器"块内偏移"
 *   - BSB: Block Select Bits, 选择访问哪个"块" (通用寄存器 / 各 socket 的寄存器 / TX / RX 缓冲)
 *   - RWB (bit2): 1=写, 0=读 (低 2 位是 OM 工作模式, 本项目固定 00)
 *   读操作要多发一个 dummy 字节, 然后才收到有效数据。
 * ============================================================================
 */
#include "bsp_w5500.h"      // 本模块 API 声明
#include "stm32f10x.h"          /* GPIO_ResetBits, GPIO_SetBits, GPIOB... */
#include "../Hardware/MySPI.h"  // 底层 SPI: MySPI_SwapByte (全双工交换一字节)

/* ======================== W5500 SPI 帧格式 ========================
 * [ADDR_H][ADDR_L][BSB<<3 | RWB][DATA...]
 *   bit2 (RWB): 1=写, 0=读 (低 2 位为 OM, 本项目固定 00)
 */
#define W5500_OM_READ       0x00            // 读操作 (RWB=0)
#define W5500_OM_WRITE      0x04            // 写操作 (RWB=1, 即 bit2 置 1)

/* ======================== BSB 块选择编码 ========================
 * W5500 的寄存器空间按"块"划分, 8 个 socket 各占 4 个块 (寄存器/TX/RX)。
 * 公式: 通用块=0; socket n 的寄存器块 = 1+n*4; TX 块 = 2+n*4; RX 块 = 3+n*4。 */
#define W5500_BLOCK_COMMON  0               // 通用寄存器块 (MAC/IP/版本号等)
#define W5500_SOCK_REG(n)   (1 + (n) * 4)   // socket n 的寄存器块 (Sn_MR/Sn_CR/Sn_SR...)
#define W5500_TX_BLOCK(n)   (2 + (n) * 4)   // socket n 的发送缓冲区块
#define W5500_RX_BLOCK(n)   (3 + (n) * 4)   // socket n 的接收缓冲区块

/* ======================== 通用寄存器地址 (Block 0) ======================== */
#define W5500_MR            0x0000          // 模式寄存器 (bit7=RST 软复位)
#define W5500_GAR0          0x0001          // 网关地址 (4 字节, GAR0~GAR3)
#define W5500_SUBR0         0x0005          // 子网掩码 (4 字节)
#define W5500_SHAR0         0x0009          // 本机 MAC (6 字节, SHAR0~SHAR5)
#define W5500_SIPR0         0x000F          // 本机 IP (4 字节)
#define W5500_PHYCFGR       0x002E          // PHY 配置 (bit0=链路状态 LINK)
#define W5500_VERSIONR      0x0039          // 芯片版本号 (W5500 固定 0x04)

/* ======================== Socket 寄存器地址 ======================== */
#define W5500_Sn_MR         0x0000          // socket 模式 (bit0=1 表示 TCP)
#define W5500_Sn_CR         0x0001          // socket 命令 (OPEN/LISTEN/CONNECT/...)
#define W5500_Sn_SR         0x0003          // socket 状态 (CLOSED/LISTEN/ESTABLISHED...)
#define W5500_Sn_PORT0      0x0004          // 端口号高字节
#define W5500_Sn_PORT1      0x0005          // 端口号低字节
#define W5500_Sn_TX_FSR0    0x0020          // 发送空闲空间 (16 位)
#define W5500_Sn_TX_WR0     0x0024          // 发送写指针 (16 位)
#define W5500_Sn_RX_RSR0    0x0026          // 已收到数据大小 (16 位)
#define W5500_Sn_RX_RD0     0x0028          // 接收读指针 (16 位)
#define W5500_Sn_TXBUF_SIZE 0x001F          // 发送缓冲区大小配置 (单位 KB)
#define W5500_Sn_RXBUF_SIZE 0x001E          // 接收缓冲区大小配置 (单位 KB)

/* ======================== 寄存器位定义 ======================== */
#define W5500_MR_RST        0x80            // MR 的 bit7: 软复位
#define W5500_Sn_MR_TCP     0x01            // Sn_MR bit0: 协议 = TCP
#define W5500_Sn_CR_OPEN    0x01            // 命令: 打开 socket
#define W5500_Sn_CR_LISTEN  0x02            // 命令: 进入监听
#define W5500_Sn_CR_DISCON  0x08            // 命令: 断开连接
#define W5500_Sn_CR_CLOSE   0x10            // 命令: 关闭 socket
#define W5500_Sn_CR_SEND    0x20            // 命令: 发送 (把 TX 缓冲的内容真正发出去)
#define W5500_Sn_CR_RECV    0x40            // 命令: 接收确认 (告诉 W5500 我已读走数据)

/* ======================== CS / RST 控制 (StdPerip GPIO) ======================== */

#define CS_LOW()    GPIO_ResetBits(GPIOB, GPIO_Pin_12)  // 片选拉低 = 选中 W5500 (SPI 访问开始)
#define CS_HIGH()   GPIO_SetBits(GPIOB, GPIO_Pin_12)    // 片选拉高 = 释放总线
#define RST_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_1)   // 复位脚拉低 = 复位
#define RST_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_1)     // 复位脚拉高 = 正常工作

/* ======================== 内部寄存器读写 ========================
 * 所有读写都遵循同一套序列: CS 拉低 → 发 3 字节头 → 收/发数据 → CS 拉高。
 * 每次 CS 拉低到拉高是一次完整的 SPI 事务。 */

/* 写 1 字节到寄存器 */
static void W5500_WriteReg(uint8_t block, uint16_t offset, uint8_t data)
{
    CS_LOW();                                       // 选中芯片
    MySPI_SwapByte((offset >> 8) & 0xFF);           // 头字节1: 偏移高 8 位
    MySPI_SwapByte(offset & 0xFF);                  // 头字节2: 偏移低 8 位
    MySPI_SwapByte((block << 3) | W5500_OM_WRITE);  // 头字节3: BSB 左移 3 位 + 写标志(bit2=1)
    MySPI_SwapByte(data);                           // 数据字节
    CS_HIGH();                                      // 释放
}

/* 读 1 字节寄存器 */
static uint8_t W5500_ReadReg(uint8_t block, uint16_t offset)
{
    CS_LOW();
    MySPI_SwapByte((offset >> 8) & 0xFF);           // 偏移高
    MySPI_SwapByte(offset & 0xFF);                  // 偏移低
    MySPI_SwapByte((block << 3) | W5500_OM_READ);   // BSB + 读标志(bit2=0)
    MySPI_SwapByte(0xFF);           /* dummy — 读操作必须多发一个字节, 芯片才把数据推上来 */
    uint8_t val = MySPI_SwapByte(0xFF);             // 这才是真正的数据
    CS_HIGH();
    return val;
}

/* 读 16 位寄存器 (W5500 是大端: 先高字节后低字节) */
static uint16_t W5500_ReadReg16(uint8_t block, uint16_t offset)
{
    uint16_t hi = W5500_ReadReg(block, offset);     // 高字节在低地址
    uint16_t lo = W5500_ReadReg(block, offset + 1); // 低字节在高地址
    return (hi << 8) | lo;                          // 拼成一个 16 位值
}

/* 写 16 位寄存器 (大端: 先写高字节) */
static void W5500_WriteReg16(uint8_t block, uint16_t offset, uint16_t data)
{
    W5500_WriteReg(block, offset, (data >> 8) & 0xFF);      // 高字节
    W5500_WriteReg(block, offset + 1, data & 0xFF);         // 低字节
}

/* 写一段数据到缓冲区 (一次事务里连续发多个字节, 效率高; 用于 TX 缓冲写入) */
static void W5500_WriteBuf(uint8_t block, uint16_t offset,
                            const uint8_t *data, uint16_t len)
{
    CS_LOW();
    MySPI_SwapByte((offset >> 8) & 0xFF);           // 偏移高
    MySPI_SwapByte(offset & 0xFF);                  // 偏移低
    MySPI_SwapByte((block << 3) | W5500_OM_WRITE);  // 写标志
    for (uint16_t i = 0; i < len; i++) {
        MySPI_SwapByte(data[i]);                    // 连续写数据
    }
    CS_HIGH();
}

/* 从缓冲区读一段数据 (用于 RX 缓冲读取) */
static void W5500_ReadBuf(uint8_t block, uint16_t offset,
                           uint8_t *data, uint16_t len)
{
    CS_LOW();
    MySPI_SwapByte((offset >> 8) & 0xFF);           // 偏移高
    MySPI_SwapByte(offset & 0xFF);                  // 偏移低
    MySPI_SwapByte((block << 3) | W5500_OM_READ);   // 读标志
    MySPI_SwapByte(0xFF);           /* dummy — 读前一个空字节 */
    for (uint16_t i = 0; i < len; i++) {
        data[i] = MySPI_SwapByte(0xFF);             // 连续读数据
    }
    CS_HIGH();
}

/* ======================== 公开 API ======================== */

/* 初始化 W5500: 硬件复位 + 校验版本号 (版本不对则软复位重试一次)。 */
int BSP_W5500_Init(void)
{
    /* 1. 硬件复位 — RST 拉低至少 500us, 这里约 5ms, 然后拉高后再等约 10ms 让它稳定 */
    RST_LOW();
    for (volatile int d = 0; d < 360000; d++);  // ~5ms @72MHz (空转延时)
    RST_HIGH();
    for (volatile int d = 0; d < 720000; d++);  // ~10ms @72MHz

    /* 2. 检查 VERSIONR — W5500 固定返回 0x04, 读不到说明 SPI 通信有问题 */
    uint8_t ver = W5500_ReadReg(W5500_BLOCK_COMMON, W5500_VERSIONR);
    if (ver != 0x04) {
        /* 软复位重试 — 写 MR 的 RST 位(bit7), 芯片自己复位一遍, 再读一次 */
        W5500_WriteReg(W5500_BLOCK_COMMON, W5500_MR, W5500_MR_RST);
        for (volatile int d = 0; d < 720000; d++);      // 等软复位完成
        ver = W5500_ReadReg(W5500_BLOCK_COMMON, W5500_VERSIONR);
        if (ver != 0x04) return -1;                     // 仍然不对 → 初始化失败
    }
    return 0;
}

/* 配置网络参数: MAC / IP / 子网掩码 / 网关。
   这几个都是"多字节寄存器", 逐字节写 (地址连续)。 */
void BSP_W5500_SetNetwork(const uint8_t *mac, const uint8_t *ip,
                          const uint8_t *subnet, const uint8_t *gw)
{
    for (int i = 0; i < 6; i++)                         // MAC 6 字节
        W5500_WriteReg(W5500_BLOCK_COMMON, W5500_SHAR0 + i, mac[i]);

    for (int i = 0; i < 4; i++)                         // IP 4 字节
        W5500_WriteReg(W5500_BLOCK_COMMON, W5500_SIPR0 + i, ip[i]);

    for (int i = 0; i < 4; i++)                         // 子网掩码 4 字节
        W5500_WriteReg(W5500_BLOCK_COMMON, W5500_SUBR0 + i, subnet[i]);

    for (int i = 0; i < 4; i++)                         // 网关 4 字节
        W5500_WriteReg(W5500_BLOCK_COMMON, W5500_GAR0 + i, gw[i]);
}

/* 初始化一个 socket: 配缓冲区大小 → 设 TCP → 设端口 → OPEN。 */
void BSP_W5500_SocketInit(uint8_t sn, uint16_t port)
{
    uint8_t block = W5500_SOCK_REG(sn);                 // 该 socket 的寄存器块

    W5500_WriteReg(block, W5500_Sn_TXBUF_SIZE, 0x04);   // 发送缓冲 4KB
    W5500_WriteReg(block, W5500_Sn_RXBUF_SIZE, 0x04);   // 接收缓冲 4KB
    /* (8 个 socket 的缓冲总和不能超过 16KB; 这里各给 4KB) */

    W5500_WriteReg(block, W5500_Sn_MR, W5500_Sn_MR_TCP);    // 协议选 TCP

    W5500_WriteReg(block, W5500_Sn_PORT0, (port >> 8) & 0xFF);  // 端口高字节
    W5500_WriteReg(block, W5500_Sn_PORT1, port & 0xFF);         // 端口低字节

    W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_OPEN);       // 执行 OPEN 命令
}

/* 让 socket 进入监听状态 (服务端等待客户端连接)。 */
void BSP_W5500_SocketListen(uint8_t sn)
{
    W5500_WriteReg(W5500_SOCK_REG(sn), W5500_Sn_CR, W5500_Sn_CR_LISTEN);    // 执行 LISTEN
}

/* 主动断开当前连接 (发送 FIN)。 */
void BSP_W5500_SocketDisconnect(uint8_t sn)
{
    W5500_WriteReg(W5500_SOCK_REG(sn), W5500_Sn_CR, W5500_Sn_CR_DISCON);    // 执行 DISCON
}

/* 关闭 socket (回到 CLOSED, 释放资源)。 */
void BSP_W5500_SocketClose(uint8_t sn)
{
    W5500_WriteReg(W5500_SOCK_REG(sn), W5500_Sn_CR, W5500_Sn_CR_CLOSE);     // 执行 CLOSE
}

/* 读 socket 当前状态 (Sn_SR): LISTEN/ESTABLISHED/CLOSE_WAIT... */
uint8_t BSP_W5500_SocketStatus(uint8_t sn)
{
    return W5500_ReadReg(W5500_SOCK_REG(sn), W5500_Sn_SR);
}

/* 查询接收缓冲区里有多少字节可读。 */
uint16_t BSP_W5500_SocketRxAvail(uint8_t sn)
{
    return W5500_ReadReg16(W5500_SOCK_REG(sn), W5500_Sn_RX_RSR0);
}

/* 查询发送缓冲区还剩多少空闲字节 (用于判断能不能发)。 */
uint16_t BSP_W5500_SocketTxFree(uint8_t sn)
{
    return W5500_ReadReg16(W5500_SOCK_REG(sn), W5500_Sn_TX_FSR0);
}

/* 从 socket 接收数据。返回实际读到的字节数。
   流程: 查可读长度 → 读当前读指针 → 从 RX 缓冲读数据 → 更新读指针 → 发 RECV 命令确认。 */
uint16_t BSP_W5500_SocketRecv(uint8_t sn, uint8_t *buf, uint16_t max_len)
{
    uint8_t block = W5500_SOCK_REG(sn);
    uint16_t rx_len = W5500_ReadReg16(block, W5500_Sn_RX_RSR0);     // 可读字节数

    if (rx_len == 0) return 0;                      // 没数据
    if (rx_len > max_len) rx_len = max_len;         // 不超过调用者的缓冲大小

    uint16_t rd_ptr = W5500_ReadReg16(block, W5500_Sn_RX_RD0);      // 读指针 (环形)

    uint8_t rx_block = W5500_RX_BLOCK(sn);          // 该 socket 的 RX 缓冲块
    W5500_ReadBuf(rx_block, rd_ptr, buf, rx_len);   // 从缓冲把数据读出来

    rd_ptr += rx_len;                               // 读指针前进
    W5500_WriteReg16(block, W5500_Sn_RX_RD0, rd_ptr);   // 写回读指针

    W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_RECV);   // 发 RECV 命令: 告诉芯片"数据取走了"

    return rx_len;                                  // 返回读到的长度
}

/* 往 socket 发送数据。返回 0 成功, -1 失败 (发送缓冲不够)。
   流程: 查空闲 → 读写指针 → 数据写入 TX 缓冲 → 更新写指针 → 发 SEND 命令。 */
int8_t BSP_W5500_SocketSend(uint8_t sn, const uint8_t *data, uint16_t len)
{
    uint8_t block = W5500_SOCK_REG(sn);

    uint16_t free_size = W5500_ReadReg16(block, W5500_Sn_TX_FSR0);  // 空闲空间
    if (free_size < len) return -1;                 // 不够放 → 发送失败

    uint16_t wr_ptr = W5500_ReadReg16(block, W5500_Sn_TX_WR0);      // 写指针 (环形)

    uint8_t tx_block = W5500_TX_BLOCK(sn);          // 该 socket 的 TX 缓冲块
    W5500_WriteBuf(tx_block, wr_ptr, data, len);    // 数据写进 TX 缓冲

    wr_ptr += len;                                  // 写指针前进
    W5500_WriteReg16(block, W5500_Sn_TX_WR0, wr_ptr);   // 写回写指针

    W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_SEND);   // 发 SEND 命令: 真正发出去

    return 0;
}

/* 读 PHY 链路状态 (bit0): 1=网线已连接, 0=断开。 */
uint8_t BSP_W5500_ReadPHYLink(void)
{
    return W5500_ReadReg(W5500_BLOCK_COMMON, W5500_PHYCFGR) & 0x01;     // 只取 bit0
}
