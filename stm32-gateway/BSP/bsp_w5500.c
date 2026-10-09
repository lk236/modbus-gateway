/*
 * bsp_w5500.c — W5500 以太网控制器驱动
 *
 * W5500 是 WIZnet 的硬件 TCP/IP 协议栈芯片: 内部硬件实现 TCP/UDP/IP/ARP/ICMP,
 * 带 32KB 收发缓冲。MCU 不需跑 lwIP, 只要通过 SPI 读写它的寄存器(建连接、
 * 填数据、取数据)即可。
 *
 * 硬件连接(引脚定义见 bsp_w5500.h):
 *   SPI2: SCK=PB13, MISO=PB14, MOSI=PB15 (全双工, 模式 0)
 *   CS  = PB12 (软件片选), RST = PB1 (低电平复位), INT = PB0 (本驱动用查询, 未接中断)
 *
 * 在本项目里的角色: 提供 4 个 TCP 服务端 Socket 监听 502~505 给上位机当
 * Modbus-TCP 服务端; Socket4(506) 单独做 OTA 升级数据通道。
 *
 * 关键概念:
 *   1) 地址 = 偏移地址 + 块号(BSB)。地址空间 16 位宽, 内部有多张表(通用寄存器 /
 *      每 Socket 寄存器 / TX 缓冲 / RX 缓冲), 靠 BSB 这 5 位决定访问哪张表。
 *   2) 每帧 SPI 先发 3 字节表头: [地址高][地址低][BSB<<3 | 读写位], 之后才是数据。
 *   3) Socket 收发缓冲是环形缓冲区, 指针会绕回, 故要与缓冲大小取模(& mask),
 *      一次传输可能拆成"绕回前一段 + 绕回后一段"两次搬运。
 */
#include "bsp_w5500.h"
#include "spi.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* 前置声明: w5500_wait_cr 定义在本文件后半部分, 但 SocketInit/Listen/Close 先用它 */
static void w5500_wait_cr(uint8_t block);

/*
 * SPI 单字节全双工读写。
 *
 * SPI 是全双工的——MOSI 移出 1 bit 的同时 MISO 移入 1 bit, 所以"发一字节"与
 * "收一字节"是同一次操作, 没有单独的读/写函数。W5500 也正靠此: 发地址给它,
 * 它同时把数据回给我们。
 *
 * 参数 data: 要发送的字节。返回: 同一时刻收到的字节。
 */
static uint8_t w5500_spi_rw(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, 10);
    return rx;
}

/* CS 拉低 = 开始与 W5500 通信。
 * SPI 总线上可挂多个从机, 靠各自 CS 区分; 拉低谁就是跟谁说话 */
static void w5500_cs_low(void)  { W5500_CS_LOW(); }

/* CS 拉高 = 结束本次通信, 释放总线。
 * 一次寄存器访问必须是"CS 拉低 → 发 4 字节 → CS 拉高"的完整过程, 中途被打断会破坏时序 */
static void w5500_cs_high(void) { W5500_CS_HIGH(); }

/*
 * 写 W5500 寄存器 (1 字节)。
 *
 * 一帧格式: [地址高字节][地址低字节][控制字节][数据字节]
 * 控制字节 = (块号 << 3) | 读写位; BIT2=1 写, =0 读。块号从 BIT3 起排, 故 <<3。
 * 例: 通用寄存器块(0) → 0x04; Socket0 寄存器块(1) → 0x0C。
 *
 * 参数 block: BSB 块号(W5500_BLOCK_COMMON / W5500_SOCK_REG(n) / ...)
 * 参数 offset: 块内偏移(如 W5500_VERSIONR = 0x0039); data: 要写的 1 字节
 */
void BSP_W5500_WriteReg(uint8_t block, uint16_t offset, uint8_t data)
{
    uint8_t control = (block << 3) | 0x04;   /* 0x04 = BIT2 置 1 = 写 */

    w5500_cs_low();                          /* 拉低片选, 选中 W5500 */
    w5500_spi_rw((uint8_t)(offset >> 8));    /* 第 1 字节: 地址高 8 位 */
    w5500_spi_rw((uint8_t)(offset & 0xFF));  /* 第 2 字节: 地址低 8 位 */
    w5500_spi_rw(control);                   /* 第 3 字节: 块号 + 写标志 */
    w5500_spi_rw(data);                      /* 第 4 字节: 真正要写的数据 */
    w5500_cs_high();                         /* 拉高片选, 结束这一帧 */
}

/*
 * 读 W5500 寄存器 (1 字节)。与写两点不同:
 *   1) 控制字节 BIT2 = 0, 即 (block << 3), 不 | 0x04;
 *   2) 第 4 字节发什么都被忽略, 取同时回给我们的字节, 故发 0x00。
 * 返回: 读到的 1 字节寄存器内容。
 */
uint8_t BSP_W5500_ReadReg(uint8_t block, uint16_t offset)
{
    uint8_t control = (block << 3) | 0x00;   /* BIT2 = 0 = 读 */
    uint8_t data;

    w5500_cs_low();
    w5500_spi_rw((uint8_t)(offset >> 8));    /* 地址高字节 */
    w5500_spi_rw((uint8_t)(offset & 0xFF));  /* 地址低字节 */
    w5500_spi_rw(control);                   /* 块号 + 读标志 */
    data = w5500_spi_rw(0x00);               /* 发哑元, 取回数据 */
    w5500_cs_high();
    return data;
}

/*
 * 连续写 W5500 缓冲区 (多字节)。
 * 表头同单字节写, 之后可连续发 len 个字节: W5500 内部地址自增, 不用重发地址。
 * 参数 data/len: 数据首地址和长度 (最长 65535)。
 */
void BSP_W5500_WriteBuf(uint8_t block, uint16_t offset, const uint8_t *data, uint16_t len)
{
    uint8_t control = (block << 3) | 0x04;

    w5500_cs_low();
    w5500_spi_rw((uint8_t)(offset >> 8));
    w5500_spi_rw((uint8_t)(offset & 0xFF));
    w5500_spi_rw(control);
    for (uint16_t i = 0; i < len; i++) {     /* 内部地址自动递增 */
        w5500_spi_rw(data[i]);
    }
    w5500_cs_high();                         /* 一次 CS 内完成整段搬运 */
}

/*
 * 连续读 W5500 缓冲区 (多字节), 与 WriteBuf 对称。
 * 参数 data/len: 接收缓冲首地址和要读的字节数(调用者保证空间足够)。
 */
void BSP_W5500_ReadBuf(uint8_t block, uint16_t offset, uint8_t *data, uint16_t len)
{
    uint8_t control = (block << 3) | 0x00;

    w5500_cs_low();
    w5500_spi_rw((uint8_t)(offset >> 8));
    w5500_spi_rw((uint8_t)(offset & 0xFF));
    w5500_spi_rw(control);
    for (uint16_t i = 0; i < len; i++) {     /* 每读一个发一次哑元 0x00 */
        data[i] = w5500_spi_rw(0x00);
    }
    w5500_cs_high();
}

/*
 * 读 16 位寄存器。W5500 的 16 位量(端口号 / 缓冲余量 / 收发指针)是大端存储,
 * 而 Cortex-M 是小端, 不能直接 memcpy, 必须手动拼: (高 << 8) | 低。
 * 复用 ReadBuf 连读 2 字节: buf[0]=高, buf[1]=低; W5500 要求这两字节连续读, 中间不能插入其他访问。
 */
uint16_t BSP_W5500_ReadReg16(uint8_t block, uint16_t offset)
{
    uint8_t buf[2];
    BSP_W5500_ReadBuf(block, offset, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

/* 写 16 位寄存器 (大端: 先发高字节) */
void BSP_W5500_WriteReg16(uint8_t block, uint16_t offset, uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xFF;          /* 高字节 */
    buf[1] = data & 0xFF;                 /* 低字节 */
    BSP_W5500_WriteBuf(block, offset, buf, 2);
}

/*
 * W5500 硬件复位, 两步:
 *   1) RST 脚拉低 ≥10ms 再拉高, 寄存器回默认值; 拉高后等 ~50ms 让 PLL/PHY 稳定;
 *   2) 往 MR 写 0x80(MR_RST) 做软件复位, 让协议栈内部状态机归零。
 * 两种都要: 硬件复位更彻底(连 PHY 一起), 软件复位更快且不依赖引脚。
 */
void BSP_W5500_Reset(void)
{
    W5500_RST_LOW();                                  /* 拉低 RST, 进入复位 */
    HAL_Delay(10);                                    /* 保持 ≥10ms (数据手册要求) */
    W5500_RST_HIGH();                                 /* 释放复位 */
    HAL_Delay(50);                                    /* 等内部电路稳定 */

    BSP_W5500_WriteReg(W5500_BLOCK_COMMON, W5500_MR, W5500_MR_RST);  /* 软件复位 */
    HAL_Delay(10);                                    /* 等复位完成 */
}

/*
 * 等待网线插好 / PHY 建立链路。
 * PHYCFGR 的 BIT0(LNK)=1 表示网线已与对端建立物理连接; 网线没插则后面建 Socket 必然失败。
 * 最多等 500×10ms = 5 秒, 超时就算了(不阻塞启动), 由后面读回 PHYCFGR 打印结果。
 */
static void w5500_wait_phy_link(void)
{
    uint8_t val;
    for (uint32_t i = 0; i < 500; i++) {                         /* 最多 500 轮 */
        val = BSP_W5500_ReadReg(W5500_BLOCK_COMMON, W5500_PHYCFGR);
        if (val & W5500_PHYCFGR_LNK) {                           /* BIT0=1, 链路已通 */
            break;
        }
        HAL_Delay(10);                                           /* 每轮等 10ms */
    }
}

/*
 * 一次性写入 MAC / IP / 子网掩码 / 网关。这四项在通用寄存器块里各占一段连续地址:
 *   SHAR0(0x0009) 起 6 字节 = MAC;  SIPR0(0x000F) 起 4 字节 = IP
 *   SUBR0(0x0005) 起 4 字节 = 掩码; GAR0 (0x0001) 起 4 字节 = 网关
 * 地址连续, 正好用 WriteBuf 一次搬完。参数是"字节数组", 按点分十进制顺序排,
 * 例如 192.168.1.10 → {192,168,1,10}。
 */
void BSP_W5500_SetNetwork(const uint8_t *mac, const uint8_t *ip,
                          const uint8_t *subnet, const uint8_t *gateway)
{
    BSP_W5500_WriteBuf(W5500_BLOCK_COMMON, W5500_SHAR0, mac, 6);      /* MAC  (6 字节) */
    BSP_W5500_WriteBuf(W5500_BLOCK_COMMON, W5500_SIPR0, ip, 4);       /* IP   (4 字节) */
    BSP_W5500_WriteBuf(W5500_BLOCK_COMMON, W5500_SUBR0, subnet, 4);   /* 掩码 (4 字节) */
    BSP_W5500_WriteBuf(W5500_BLOCK_COMMON, W5500_GAR0, gateway, 4);   /* 网关 (4 字节) */
}

/*
 * W5500 完整初始化:
 *   复位 → 关掉全部 8 个 Socket → 用 VERSIONR 自检 SPI → 写网络参数 →
 *   回读校验 → 复位 PHY → 等链路 → 打印 PHY 状态
 * 每步都 printf, 因为网络问题最难查("SPI 没通"/"网线没插"/"寄存器没写进去"),
 * 靠回读打印能一眼定位。
 * 参数 ip/subnet/gateway: 点分十进制拆成的 4 字节数组。
 */
void BSP_W5500_Init(const uint8_t *ip, const uint8_t *subnet, const uint8_t *gateway)
{
    printf("[W5500] Init start\r\n");

    BSP_W5500_Reset();                                        /* 步骤 1: 复位芯片 */
    printf("[W5500] Reset done\r\n");

    /* 步骤 2: 关掉所有 8 个 Socket。复位后理论都关着, 但热启动时可能有残留的
     * 未释放 Socket, 逐个发 CLOSE 更保险。 */
    for (uint8_t i = 0; i < 8; i++) {
        BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_CR, W5500_Sn_CR_CLOSE);
        w5500_wait_cr(W5500_SOCK_REG(i));
    }
    HAL_Delay(5);

    /* 步骤 3: 读版本寄存器自检。VERSIONR 固定为 0x04 (W5500 的"身份证");
     * 读出来不是 0x04 说明 SPI 通信有问题(接线错 / CS 不对 / SPI 模式时钟配错),
     * 这是判断"驱动通不通"最快的一招。 */
    uint8_t version = BSP_W5500_ReadReg(W5500_BLOCK_COMMON, W5500_VERSIONR);
    printf("[W5500] VERSIONR=0x%02X (expect 0x04)\r\n", version);
    if (version != 0x04) {
        printf("[W5500] ERROR: SPI communication failed!\r\n");
    }

    /* 步骤 4: 写网络参数。MAC 用本地管理地址格式(第 1 字节 BIT1 置 1, 即 0x02),
     * 避免与真实厂商 MAC 冲突; 同一网络里 MAC 不能重复。 */
    uint8_t mac[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    BSP_W5500_SetNetwork(mac, ip, subnet, gateway);

    /* 步骤 5: 回读校验, 确认写入真的生效(写进去不代表芯片认了) */
    uint8_t read_mac[6], read_ip[4], read_sub[4], read_gw[4];
    BSP_W5500_ReadBuf(W5500_BLOCK_COMMON, W5500_SHAR0, read_mac, 6);
    BSP_W5500_ReadBuf(W5500_BLOCK_COMMON, W5500_SIPR0, read_ip, 4);
    BSP_W5500_ReadBuf(W5500_BLOCK_COMMON, W5500_SUBR0, read_sub, 4);
    BSP_W5500_ReadBuf(W5500_BLOCK_COMMON, W5500_GAR0, read_gw, 4);
    printf("[W5500] MAC readback: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           read_mac[0], read_mac[1], read_mac[2], read_mac[3], read_mac[4], read_mac[5]);
    printf("[W5500] IP readback: %d.%d.%d.%d\r\n", read_ip[0], read_ip[1], read_ip[2], read_ip[3]);
    printf("[W5500] SUB readback: %d.%d.%d.%d\r\n", read_sub[0], read_sub[1], read_sub[2], read_sub[3]);
    printf("[W5500] GW readback: %d.%d.%d.%d\r\n", read_gw[0], read_gw[1], read_gw[2], read_gw[3]);

    /* 步骤 6: 复位 PHY(负责网线上的实际电气信号)。置 PHYCFGR 的 BIT7(RST)=1
     * 复位, 再写 0x00 释放。换网线、链路不稳时这步常能修好。 */
    printf("[W5500] PHY reset...\r\n");
    BSP_W5500_WriteReg(W5500_BLOCK_COMMON, W5500_PHYCFGR, W5500_PHYCFGR_RST);
    HAL_Delay(1);
    BSP_W5500_WriteReg(W5500_BLOCK_COMMON, W5500_PHYCFGR, 0x00);
    HAL_Delay(1);

    /* 步骤 7: 等网线链路建立 */
    printf("[W5500] Wait PHY link...\r\n");
    w5500_wait_phy_link();

    /* 步骤 8: 读 PHYCFGR 打印自协商结果: BIT0 LNK(1=已连接) / BIT1 SPD(1=100M) / BIT2 DPX(1=全双工) */
    uint8_t phycfgr = BSP_W5500_ReadReg(W5500_BLOCK_COMMON, W5500_PHYCFGR);
    printf("[W5500] PHYCFGR=0x%02X LNK=%d SPD=%d DPX=%d\r\n",
           phycfgr,
           (phycfgr & W5500_PHYCFGR_LNK) ? 1 : 0,      /* BIT0 */
           (phycfgr >> 1) & 0x01,                      /* BIT1 */
           (phycfgr >> 2) & 0x01);                     /* BIT2 */

    /* Socket0 此时还没 OPEN, 读一下状态寄存器做参考, 应该是 CLOSED(0x00) */
    uint8_t sr = BSP_W5500_ReadReg(W5500_BLOCK_S0, W5500_Sn_SR);
    printf("[W5500] S0_SR=0x%02X\r\n", sr);
    printf("[W5500] Init done\r\n");
}

/*
 * 初始化一个 Socket (打开并绑定端口)。Socket 是 W5500 里"一条网络连接"的抽象,
 * 8 个 Socket 各干各的(本项目 0~3 做 4 路 Modbus-TCP, 4 号做 OTA)。
 * 步骤:
 *   1) 清中断标志: 往 Sn_IR 写 0xFF(写 1 清 0), 防旧标志干扰;
 *   2) 设工作模式: Sn_MR 写协议类型(TCP=0x01);
 *   3) 绑本地端口: 写 Sn_PORT0/1 (16 位);
 *   4) 发 OPEN 命令: 往 Sn_CR 写 0x01。
 * 注意端口和模式必须在 OPEN 之前设好, 顺序反了就绑不上。
 * 参数 sn: 0~7; protocol: 本项目传 W5500_Sn_MR_TCP; port: 502~505 / OTA 506。
 */
void BSP_W5500_SocketInit(uint8_t sn, uint8_t protocol, uint16_t port)
{
    uint8_t block = W5500_SOCK_REG(sn);      /* 换算成该 Socket 的寄存器块号 */

    BSP_W5500_WriteReg(block, W5500_Sn_IR, 0xFF);   /* 1) 写 1 清掉全部中断标志 */

    BSP_W5500_WriteReg(block, W5500_Sn_MR, protocol);  /* 2) 设置协议模式 (TCP=0x01) */

    BSP_W5500_WriteReg16(block, W5500_Sn_PORT0, port);  /* 3) 设置本地端口 (16 位) */

    BSP_W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_OPEN);  /* 4) 发 OPEN 命令 */
    w5500_wait_cr(block);
}

/*
 * 让 Socket 进入监听 (TCP 服务端)。必须先 SocketInit 完成 OPEN 再发 LISTEN;
 * 发完后该 Socket 等待客户端三次握手, 成功后状态自动变 ESTABLISHED(0x17)。
 */
void BSP_W5500_SocketListen(uint8_t sn)
{
    uint8_t block = W5500_SOCK_REG(sn);
    BSP_W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_LISTEN);
    w5500_wait_cr(block);
}

/*
 * 关闭 Socket (释放连接、回收缓冲区)。CLOSE 终止连接并让 Socket 回 CLOSED,
 * 之后可重新 Init 复用。正常应先 DISCON 再 CLOSE, 这里 CLOSE 也能强制收回。
 */
void BSP_W5500_SocketClose(uint8_t sn)
{
    uint8_t block = W5500_SOCK_REG(sn);
    BSP_W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_CLOSE);
    w5500_wait_cr(block);
}

/*
 * 主动断开连接 (走 TCP 正常挥手流程)。与 CLOSE 的区别: DISCON 按协议发 FIN 包,
 * 优雅断开, 对端能感知到"连接被正常关闭"。
 */
void BSP_W5500_SocketDisconnect(uint8_t sn)
{
    uint8_t block = W5500_SOCK_REG(sn);
    BSP_W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_DISCON);
    w5500_wait_cr(block);
}

/*
 * 读 Socket 当前状态 (即 Sn_SR)。常见值见 bsp_w5500.h:
 *   0x00 CLOSED / 0x13 INIT / 0x14 LISTEN / 0x17 ESTABLISHED / 0x1C CLOSE_WAIT。
 * 上层任务靠轮询这个值判断"客户端连上没""对方断开没"。
 */
uint8_t BSP_W5500_SocketStatus(uint8_t sn)
{
    uint8_t block = W5500_SOCK_REG(sn);
    return BSP_W5500_ReadReg(block, W5500_Sn_SR);
}

/*
 * 查询 Socket 里"已收到、还没被取走"的字节数。
 * Sn_RX_RSR 由硬件维护: 收到数据就加, 我们 RECV 取走后减。
 * 上层通常先查它 >0, 有货了再调 SocketRecv 取。
 */
uint16_t BSP_W5500_SocketRxAvail(uint8_t sn)
{
    uint8_t block = W5500_SOCK_REG(sn);
    return BSP_W5500_ReadReg16(block, W5500_Sn_RX_RSR0);   /* 16 位, 大端 */
}

/*
 * 查询 Socket 发送缓冲区里还能塞多少字节 (Sn_TX_FSR)。
 * 要发 len 字节必须等 FSR >= len 才能发, 否则丢数据; SocketSend 里就是这么做的。
 */
uint16_t BSP_W5500_SocketTxFree(uint8_t sn)
{
    uint8_t block = W5500_SOCK_REG(sn);
    return BSP_W5500_ReadReg16(block, W5500_Sn_TX_FSR0);   /* 16 位, 大端 */
}

/*
 * 等待 Socket 命令执行完毕。Sn_CR 是命令寄存器: 写入命令码后芯片开始执行,
 * 执行完硬件自动清成 0x00, 所以"轮询等它变 0"就等于"等这条命令做完"。
 * 最长等 1000 次(每次一次 SPI 读, 约几十微秒), 超时就返回, 防芯片异常时死等卡死任务。
 */
static void w5500_wait_cr(uint8_t block)
{
    for (uint16_t i = 0; i < 1000; i++) {
        if (BSP_W5500_ReadReg(block, W5500_Sn_CR) == 0x00) {  /* 命令被清 0 = 完成 */
            break;
        }
    }
}

/*
 * 通过 Socket 发送数据。核心是"环形缓冲区 + 写指针"模型:
 *   Sn_TX_WR  = 软件写指针(下一次该往哪写, 由我们维护);
 *   Sn_TX_FSR = 剩余空闲空间(硬件维护)。
 * 流程:
 *   1) 等空闲空间 >= 要发长度(最多 2000×1ms), 不够就报错返回;
 *   2) 物理偏移 = 写指针 & mask(mask = 缓冲区大小-1, 按位与代替取模);
 *   3) 环形容不下则先写"到尾部这一段", 剩余从头部 0 续写(绕回);
 *   4) 更新写指针(加完整长度, 指针可超出缓冲区, 靠 mask 取模);
 *   5) 发 SEND 命令, 芯片打包成 TCP 包发出;
 *   6) 等 Sn_IR 的 SEND_OK(成功) / TIMEOUT(超时) 标志。
 * 返回: 0=成功, -1=没有空闲空间(丢包), -2=发送超时。
 */
int8_t BSP_W5500_SocketSend(uint8_t sn, const uint8_t *data, uint16_t len)
{
    if (len == 0) return 0;                  /* 空包直接返回, 免得白跑流程 */

    uint8_t block = W5500_SOCK_REG(sn);      /* Socket 寄存器块 */
    uint8_t tx_block = W5500_TX_BLOCK(sn);   /* Socket TX 缓冲区块 */

    /* 步骤 1: 等缓冲区腾出足够空间。TCP 可靠传输不能硬塞, 必须等 FSR >= len;
     * 最多等 2 秒, 超时报错让上层重试/丢弃 */
    for (uint16_t i = 0; i < 2000; i++) {
        if (BSP_W5500_ReadReg16(block, W5500_Sn_TX_FSR0) >= len) break;
        osDelay(1);
    }
    if (BSP_W5500_ReadReg16(block, W5500_Sn_TX_FSR0) < len) {            /* 再查一次判超时 */
        printf("[W5500] TX no free space!\r\n");
        return -1;
    }

    /* 步骤 2: 读写指针并算出物理偏移 */
    uint16_t tx_wr = BSP_W5500_ReadReg16(block, W5500_Sn_TX_WR0);   /* 逻辑写指针 */
    uint16_t tx_size = (uint16_t)BSP_W5500_ReadReg(block, W5500_Sn_TXBUF_SIZE) * 1024;  /* 字节数 */
    uint16_t tx_mask = tx_size - 1;    /* 缓冲区是 2 的幂(1/2/4/8/16KB), 减 1 得掩码 */
    uint16_t offset = tx_wr & tx_mask;

    /* 步骤 3: 算"从 offset 到缓冲区末尾"能写多少 */
    uint16_t first = tx_size - offset;
    if (first > len) first = len;         /* 尾部空间够则一次写完 */

    BSP_W5500_WriteBuf(tx_block, offset, data, first);   /* 先写第一段(不绕回) */
    if (len > first) {                                   /* 剩下的要绕回 */
        BSP_W5500_WriteBuf(tx_block, 0, data + first, len - first);  /* 从头部写 */
    }

    /* 步骤 4: 更新写指针。加的是完整 len(不是 first), 指针可超出缓冲区,
     * 下次用时靠 mask 取模, 这样能区分"逻辑上第几圈" */
    tx_wr += len;
    BSP_W5500_WriteReg16(block, W5500_Sn_TX_WR0, tx_wr);

    /* 步骤 5: 发 SEND 命令, 通知芯片"数据已就位" */
    BSP_W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_SEND);
    w5500_wait_cr(block);

    /* 步骤 6: 等发送结果中断标志(写 1 清标志, 免得影响下次判断) */
    int8_t result = 0;
    for (uint16_t i = 0; i < 500; i++) {
        uint8_t ir = BSP_W5500_ReadReg(block, W5500_Sn_IR);

        if (ir & W5500_Sn_IR_SEND_OK) {                       /* BIT4: 发送成功 */
            BSP_W5500_WriteReg(block, W5500_Sn_IR, W5500_Sn_IR_SEND_OK);
            result = 0;
            break;
        }
        if (ir & W5500_Sn_IR_TIMEOUT) {                       /* BIT3: 发送超时 */
            BSP_W5500_WriteReg(block, W5500_Sn_IR, W5500_Sn_IR_TIMEOUT);
            printf("[W5500] Send TIMEOUT!\r\n");
            result = -2;
            break;
        }
        osDelay(1);
    }

    if (result != 0) {
        printf("[W5500] Send failed, result=%d\r\n", result);
    }

    return result;
}

/*
 * 从 Socket 接收数据, 与发送对称, 但是"读指针"模型:
 *   Sn_RX_RSR 硬件维护: 已收到多少字节还没被取走;
 *   Sn_RX_RD  软件维护: 我们读到哪了;
 *   取完数据必须发 RECV 命令, 芯片才把这块空间还给硬件。
 * 流程:
 *   1) 查 RX_RSR, 为 0 直接返回 0;
 *   2) 调用者缓冲比实际数据小则只取这么多(len 收敛到 rx_rsr);
 *   3) 用读指针算物理偏移, 分"绕回前/绕回后"两段读出;
 *   4) 更新读指针, 发 RECV 命令释放 RX 空间。
 * 返回: 实际读到的字节数(需完整报文时由上层自行攒够再解析)。
 */
uint16_t BSP_W5500_SocketRecv(uint8_t sn, uint8_t *buf, uint16_t len)
{
    uint8_t block = W5500_SOCK_REG(sn);       /* Socket 寄存器块 */
    uint8_t rx_block = W5500_RX_BLOCK(sn);    /* Socket RX 缓冲区块 */

    /* 步骤 1: 看有没有数据 (RX_RSR 是 16 位"待取字节数") */
    uint16_t rx_rsr = BSP_W5500_ReadReg16(block, W5500_Sn_RX_RSR0);
    if (rx_rsr == 0) return 0;

    /* 步骤 2: 不能读超过实际数据的量, 否则会读到垃圾 */
    if (len > rx_rsr) len = rx_rsr;

    /* 步骤 3: 读指针 → 物理偏移, 分两段搬(处理环形绕回) */
    uint16_t rx_rd = BSP_W5500_ReadReg16(block, W5500_Sn_RX_RD0);
    uint16_t rx_size = (uint16_t)BSP_W5500_ReadReg(block, W5500_Sn_RXBUF_SIZE) * 1024;
    uint16_t rx_mask = rx_size - 1;
    uint16_t offset = rx_rd & rx_mask;

    uint16_t first = rx_size - offset;   /* 这一段能读到缓冲区尾部 */
    if (first > len) first = len;

    BSP_W5500_ReadBuf(rx_block, offset, buf, first);        /* 读第一段 */
    if (len > first) {                                      /* 需要绕回 */
        BSP_W5500_ReadBuf(rx_block, 0, buf + first, len - first);  /* 从头部读剩余 */
    }

    /* 步骤 4: 推进读指针, 并通知芯片"我取走了" */
    rx_rd += len;
    BSP_W5500_WriteReg16(block, W5500_Sn_RX_RD0, rx_rd);
    BSP_W5500_WriteReg(block, W5500_Sn_CR, W5500_Sn_CR_RECV);
    w5500_wait_cr(block);

    return len;
}
