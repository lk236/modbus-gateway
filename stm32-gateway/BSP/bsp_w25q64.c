/*
 * bsp_w25q64.c — W25Q64 SPI Flash 存储驱动
 * ============================================================================
 * 【这是什么芯片】
 *   W25Q64 是华邦(Winbond)的 NOR Flash, 容量 64Mbit = 8MB。
 *   通过 SPI1 访问: SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA4 (软件片选)。
 *   特点: 掉电不丢数据, 但"写"很麻烦——这就是本文件大部分代码在处理的复杂度。
 *
 * 【NOR Flash 的 4 条铁律】
 *   1) 只能"把 1 写成 0", 不能"把 0 写成 1"。
 *      芯片里的存储单元默认(擦除后)是 1。编程(写)只能把某些位拉低成 0。
 *      所以想把某位从 0 变回 1, 必须整片/整扇区擦除。
 *   2) 擦除的最小单位是"扇区"(4KB), 不是字节。不能只擦 1 个字节。
 *      擦除后整个扇区的 4KB 全部变成 0xFF。
 *   3) 写之前必须先"写使能"(发 0x06 命令), 否则编程/擦除命令会被忽略。
 *      写使能是一个"一次性"的锁存标志(WEL), 命令发完后自动清零。
 *   4) 每条 擦除/编程 命令都是"异步"的: 命令发出后芯片需要几十微秒~几百毫秒
 *      自己去忙, 期间必须轮询状态寄存器的 BUSY 位等它变 0, 否则后续命令失效。
 *
 * 【本项目怎么用这 8MB】
 *   0x000000 ~ 0x000FFF  配置扇区  —— 存网关参数(IP/掩码/端口/波特率...)
 *   0x001000 ~ 0x001FFF  日志元数据 —— 存"日志总条数 + 环形写指针"
 *   0x002000 ~ 0x7FFFFF  日志数据区 —— 环形缓冲区, 最多 2048 条记录
 *
 * 【并发问题】
 *   本项目有多个 FreeRTOS 任务都会读写这片 Flash (日志任务、Modbus 任务、
 *   配置任务)。Flash 是有状态的硬件, 两个任务交叉操作(比如 A 刚发完写使能,
 *   B 插进来读了状态, A 再写就失败了), 会互相破坏。
 *   解决办法: 用一把互斥锁 g_w25q64_mutex 把所有 Flash 访问串行化。
 *   锁本身不可重入, 所以文件里区分了两套内部函数:
 *     - xxx()        : 加锁版, 给外部调用
 *     - xxxNoLock()  : 不加锁版, 只能在已持锁的临界区内调用
 * ============================================================================
 */
#include "bsp_w25q64.h"
#include <string.h>
#include <stdio.h>

/* 全局互斥锁: 保护整片 Flash 的访问串行化。在 BSP_W25Q64_Init 里创建 */
static osMutexId_t g_w25q64_mutex;

/* ---- 日志环形缓冲区的三个运行时状态 ---- */
static uint16_t g_log_write_index;   /* 环形写指针: 下一条要写的物理槽位(0 ~ 条数上限-1) */
static uint16_t g_log_count;         /* 当前有效日志条数 (封顶 = 上限) */
static uint8_t  g_log_meta_dirty;    /* 元数据是否需要回写标志 (预留字段, 当前实现每写必回写) */

/* 一个 4KB 扇区能存多少条日志: 4096 / sizeof(W25Q64_LogEntry)=8 → 512 条。
 * 用于把"条目索引"换算成"它属于哪个扇区" */
#define LOG_ENTRIES_PER_SECTOR  (W25Q64_SECTOR_SIZE / sizeof(W25Q64_LogEntry))

/* ---------------------------------------------------------------------------
 * CRC32 查表 (多项式 0xEDB88320, 即反射后的 CRC-32/ISO-HDLC, 与以太网/zip 同款)
 *
 * 为什么用"查表法"而不是逐位计算?
 *   逐位算 CRC 每字节要循环 8 次, 速度慢。预先算好 256 个中间值存在表里,
 *   每字节只需: 1 次异或 + 1 次查表 + 1 次移位, 快得多。
 *   代价是占用 1KB ROM / 或 RAM (这里是 const, 放在 Flash 里)。
 *
 * 用途: 给配置文件做完整性校验, 防止 Flash 里的数据被写坏/读错后当成功效配置用。
 * --------------------------------------------------------------------------- */
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/* ---------------------------------------------------------------------------
 * 按查表法计算 CRC32
 *
 * 算法三步 (这是 CRC 的标准套路):
 *   1) 初值取 0xFFFFFFFF (而不是 0), 让开头几个字节也能被充分打散。
 *   2) 每个字节: 用 (crc ^ 当前字节) 的低 8 位去查表, 结果异或上 crc 右移 8 位。
 *      含义是"把已经算好的 crc 按位继续推进", 查表只是把 8 次移位异或压缩成 1 次。
 *   3) 最终结果再取反 (~), 得到标准 CRC32。
 *
 * 参数 buf/len: 数据首地址与长度
 * 返回值: 32 位 CRC 值
 * --------------------------------------------------------------------------- */
static uint32_t BSP_W25Q64_CRC32(uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;                            /* 1) 初值全 1 */

    for (uint32_t i = 0; i < len; i++) {
        /* crc 低 8 位与数据字节异或 → 作为查表下标(所以 &0xFF);
         * 查表得到"这 8 位的效果", 再与 crc>>8 异或, 相当于整体右移 8 位继续推进 */
        crc = crc32_table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
    }

    return crc ^ 0xFFFFFFFF;                              /* 3) 末尾取反 */
}

/* ---------------------------------------------------------------------------
 * 加锁 / 解锁: 对 Flash 硬件访问做互斥
 *
 * 为什么需要? 见文件头"并发问题"。多个任务若交叉访问, 会破坏 Flash 命令时序。
 * 为什么判断 != NULL? Init 早期锁可能还没创建, 此时(单任务阶段)直接放行,
 * 避免空指针解引用崩溃。
 * --------------------------------------------------------------------------- */
static void BSP_W25Q64_Lock(void)
{
    if (g_w25q64_mutex != NULL) {
        osMutexAcquire(g_w25q64_mutex, osWaitForever);   /* 永久等待, 直到拿到锁 */
    }
}

static void BSP_W25Q64_Unlock(void)
{
    if (g_w25q64_mutex != NULL) {
        osMutexRelease(g_w25q64_mutex);                  /* 释放锁, 唤醒等待者 */
    }
}

/* ---------------------------------------------------------------------------
 * SPI 收发一个字节 (全双工)
 *
 * 用 HAL_SPI_TransmitReceive 而不是分别 Transmit/Receive: SPI 没有独立的读操作,
 * 读数据本质是"发一个哑元字节, 同时把它移回来的数据收下", 一次调用最省事。
 *
 * 异常处理: 传输失败返回 0xFF。0xFF 在 Flash 里代表"擦除态", 是个中性的安全值。
 * --------------------------------------------------------------------------- */
static uint8_t BSP_W25Q64_SPI_SendByte(uint8_t byte)
{
    uint8_t rx;     /* 同时收到的字节 */
    if (HAL_SPI_TransmitReceive(&hspi1, &byte, &rx, 1, 100) != HAL_OK) {  /* 超时 100ms */
        return 0xFF;                        /* 失败: 返回擦除态值 */
    }
    return rx;
}

/* 只发数据不关心回读 (用于页编程里把一整页数据推出去)。超时 500ms */
static W25Q64_Status BSP_W25Q64_SPI_Transmit(uint8_t *tx, uint16_t len)
{
    if (HAL_SPI_Transmit(&hspi1, tx, len, 500) != HAL_OK) {
        return W25Q64_ERROR;
    }
    return W25Q64_OK;
}

/* 只收数据 (用于读命令下发后连续取数据)。超时 500ms */
static W25Q64_Status BSP_W25Q64_SPI_Receive(uint8_t *rx, uint16_t len)
{
    if (HAL_SPI_Receive(&hspi1, rx, len, 500) != HAL_OK) {
        return W25Q64_ERROR;
    }
    return W25Q64_OK;
}

/* ---------------------------------------------------------------------------
 * 等待芯片空闲 (轮询状态寄存器1的 BUSY 位)
 *
 * 原理: 任何 擦除/编程 命令发出后, 芯片的 SR1.BUSY(BIT0) 会置 1,
 *       表示"我还在忙, 别来打扰"。完成后自动变 0。
 *       所以"读 SR1 直到 BUSY=0"就是"等这笔操作真正落地"。
 *
 * 读 SR1 的动作: CS 拉低 → 发命令 0x05 → 连续读状态字节 → CS 拉高,
 * 是一笔完整的独立 SPI 事务(每次都要重新拉 CS)。
 *
 * 参数 timeout_ms: 超时上限(毫秒)。扇区擦除约 400ms, 全片擦除约 60s, 所以调用方
 *                  给的超时值不同 (见下面各函数)。
 * 返回值: W25Q64_OK 表示等到了空闲; W25Q64_TIMEOUT 表示超时(芯片可能坏了)
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_WaitBusy(uint32_t timeout_ms)
{
    uint8_t sr;                                              /* 状态寄存器内容 */
    uint32_t start = osKernelGetTickCount();                 /* 记录起始 tick, 算超时 */

    do {
        W25Q64_CS_LOW();                                     /* 选中芯片 */
        BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_STATUS1);    /* 命令: 读状态寄存器1 */
        sr = BSP_W25Q64_SPI_SendByte(0xFF);                  /* 读回 1 字节状态 */
        W25Q64_CS_HIGH();                                    /* 释放芯片 */

        if (!(sr & W25Q64_SR1_BUSY)) {                       /* BIT0=0 → 不忙了 */
            return W25Q64_OK;
        }
        osDelay(1);                                          /* 忙就等 1ms 再查(让出 CPU) */

    } while ((osKernelGetTickCount() - start) < timeout_ms);  /* 未超时就继续 */

    return W25Q64_TIMEOUT;                                   /* 超时 */
}

/* ---------------------------------------------------------------------------
 * 发送"写使能"命令, 并回读状态确认 WEL 已置位
 *
 * WEL (Write Enable Latch) 是 Flash 内部的一把"写锁":
 *   只有它 = 1 时, 后面的 页编程(0x02)/扇区擦除(0x20) 等命令才会被执行;
 *   否则命令直接被忽略(而且不报错! 这是最坑的地方——代码看起来成功, 数据没写进去)。
 *   每次 编程/擦除 完成后, WEL 自动清零, 所以每次操作前都要重新发 0x06。
 *
 * 这里多做一步"回读确认": 发完 0x06 再读 SR1, 检查 WEL(BIT1)=1。
 * 为什么要确认? 因为有些情况下(上一次操作还没结束、时序被打断)写使能会失败,
 * 提前发现比写进去发现没数据要好。
 *
 * 返回值: W25Q64_OK = 写使能成功; W25Q64_ERROR = WEL 没置起来
 * --------------------------------------------------------------------------- */
static W25Q64_Status BSP_W25Q64_WriteEnable(void)
{
    W25Q64_CS_LOW();                                          /* 独立事务 1: 发写使能 */
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_WRITE_ENABLE);         /* 命令 0x06 */
    W25Q64_CS_HIGH();

    W25Q64_CS_LOW();                                          /* 独立事务 2: 回读验证 */
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_STATUS1);         /* 命令 0x05 */
    uint8_t sr = BSP_W25Q64_SPI_SendByte(0xFF);               /* 读状态 */
    W25Q64_CS_HIGH();

    if (sr & W25Q64_SR1_WEL) {                                /* BIT1=1 → 写使能已生效 */
        return W25Q64_OK;
    }
    return W25Q64_ERROR;                                      /* 没生效, 上报失败 */
}

/* ---------------------------------------------------------------------------
 * 不带锁版本的写使能 (逻辑与上面完全相同)
 *
 * 存在的意义: 供"已经在临界区内(已持锁)"的内部函数调用。
 * FreeRTOS 的普通互斥锁不可重入——同一个任务再次 Acquire 同一把锁会死锁,
 * 所以必须提供 NoLock 版本给内部用。这是嵌入式里很常见的写法。
 * --------------------------------------------------------------------------- */
static W25Q64_Status BSP_W25Q64_WriteEnableNoLock(void)
{
    W25Q64_CS_LOW();                                          /* 发写使能 */
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_WRITE_ENABLE);
    W25Q64_CS_HIGH();

    W25Q64_CS_LOW();                                          /* 回读 WEL */
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_STATUS1);
    uint8_t sr = BSP_W25Q64_SPI_SendByte(0xFF);
    W25Q64_CS_HIGH();

    if (sr & W25Q64_SR1_WEL) {
        return W25Q64_OK;
    }
    return W25Q64_ERROR;
}

/* ---------------------------------------------------------------------------
 * 不带锁版本的"任意地址读"
 *
 * Flash 读取的三段式命令: [命令 0x03][24 位地址 高→低][之后连续读出数据]
 *   - 地址是 24 位, 因为 8MB = 0x800000, 需要 3 字节表示
 *   - 地址发完立刻可以连续读, 芯片内部地址自动递增, 没有"页"限制
 *     (这是"读"和"写"最大的区别: 读可以任意长度跨页, 写不行)
 *
 * 参数检查: 地址不能越界, 且 addr+len 也不能越过芯片末尾 (8MB)。
 *           注意此处的加法要用 32 位, 否则会溢出误判。
 * 返回值: W25Q64_OK / W25Q64_INVALID(参数非法) / HAL 传输错误
 * --------------------------------------------------------------------------- */
static W25Q64_Status BSP_W25Q64_ReadNoLock(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (addr >= W25Q64_CAPACITY || len == 0 || (addr + len) > W25Q64_CAPACITY) return W25Q64_INVALID;
    if (buf == NULL) return W25Q64_INVALID;                   /* 空指针保护 */

    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_DATA);            /* 命令 0x03: 读数据 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);             /* 地址第 1 字节(最高位) */
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);              /* 地址第 2 字节 */
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);                     /* 地址第 3 字节(最低位) */
    W25Q64_Status ret = BSP_W25Q64_SPI_Receive(buf, (uint16_t)len);  /* 连续读出 len 字节 */
    W25Q64_CS_HIGH();
    return ret;
}

/* ---------------------------------------------------------------------------
 * 不带锁版本的"扇区擦除" (擦除 4KB)
 *
 * 注意: 传入的 addr 只要落在这个扇区内, 芯片就会把包含它的整个 4KB 扇区擦掉,
 *       地址不需要对齐到 4096 边界(但推荐对齐, 语义更清楚)。
 *
 * 流程: 写使能 → 发擦除命令+地址 → 等 BUSY 清零(约 400ms)
 * --------------------------------------------------------------------------- */
static W25Q64_Status BSP_W25Q64_SectorEraseNoLock(uint32_t addr)
{
    W25Q64_Status ret;

    if (addr >= W25Q64_CAPACITY) return W25Q64_INVALID;       /* 地址越界检查 */

    ret = BSP_W25Q64_WriteEnableNoLock();                     /* 擦除前必须先写使能 */
    if (ret != W25Q64_OK) return ret;                         /* 写使能失败就别发命令了 */

    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_SECTOR_ERASE);         /* 命令 0x20: 扇区擦除 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);             /* 24 位地址 3 字节 */
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);
    W25Q64_CS_HIGH();

    return BSP_W25Q64_WaitBusy(1000);                         /* 等擦除完成, 最长 1 秒 */
}

/* ---------------------------------------------------------------------------
 * 不带锁版本的"页编程" (一次最多写 256 字节, 且不能跨页)
 *
 * 【关键约束】一次 Page Program 写的这 len 个字节, 必须全部落在同一个 256 字节
 *   页内。如果跨越了页边界, 芯片的地址在页尾不会进到下一页, 而是"回卷"到本页开头,
 *   把开头的旧数据覆盖掉 —— 数据就乱了。
 *   所以调用者必须保证不跨页; BSP_W25Q64_Write 就是专门负责拆页的。
 *
 * 流程: 检查参数 → 写使能 → 发命令+地址 → 连续发数据 → 等 BUSY 清零(约几毫秒)
 *
 * 参数 len: 必须 <= 256 (W25Q64_PAGE_SIZE)
 * --------------------------------------------------------------------------- */
static W25Q64_Status BSP_W25Q64_PageWriteNoLock(uint32_t addr, uint8_t *buf, uint16_t len)
{
    W25Q64_Status ret;

    if (addr >= W25Q64_CAPACITY || len == 0 || len > W25Q64_PAGE_SIZE) return W25Q64_INVALID;
    if (buf == NULL) return W25Q64_INVALID;

    ret = BSP_W25Q64_WriteEnableNoLock();                     /* 写使能 */
    if (ret != W25Q64_OK) return ret;

    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_PAGE_PROGRAM);         /* 命令 0x02: 页编程 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);             /* 24 位地址 3 字节 */
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);
    ret = BSP_W25Q64_SPI_Transmit(buf, len);                  /* 连续发 len 字节数据 */
    W25Q64_CS_HIGH();

    if (ret != W25Q64_OK) return ret;
    return BSP_W25Q64_WaitBusy(100);                          /* 等写入完成, 最长 100ms */
}

/* ---------------------------------------------------------------------------
 * Flash 初始化
 *
 * 做四件事:
 *   1) 创建互斥锁 (必须最先做, 后面所有访问都要用它)
 *   2) CS 拉高 + 延时, 让总线进入空闲态
 *   3) 读 JEDEC ID 自检: 厂商码 0xEF(华邦) + 设备码 0x4017(W25Q64) 才对
 *   4) 从 Flash 恢复日志状态(条数/写指针), 并"校准"条数
 *
 * 【第 4 步为什么要校准?】
 *   日志区是环形缓冲区, 元数据里存的"条数"是软件记的, 可能因为掉电时机不对
 *   而偏大(比如元数据写成功了但日志数据没写完)。所以上电时从最新一条往前数,
 *   数到第一条无效记录为止, 用实际有效条数覆盖元数据里的值, 保证不会读到垃圾。
 *
 * 返回值: W25Q64_OK=成功; W25Q64_ERROR=锁创建失败或读 ID 失败; W25Q64_INVALID=ID 不对
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_Init(void)
{
    uint8_t manuf;        /* 厂商 ID */
    uint16_t dev_id;      /* 设备 ID */

    /* 1) 创建互斥锁。osMutexPrioInherit = 优先级继承,
     *    防止"低优先级任务持锁时被中优先级任务抢占"导致的优先级反转问题。 */
    osMutexAttr_t mutex_attr = { NULL, osMutexPrioInherit, NULL, 0 };
    g_w25q64_mutex = osMutexNew(&mutex_attr);
    if (g_w25q64_mutex == NULL) {
        return W25Q64_ERROR;
    }

    /* 2) 总线置空闲: CS 拉高(不选中) + 短延时 */
    W25Q64_CS_HIGH();
    osDelay(1);

    /* 3) 自检: 读 JEDEC ID (命令 0x9F, 返回 厂商码 + 设备码 2 字节) */
    if (BSP_W25Q64_ReadJEDEC_ID(&manuf, &dev_id) != W25Q64_OK) {
        return W25Q64_ERROR;
    }

    if (manuf != 0xEF || dev_id != 0x4017) {      /* 华邦 + W25Q64 的固定组合 */
        return W25Q64_INVALID;                    /* ID 不对: 接线/型号有问题 */
    }

    /* 先给运行时状态一个干净的初值 */
    g_log_write_index = 0;
    g_log_count = 0;
    g_log_meta_dirty = 0;

    /* 4a) 读日志元数据扇区的前 8 字节, 恢复条数与写指针。
     *     元数据格式: [0..1]保留(0xFF) [2..3]条数(大端) [4..5]写指针(大端) */
    {
        uint8_t meta[8];
        if (BSP_W25Q64_Read(W25Q64_LOG_META_ADDR, meta, 8) == W25Q64_OK) {   /* 读元数据 */
            uint16_t stored_count = (meta[2] << 8) | meta[3];                /* 拼出条数 */
            uint16_t stored_index = (meta[4] << 8) | meta[5];                /* 拼出写指针 */
            /* 只接受"合理值"(不超过上限), 防止读到乱码把缓冲区搞乱 */
            if (stored_count <= W25Q64_LOG_ENTRY_MAX &&
                stored_index <= W25Q64_LOG_ENTRY_MAX) {
                g_log_count = stored_count;
                g_log_write_index = stored_index;
            }
        }
    }

    /* 4b) 校准: 从最新一条(写指针位置)往回逐条读, 数出真正有效的条数。
     *     合法条目的判定: type 在枚举范围内, 且 timestamp != 0xFFFFFFFF(擦除态)。 */
    {
        uint16_t valid = 0;                                     /* 实际有效条数 */
        uint16_t idx = g_log_write_index;                       /* 从写指针处往回走 */
        /* 最多回溯 g_log_count 条(再多的不算数), 也不超过物理上限 */
        uint16_t scan_max = (g_log_count < W25Q64_LOG_ENTRY_MAX) ? g_log_count : W25Q64_LOG_ENTRY_MAX;

        for (uint16_t i = 0; i < scan_max; i++) {
            /* 算出第 idx 个槽位的物理地址 */
            uint32_t addr = W25Q64_LOG_DATA_ADDR + ((uint32_t)idx * sizeof(W25Q64_LogEntry));
            W25Q64_LogEntry tmp;
            /* 读失败 → 后面的更老, 也没必要继续 */
            if (BSP_W25Q64_Read(addr, (uint8_t *)&tmp, sizeof(W25Q64_LogEntry)) != W25Q64_OK) break;
            /* 类型非法 → 说明这是没写过的槽位(擦除态 0xFF 不在枚举内), 停止计数 */
            if (tmp.type < W25Q64_LOG_SYSTEM_BOOT || tmp.type > W25Q64_LOG_SENSOR_DATA) break;
            /* 时间戳为擦除态 → 同样是空槽位 */
            if (tmp.timestamp == 0xFFFFFFFF) break;

            valid++;                        /* 这一条是真的, 计数 +1 */
            if (idx == 0) break;            /* 已经走到 0 号槽, 到头了 */
            idx--;                          /* 继续往前(更老的一条) */
        }

        /* 实际条数比元数据少 → 说明有"幽灵条目", 用实际值纠正 */
        if (valid < g_log_count) {
            printf("[W25Q64] Log count calibrated: %u -> %u\r\n", g_log_count, valid);
            g_log_count = valid;
        }
    }

    return W25Q64_OK;
}

/* ---------------------------------------------------------------------------
 * 读 JEDEC ID (厂商码 + 设备码), 用来确认芯片型号/通信是否正常
 *
 * 命令 0x9F 发出后, 芯片依次返回 3 个字节:
 *   [厂商 ID = 0xEF 华邦] [存储类型 = 0x40] [容量 = 0x17(64Mbit)]
 * 本函数只取第 1 个和第 2/3 个: manuf=0xEF, dev_id = 0x4017。
 *
 * 参数用指针返回, 且允许传 NULL 表示"这一项不关心"(下面的 if 判空)。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_ReadJEDEC_ID(uint8_t *manuf, uint16_t *dev_id)
{
    BSP_W25Q64_Lock();                                        /* 加锁, 串行访问 */
    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_JEDEC_ID);        /* 命令 0x9F */
    uint8_t m  = BSP_W25Q64_SPI_SendByte(0xFF);               /* 第 1 字节: 厂商 */
    uint8_t d1 = BSP_W25Q64_SPI_SendByte(0xFF);               /* 第 2 字节: 类型 */
    uint8_t d2 = BSP_W25Q64_SPI_SendByte(0xFF);               /* 第 3 字节: 容量 */
    W25Q64_CS_HIGH();
    BSP_W25Q64_Unlock();                                      /* 解锁 */

    if (manuf) *manuf = m;                                    /* 调用者要才写回 */
    if (dev_id) *dev_id = ((uint16_t)d1 << 8) | d2;           /* 拼成 16 位设备码 */
    return W25Q64_OK;
}

/* ---------------------------------------------------------------------------
 * 扇区擦除 (对外加锁版)
 *
 * 与 NoLock 版逻辑一致, 只是在整个操作外面加了互斥锁, 并保证任何提前 return
 * 的路径都会解锁(看那些 { ...Unlock(); return ret; } 的写法, 就是防漏解锁)。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_SectorErase(uint32_t addr)
{
    W25Q64_Status ret;

    if (addr >= W25Q64_CAPACITY) return W25Q64_INVALID;

    BSP_W25Q64_Lock();
    ret = BSP_W25Q64_WriteEnable();                            /* 写使能 */
    if (ret != W25Q64_OK) { BSP_W25Q64_Unlock(); return ret; } /* 失败也要解锁! */

    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_SECTOR_ERASE);          /* 0x20 扇区擦除 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);
    W25Q64_CS_HIGH();

    ret = BSP_W25Q64_WaitBusy(1000);                           /* 等擦除完成 */
    BSP_W25Q64_Unlock();
    return ret;
}

/* ---------------------------------------------------------------------------
 * 页编程 (对外加锁版, 一次最多 256 字节且不跨页)
 *
 * 比 NoLock 版多了一步: 进来先等 BUSY 清零。
 * 为什么? 因为可能上一次"扇区擦除"是异步发起的还没做完(擦除要 ~400ms),
 * 此时直接写会失败。先等它忙完, 再写。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_PageWrite(uint32_t addr, uint8_t *buf, uint16_t len)
{
    W25Q64_Status ret;

    if (addr >= W25Q64_CAPACITY || len == 0 || len > W25Q64_PAGE_SIZE) return W25Q64_INVALID;
    if (buf == NULL) return W25Q64_INVALID;

    BSP_W25Q64_Lock();
    /* 等上一笔操作完成 (异步擦除可能还在进行中, 扇区擦除需 ~400ms) */
    ret = BSP_W25Q64_WaitBusy(500);
    if (ret != W25Q64_OK) { BSP_W25Q64_Unlock(); return ret; }
    ret = BSP_W25Q64_WriteEnable();                            /* 写使能 */
    if (ret != W25Q64_OK) { BSP_W25Q64_Unlock(); return ret; }

    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_PAGE_PROGRAM);          /* 0x02 页编程 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);
    ret = BSP_W25Q64_SPI_Transmit(buf, len);                   /* 发数据 */
    W25Q64_CS_HIGH();

    if (ret != W25Q64_OK) { BSP_W25Q64_Unlock(); return ret; }
    ret = BSP_W25Q64_WaitBusy(100);                            /* 等写完 */
    BSP_W25Q64_Unlock();
    return ret;
}

/* ---------------------------------------------------------------------------
 * 任意地址读 (对外加锁版)
 *
 * 与 ReadNoLock 不同, 这里参数检查放在加锁之前(尽早失败, 少一次锁开销)。
 * 读取是只读操作, 本不破坏数据; 但仍然加锁, 是为了防止读的过程中被别的
 * 任务插入一个"擦除/写"命令, 导致本次 SPI 事务时序被打断、读到半新半旧的数据。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (addr >= W25Q64_CAPACITY || len == 0 || (addr + len) > W25Q64_CAPACITY) return W25Q64_INVALID;
    if (buf == NULL) return W25Q64_INVALID;

    BSP_W25Q64_Lock();
    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_DATA);             /* 0x03 读数据 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);
    W25Q64_Status ret = BSP_W25Q64_SPI_Receive(buf, (uint16_t)len);
    W25Q64_CS_HIGH();
    BSP_W25Q64_Unlock();
    return ret;
}

/* ---------------------------------------------------------------------------
 * 任意长度写 —— 内部自动"拆页"
 *
 * 这是对外最好用的写接口: 想写多少写多少, 不用管 256 字节页边界。
 * 原理就是循环: 每次只写到"当前页的末尾"为止, 然后进入下一页继续。
 *
 *   例: 从地址 0x0100 写 300 字节 (页大小 256)
 *       第 1 轮: 页内偏移 = 0x100 & 0xFF = 0x00, 本页剩余 = 256, 写 256 字节
 *       第 2 轮: 地址 0x0200, 偏移 0, 本页剩余 256, 但只剩 44 字节 → 写 44
 *
 * 【重要提醒】本函数只负责写, 不负责擦除。调用者必须保证目标区域已经是
 *   擦除过的(0xFF), 否则写出来的数据是"新旧按位与"的结果, 是错的。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_Write(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (addr >= W25Q64_CAPACITY || len == 0 || (addr + len) > W25Q64_CAPACITY) return W25Q64_INVALID;
    if (buf == NULL) return W25Q64_INVALID;

    uint32_t remain = len;        /* 还剩多少字节没写 */
    uint32_t cur_addr = addr;     /* 当前写到哪个地址 */
    uint8_t *p = buf;             /* 当前数据源指针 */

    /* 一页最多写 256 字节, 且一次页编程不能跨页边界 (W25Q64 页地址溢出会回卷)。
     * 所以对跨页的数据, 拆成若干次 PageWrite, 每次最多写到本页末尾。 */
    while (remain > 0) {
        uint32_t page_offset = cur_addr & 0xFF;   /* 当前地址在页内的偏移 (0~255) */

        /* 本轮可写长度 = min(本页剩余空间, 剩余总量) */
        uint16_t chunk = (uint16_t)((W25Q64_PAGE_SIZE - page_offset) < remain
                                    ? (W25Q64_PAGE_SIZE - page_offset) : remain);

        W25Q64_Status ret = BSP_W25Q64_PageWrite(cur_addr, p, chunk);   /* 写这一块 */
        if (ret != W25Q64_OK) return ret;                              /* 失败立刻返回 */

        cur_addr += chunk;      /* 地址前移 */
        p += chunk;             /* 数据指针前移 */
        remain -= chunk;        /* 剩余量减少 */
    }
    return W25Q64_OK;
}

/* ---------------------------------------------------------------------------
 * 全片擦除 (整个 8MB 清零为 0xFF)
 *
 * 【慎用】全片擦除非常慢: 手册标称典型 60 秒, 本函数给了 60 秒超时。
 * 擦除期间芯片完全不可用, 且如果中途掉电, 芯片状态不确定。
 * 日常操作应该用扇区擦除, 只有"出厂恢复"才用它。
 *
 * 注意: 全片擦除命令(0xC7)后面不需要地址参数。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_ChipErase(void)
{
    W25Q64_Status ret;

    BSP_W25Q64_Lock();
    ret = BSP_W25Q64_WriteEnable();                            /* 写使能 */
    if (ret != W25Q64_OK) { BSP_W25Q64_Unlock(); return ret; }

    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_CHIP_ERASE);            /* 0xC7 全片擦除(无地址) */
    W25Q64_CS_HIGH();

    ret = BSP_W25Q64_WaitBusy(60000);                          /* 最多等 60 秒 */
    BSP_W25Q64_Unlock();
    return ret;
}

/* ======================== OTA 异步擦除支持 ======================== */

/* ---------------------------------------------------------------------------
 * 只"发出"擦除命令, 不等它完成 (异步)
 *
 * 为什么需要这样一个"半截"接口?
 *   OTA 升级时要在 Flash 里腾出一大块空间(暂存新固件), 需要连续擦很多个扇区。
 *   每个扇区擦除要 400ms, 如果按普通 SectorErase 逐个"发命令+死等", 会阻塞很久,
 *   期间看门狗/网络任务都没法响应。
 *   所以改成"先发命令, 让芯片自己忙, 然后去干别的事, 稍后再来查 IsBusy"。
 *
 * 使用方式: SendEraseCmd(addr) → 做别的 → 循环等 IsBusy()==0 → 再发下一个
 * 注意: 这里虽然立即 Unlock 了, 但"芯片正在忙"这个状态是全局的,
 *       其它任务若此时读写 Flash 会拿到错误结果——所以调用方必须配合 IsBusy 使用。
 * --------------------------------------------------------------------------- */
void BSP_W25Q64_SendEraseCmd(uint32_t addr)
{
    BSP_W25Q64_Lock();
    BSP_W25Q64_WriteEnable();                                  /* 擦除前必须写使能 */
    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_SECTOR_ERASE);          /* 0x20 扇区擦除 */
    BSP_W25Q64_SPI_SendByte((addr >> 16) & 0xFF);
    BSP_W25Q64_SPI_SendByte((addr >> 8) & 0xFF);
    BSP_W25Q64_SPI_SendByte(addr & 0xFF);
    W25Q64_CS_HIGH();
    BSP_W25Q64_Unlock();                                       /* 发完就撤, 不等它忙完 */
}

/* ---------------------------------------------------------------------------
 * 查询芯片当前是否在忙 (非阻塞, 不等待)
 *
 * 与 WaitBusy 的区别: WaitBusy 会一直等到它不忙; 这个函数是"问一句就走",
 * 返回值 1=还在忙, 0=空闲。上层可以自己决定"忙的时候去干点别的"。
 * --------------------------------------------------------------------------- */
int BSP_W25Q64_IsBusy(void)
{
    uint8_t sr;
    BSP_W25Q64_Lock();
    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_STATUS1);          /* 读 SR1 */
    sr = BSP_W25Q64_SPI_SendByte(0xFF);
    W25Q64_CS_HIGH();
    BSP_W25Q64_Unlock();
    return (sr & W25Q64_SR1_BUSY) ? 1 : 0;                     /* BIT0=1 → 忙 */
}

/* ---------------------------------------------------------------------------
 * 读状态寄存器 2 (SR2)
 *
 * SR2 里最常用的是 QE 位(Quad Enable)。本项目是标准 SPI 单线模式, 用不到 QE,
 * 但 OTA 流程里会读一下确认芯片配置没被人改过(QE=1 会让 SPI 变成四线模式,
 * 那样普通 SPI 读写就全是乱码了)。
 *
 * 参数 sr2: 输出参数, 传 NULL 直接返回(调用者不关心就别白读一次)。
 * --------------------------------------------------------------------------- */
void BSP_W25Q64_ReadStatus2(uint8_t *sr2)
{
    if (sr2 == NULL) return;
    BSP_W25Q64_Lock();
    W25Q64_CS_LOW();
    BSP_W25Q64_SPI_SendByte(W25Q64_CMD_READ_STATUS2);          /* 0x35 读 SR2 */
    *sr2 = BSP_W25Q64_SPI_SendByte(0xFF);
    W25Q64_CS_HIGH();
    BSP_W25Q64_Unlock();
}

/* ---------------------------------------------------------------------------
 * 计算配置结构体的 CRC32
 *
 * 注意长度是 sizeof(W25Q64_Config) - 4: 最后 4 字节本身就是 crc32 字段,
 * 算校验和时必须把它排除, 否则永远算不出稳定值(自己包含自己, 循环依赖)。
 * --------------------------------------------------------------------------- */
static uint32_t BSP_W25Q64_ConfigCRC(W25Q64_Config *cfg)
{
    return BSP_W25Q64_CRC32((uint8_t *)cfg, sizeof(W25Q64_Config) - 4);
}

/* ---------------------------------------------------------------------------
 * 用出厂默认值填充一份配置结构体
 *
 * 什么时候用? 首次上电(Flash 全是 0xFF, 校验必然失败)、或者 Flash 里的配置
 * 被写坏时, 回退到一套可用的默认参数, 保证系统"至少能跑起来"。
 *
 * 默认值: 网关 192.168.10.100/24, 网关 192.168.10.1, 轮询 1000ms,
 *         485 波特率 9600, 从机地址 1, 关闭 DHCP, TCP 端口 502。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_ConfigDefault(W25Q64_Config *cfg)
{
    if (cfg == NULL) return W25Q64_INVALID;
    memset(cfg, 0, sizeof(W25Q64_Config));         /* 先全部清零, 包括保留字节 */
    cfg->magic   = W25Q64_CONFIG_MAGIC;            /* 打上魔数, 标记"这是一份合法配置" */
    cfg->gateway_ip[0] = 192; cfg->gateway_ip[1] = 168; cfg->gateway_ip[2] = 10; cfg->gateway_ip[3] = 100;
    cfg->subnet_mask[0] = 255; cfg->subnet_mask[1] = 255; cfg->subnet_mask[2] = 255; cfg->subnet_mask[3] = 0;
    cfg->default_gw[0]  = 192; cfg->default_gw[1]  = 168; cfg->default_gw[2]  = 10; cfg->default_gw[3]  = 1;
    cfg->poll_cycle_ms  = 1000;                    /* RS-485 轮询周期 1 秒 */
    cfg->rs485_baudrate = 9600;                    /* 485 波特率 */
    cfg->slave_id       = 1;                       /* 默认从机地址 */
    cfg->dhcp_enable    = 0;                       /* 不用 DHCP, 用静态 IP */
    cfg->tcp_port       = 502;                     /* Modbus-TCP 标准端口 */
    cfg->crc32 = BSP_W25Q64_ConfigCRC(cfg);        /* 最后算 CRC, 必须放在所有字段之后 */
    return W25Q64_OK;
}

/* ---------------------------------------------------------------------------
 * 判断一份配置结构是否有效: magic 匹配 + CRC 校验通过
 *
 * 为什么要双重校验? magic 只能挡"明显是垃圾数据"的情况(比如 0xFF);
 * CRC 才能挡"数据被写坏一两个 bit"这种更难发现的问题。
 *
 * 返回值: 1=有效, 0=无效
 * --------------------------------------------------------------------------- */
uint8_t BSP_W25Q64_ConfigIsValid(W25Q64_Config *cfg)
{
    if (cfg == NULL) return 0;
    if (cfg->magic != W25Q64_CONFIG_MAGIC) return 0;   /* 魔数不对, 直接判无效 */
    uint32_t calc = BSP_W25Q64_ConfigCRC(cfg);         /* 重算 CRC */
    return (calc == cfg->crc32) ? 1 : 0;               /* 与存的 CRC 比对 */
}

/* ---------------------------------------------------------------------------
 * 上电加载配置
 *
 * 逻辑: 从 Flash 读一份 → 有效则复制给调用方;
 *       无效(CRC 错/首次上电全 0xFF)则回退为出厂默认并写回 Flash,
 *       保证从此以后 Flash 里永远有一份合法配置。
 *
 * 参数 cfg: 输出, 调用方提供的结构体, 函数负责填好
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_ConfigLoad(W25Q64_Config *cfg)
{
    if (cfg == NULL) return W25Q64_INVALID;

    W25Q64_Config tmp;                                        /* 先读到临时变量 */
    W25Q64_Status ret = BSP_W25Q64_Read(W25Q64_CONFIG_ADDR, (uint8_t *)&tmp, sizeof(W25Q64_Config));
    if (ret != W25Q64_OK) return ret;                         /* 读都读不到, 直接失败 */

    if (BSP_W25Q64_ConfigIsValid(&tmp)) {
        memcpy(cfg, &tmp, sizeof(W25Q64_Config));             /* Flash 里是有效配置 → 直接采用 */
        return W25Q64_OK;
    }

    /* 走到这里说明 Flash 里没有合法配置(首次上电或被写坏) */
    BSP_W25Q64_ConfigDefault(cfg);                            /* 回退默认值 */
    return BSP_W25Q64_ConfigSave(cfg);                        /* 并写回, 让下次能有合法配置 */
}

/* ---------------------------------------------------------------------------
 * 保存配置到 Flash
 *
 * 顺序很讲究:
 *   1) 重算 magic + CRC —— 因为调用方可能刚改了某些字段, CRC 必须同步更新,
 *      否则下次上电校验不通过, 配置就"白存了"
 *   2) 擦除整个配置扇区 —— NOR Flash 写之前必须先擦
 *   3) 延时 10ms —— 给擦除操作一点余量(擦除是异步的)
 *   4) 页编程写入整个结构体 —— 256 字节正好一页, 不会跨页
 *
 * 【掉电风险】擦除完到写回之间如果掉电, 配置就丢了。更稳妥的做法是"双备份扇区
 *   交替写"(A/B 区), 本项目的实现较简单, 属于可以优化但未实现的点。
 *   主动说明这个取舍是好的工程习惯。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_ConfigSave(W25Q64_Config *cfg)
{
    if (cfg == NULL) return W25Q64_INVALID;

    cfg->magic = W25Q64_CONFIG_MAGIC;                 /* 保证魔数正确 */
    cfg->crc32 = BSP_W25Q64_ConfigCRC(cfg);           /* 任何字段改动后重建 CRC */

    W25Q64_Status ret = BSP_W25Q64_SectorErase(W25Q64_CONFIG_ADDR);   /* 2) 擦除配置扇区 */
    if (ret != W25Q64_OK) return ret;

    osDelay(10);    /* 3) 稍微等待, 确保擦除结束再写 */

    /* 4) 写入。sizeof(W25Q64_Config)=256 = 一页, 不会跨页 */
    return BSP_W25Q64_PageWrite(W25Q64_CONFIG_ADDR, (uint8_t *)cfg, sizeof(W25Q64_Config));
}

/* ---------------------------------------------------------------------------
 * 写一条日志到 Flash 的环形缓冲区 (由日志任务调用, 异步)
 *
 * 日志区 = 一块固定大小的连续区域 (W25Q64_LOG_DATA_ADDR 起), 用"环形"管理:
 *   写指针 g_log_write_index 递增, 到底部回绕到 0; 记录数 g_log_count 封顶为
 *   W25Q64_LOG_ENTRY_MAX, 满了之后新日志覆盖最老的一条。
 * 日志的元数据(总条数 + 写指针)单独存在 W25Q64_LOG_META_ADDR, 上电据此恢复。
 *
 * 【为什么要环形?】Flash 寿命有限(约 10 万次擦写), 不可能无限写。环形 + 覆盖
 *   最老记录, 让擦写点在整片区域上轮转, 大大延长寿命。
 *
 * 【为什么写入前要判断"是否已擦除"?】
 *   每个 4KB 扇区装着 512 条日志。写第 1 条时扇区是新的(全 0xFF), 直接写就行;
 *   但覆盖到第 513 条、又绕回这个扇区时, 扇区里已经全是旧数据, 必须先擦除整个
 *   扇区才能写。而"擦除会一次性抹掉这个扇区里的 512 条旧日志"——这是环形日志
 *   固有的代价, 所以一次擦除后, 512 条会依次写满。
 *   判断方法: 读一下目标地址, 看是不是 0xFFFFFFFF, 是就说明还没被写过, 免擦。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_LogWrite(W25Q64_LogEntry *entry)
{
    W25Q64_Status ret;
    uint32_t entry_addr, sector_addr;      /* 本条记录地址 / 它所在扇区首地址 */

    if (entry == NULL) return W25Q64_INVALID;

    BSP_W25Q64_Lock();                     /* 整个写日志过程持锁, 保证原子性 */

    /* ---- 1) 环形指针前进: 写指针 +1, 越界回绕; 条数在没到上限前逐增 ---- */
    g_log_write_index++;
    if (g_log_write_index >= W25Q64_LOG_ENTRY_MAX) {
        g_log_write_index = 0;             /* 回绕到 0 号槽 */
    }
    if (g_log_count < W25Q64_LOG_ENTRY_MAX) {
        g_log_count++;                     /* 还没满, 条数增长(满了就不再加) */
    }

    /* ---- 2) 算出本条记录的物理地址, 以及它所在扇区的首地址 ---- */
    entry_addr  = W25Q64_LOG_DATA_ADDR + ((uint32_t)g_log_write_index * sizeof(W25Q64_LogEntry));
    /* 扇区地址 = 数据区起点 + (第几个扇区) × 扇区大小 */
    sector_addr = W25Q64_LOG_DATA_ADDR + ((uint32_t)(g_log_write_index / LOG_ENTRIES_PER_SECTOR) * W25Q64_SECTOR_SIZE);

    /* ---- 3) 判断目标位置是否已被写过, 决定要不要先擦扇区 ---- */
    {
        W25Q64_LogEntry probe;
        uint8_t need_erase = 1;            /* 默认认为需要擦 */

        /* 读出该地址原有内容, 若为全 0xFF(擦除态)则说明这块是空的, 无需再擦 */
        if (BSP_W25Q64_ReadNoLock(entry_addr, (uint8_t *)&probe, sizeof(W25Q64_LogEntry)) == W25Q64_OK) {
            uint32_t *raw = (uint32_t *)&probe;              /* 按 32 位看这 8 字节 */
            /* 头两个 32 位字都是 0xFFFFFFFF → 整条记录都是 0xFF → 空槽 */
            if (raw[0] == 0xFFFFFFFF && raw[1] == 0xFFFFFFFF) {
                need_erase = 0;
            }
        }

        /* 该扇区有旧数据(无法直接写入) → 先擦整个扇区 */
        if (need_erase) {
            ret = BSP_W25Q64_SectorEraseNoLock(sector_addr);
            if (ret != W25Q64_OK) {
                BSP_W25Q64_Unlock();       /* 出错也要解锁并返回 */
                return ret;
            }
        }
    }

    /* ---- 4) 写入这一条日志 (8 字节, 远小于一页, 不会跨页) ---- */
    ret = BSP_W25Q64_PageWriteNoLock(entry_addr, (uint8_t *)entry, sizeof(W25Q64_LogEntry));
    if (ret != W25Q64_OK) {
        BSP_W25Q64_Unlock();
        return ret;
    }

    /* ---- 5) 更新元数据扇区, 记下新的 count 与写指针 ---- */
    {
        /* 元数据格式: [0..1]保留(0xFF) [2..3]count(大端) [4..5]index(大端) */
        uint8_t meta[8];
        memset(meta, 0xFF, 8);                            /* 未用字节填 0xFF(擦除态) */
        meta[2] = (g_log_count >> 8) & 0xFF;              /* count 高字节 */
        meta[3] = g_log_count & 0xFF;                     /* count 低字节 */
        meta[4] = (g_log_write_index >> 8) & 0xFF;        /* index 高字节 */
        meta[5] = g_log_write_index & 0xFF;               /* index 低字节 */

        /* 元数据要先擦后写(每次都是整扇区重写)。这也是为什么元数据单独占一个扇区,
         * 避免和日志数据互相影响扇区擦除。 */
        ret = BSP_W25Q64_SectorEraseNoLock(W25Q64_LOG_META_ADDR);
        if (ret != W25Q64_OK) {
            BSP_W25Q64_Unlock();
            return ret;
        }

        ret = BSP_W25Q64_PageWriteNoLock(W25Q64_LOG_META_ADDR, meta, 8);
        if (ret != W25Q64_OK) {
            BSP_W25Q64_Unlock();
            return ret;
        }
    }

    BSP_W25Q64_Unlock();
    return W25Q64_OK;
}

/* ---------------------------------------------------------------------------
 * 按索引读一条日志 (index 0 = 最新, 1 = 次新, ...)
 *
 * 【环形缓冲区的索引换算 —— 最容易写错的地方】
 *   日志是按"时间"逻辑编号的, 但物理槽位是环形排列的, 两者不是同一个数。
 *   分两种情况:
 *
 *   A) 已写满(g_log_count == 上限): 写指针的"下一个"槽位就是最老的一条,
 *      所以第 index 新的那条 = (写指针 + 1 + index), 超过上限再减去上限回绕。
 *
 *   B) 没写满: 日志从 0 号槽连续写到写指针处, 所以第 index 新的那条
 *      = 写指针 - index。如果 index > 写指针, 说明请求的量超过了已有记录, 非法。
 *
 * 读完还要再校验一次类型/时间戳, 因为可能索引算对了但那个槽位其实还是空的。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_LogRead(uint16_t index, W25Q64_LogEntry *entry)
{
    if (entry == NULL) return W25Q64_INVALID;
    if (index >= g_log_count) return W25Q64_INVALID;     /* 超过已有条数, 非法 */

    uint16_t actual;                                     /* 对应的物理槽位号 */
    if (g_log_count >= W25Q64_LOG_ENTRY_MAX) {
        /* 情况 A: 已写满(环形回绕过): 物理槽位 = 写指针 + 1 + index, 越界再回绕 */
        actual = g_log_write_index + 1 + index;
        if (actual >= W25Q64_LOG_ENTRY_MAX) {
            actual -= W25Q64_LOG_ENTRY_MAX;              /* 回绕一次即可, 因为 index < 上限 */
        }
    } else {
        /* 情况 B: 没写满, 从最新往前数 index 个 */
        if (index <= g_log_write_index) {
            actual = g_log_write_index - index;
        } else {
            return W25Q64_INVALID;                       /* 索引超过已有记录 */
        }
    }

    uint32_t entry_addr = W25Q64_LOG_DATA_ADDR + ((uint32_t)actual * sizeof(W25Q64_LogEntry));
    W25Q64_Status ret = BSP_W25Q64_Read(entry_addr, (uint8_t *)entry, sizeof(W25Q64_LogEntry));
    if (ret != W25Q64_OK) return ret;

    /* 校验读回来的是"有效日志": 类型在合法枚举范围 + 时间戳不是擦除态 0xFFFFFFFF */
    if (entry->type < W25Q64_LOG_SYSTEM_BOOT || entry->type > W25Q64_LOG_SENSOR_DATA) {
        return W25Q64_INVALID;
    }
    if (entry->timestamp == 0xFFFFFFFF) {
        return W25Q64_INVALID;
    }

    return W25Q64_OK;
}

/* 返回当前有效日志条数 (供上位机查询"有多少条日志") */
uint16_t BSP_W25Q64_LogGetCount(void)
{
    return g_log_count;
}

/* ---------------------------------------------------------------------------
 * 清空全部日志
 *
 * 注意: 只清"元数据"(把 count/index 置 0), 并【不】去擦 7.5MB 的日志数据区——
 * 那样太慢且没意义。因为 count=0 后, LogRead 的 index>=g_log_count 检查
 * 会直接拒绝读取, 旧的物理数据虽然还在, 但从逻辑上已经"不存在"了。
 * 下次写日志时会按环形逻辑覆盖它们。
 * --------------------------------------------------------------------------- */
W25Q64_Status BSP_W25Q64_LogClear(void)
{
    W25Q64_Status ret;

    /* 1) 内存状态清零 */
    g_log_write_index = 0;
    g_log_count = 0;
    g_log_meta_dirty = 0;

    /* 2) 元数据扇区擦除(擦完是全 0xFF) */
    ret = BSP_W25Q64_SectorErase(W25Q64_LOG_META_ADDR);
    if (ret != W25Q64_OK) return ret;
    osDelay(10);                                    /* 等擦除稳定 */

    /* 3) 写入"0 条日志"的元数据。注意保留字节保持 0xFF, 和 LogWrite 的格式一致 */
    uint8_t meta[8];
    memset(meta, 0xFF, 8);
    meta[2] = 0;                                    /* count 高字节 */
    meta[3] = 0;                                    /* count 低字节 */
    meta[4] = 0;                                    /* index 高字节 */
    meta[5] = 0;                                    /* index 低字节 */
    ret = BSP_W25Q64_PageWrite(W25Q64_LOG_META_ADDR, meta, 8);
    return ret;
}

/* ---------------------------------------------------------------------------
 * 出厂自检函数 (只用于 bring-up 阶段手动调用, 正常固件流程里不会执行到)
 *
 * 做了 4 组验证:
 *   1) 读 JEDEC ID (结果被赋值回自身, 是给调试器下断点看的)
 *   2) 擦除 0x1000 扇区 → 写 256 字节递增数据 → 读回比对 → 再擦掉
 *   3) 配置读写: 加载 → 改 polling 周期 → 存回
 *   4) 写一条日志
 * 最后死循环, 方便连着调试器观察状态。
 *
 * 这种"上电就把外设挨个自测一遍"的写法叫 bring-up 测试, 硬件刚焊好时非常有用。
 * --------------------------------------------------------------------------- */
void BSP_W25Q64_Test(void)
{
    uint8_t manuf;
    uint16_t dev_id;

    /* 1) 读 ID。这里的自赋值是把结果留下来给调试器看, 防止被编译器优化掉 */
    if (BSP_W25Q64_ReadJEDEC_ID(&manuf, &dev_id) == W25Q64_OK) {
        manuf = manuf;
        dev_id = dev_id;
    }

    uint8_t test_buf[256];    /* 写入数据 */
    uint8_t veri_buf[256];    /* 读回比对 */

    /* 2) 先擦除测试扇区。必须擦, 否则写进去的数据不对 */
    BSP_W25Q64_SectorErase(0x00001000);

    for (uint16_t i = 0; i < 256; i++) {
        test_buf[i] = (uint8_t)(i & 0xFF);            /* 生成 0,1,2,...,255 的递增图案 */
    }

    BSP_W25Q64_PageWrite(0x00001000, test_buf, 256);  /* 写一页 */
    memset(veri_buf, 0, sizeof(veri_buf));            /* 清空接收缓冲(便于看出哪些字节没读到) */
    BSP_W25Q64_Read(0x00001000, veri_buf, 256);       /* 读回 */

    uint8_t match = 1;                                /* 逐个字节比对 */
    for (uint16_t i = 0; i < 256; i++) {
        if (test_buf[i] != veri_buf[i]) { match = 0; break; }
    }

    BSP_W25Q64_SectorErase(0x00001000);               /* 测完擦掉, 不留垃圾 */

    /* 3) 配置读写测试 */
    W25Q64_Config cfg;
    if (BSP_W25Q64_ConfigLoad(&cfg) == W25Q64_OK) {
        cfg.poll_cycle_ms = 500;                      /* 改一个字段 */
        BSP_W25Q64_ConfigSave(&cfg);                  /* 存回(会重算 CRC) */
    }

    /* 4) 写一条系统启动日志作为测试 */
    W25Q64_LogEntry log;
    log.timestamp = osKernelGetTickCount();           /* 用系统 tick 当时间戳 */
    log.type = W25Q64_LOG_SYSTEM_BOOT;
    log.reserved = 0;
    log.data = 0;
    BSP_W25Q64_LogWrite(&log);

    for (;;) {                                        /* 停在这里, 方便挂调试器 */
        osDelay(1000);
    }
}
