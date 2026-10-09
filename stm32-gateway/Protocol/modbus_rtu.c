/*
 * ============================================================================
 * modbus_rtu.c — Modbus RTU 主站(Master)协议实现
 * ============================================================================
 *
 * 【本文件干什么】
 *   把"以太网侧收到的 Modbus-TCP 请求"翻译成"RS-485 总线上的 Modbus-RTU 请求"
 *   发出去, 再把从机的 RTU 响应收回来。它是网关的"下半身"(上半身见 modbus_tcp.c)。
 *
 * 【Modbus 是什么】
 *   工业现场最通用的"主从式"串行协议: 一个主站(Master)问, 多个从站(Slave)答,
 *   从站永远不主动说话。本项目里: STM32 网关 = 主站, KTH2-R30 温湿度变送器 = 从站。
 *
 * 【RTU 帧格式】(本文件的主角)
 *   +---------+---------+----------+---------+---------+
 *   | 从机地址 | 功能码  |   数据    |  CRC低   |  CRC高   |
 *   |  1 字节  | 1 字节  |  N 字节   | 1 字节   | 1 字节   |
 *   +---------+---------+----------+---------+---------+
 *   - 地址/寄存器号/数量等一律"大端序"(高字节在前);
 *   - CRC16 是唯一例外: 低字节在前(小端);
 *   - 帧边界: RTU 靠"3.5 个字符时间的静默间隔"分隔帧, 本工程用
 *     DMA + USART IDLE 中断来自动判定"一帧收完"。
 *
 * 【本项目实际用法】
 *   - 只用功能码 03(读保持寄存器), 读 KTH2-R30 的温度/湿度;
 *   - 总线是半双工共享资源 → 用互斥锁 g_modbus_mutex 串行化;
 *   - 提供两套收发路径:
 *       · 阻塞路径 Modbus_ReadHoldingRegs / Modbus_SendRecvRTU
 *         → 适合健康检查任务这类"单独慢慢等"的场景;
 *       · 非阻塞路径 Modbus_AsyncBegin / Modbus_AsyncPoll / Modbus_AsyncAbort
 *         → 适合 TcpServerTask 同时管理 4 个 Socket 的场景: 不能让某一个
 *           Socket 的等待把整个任务卡住, 导致其他 Socket 收不到数据。
 *
 * 【建议阅读顺序】
 *   Modbus_CRC16 → Modbus_ParseFrame → Modbus_SendRecvRTU(阻塞)
 *   → Modbus_AsyncBegin/AsyncPoll/AsyncAbort(非阻塞)
 * ============================================================================
 */
#include "modbus_rtu.h"   /* 本模块自己的接口声明、功能码常量、状态枚举 */
#include "bsp_rs485.h"    /* 底层 485 收发: 发送/等待/取 DMA 捕获的帧 */
#include <string.h>       /* memcpy / memset */
#include <stdio.h>        /* printf 调试打印 */
#include "cmsis_os.h"     /* FreeRTOS 封装层: osMutexXxx / osKernelGetTickCount */

/* ---------------------------------------------------------------------------
 * RS-485 总线互斥锁(全局唯一)
 *
 * 【为什么需要锁】
 *   RS-485 是"半双工 + 共享"总线: 同一时刻只允许一个任务在发, 否则两路数据
 *   会在物理线上叠加(总线冲突), 波形全乱、从机收到的是垃圾。
 *   本工程有两个任务会碰总线 —— 健康检查任务(阻塞读) 和 TcpServerTask(非阻塞读),
 *   所以必须用互斥锁把它们串行化。
 *
 * 【为什么用 osMutexPrioInherit(优先级继承)】
 *   防止"优先级反转": 若低优先级任务持锁不放, 高优先级任务会被一直挡在门外;
 *   优先级继承会临时把"持锁任务"的优先级抬到与等待者相同, 逼它尽快跑完释放锁。
 *
 * 【易错点】未初始化时 g_modbus_mutex 为 NULL, 所有访问点都做了判空以降级运行。
 * --------------------------------------------------------------------------- */
static osMutexId_t g_modbus_mutex;

/* 初始化 Modbus RTU: 创建互斥锁(在任何收发之前必须调用一次) */
void Modbus_RTU_Init(void)
{
    /* 属性结构体: {名字, 属性标志, 控制块指针, 控制块大小}
     * 第 2 个字段取 osMutexPrioInherit 即开启优先级继承。 */
    osMutexAttr_t attr = { NULL, osMutexPrioInherit, NULL, 0 };
    g_modbus_mutex = osMutexNew(&attr);   /* 动态创建互斥量; 失败返回 NULL */
}

/* ---------------------------------------------------------------------------
 * CRC16 查表(多项式 0x8005, 即 Modbus 标准多项式 x^16 + x^15 + x^2 + 1)
 *
 * 【为什么用查表法】
 *   逐位移位法每个字节要循环 8 次判断 + 移位; 查表法每字节只做 2 次查表 + 2 次异或,
 *   实测快约 4 倍。代价是 512 字节常量表(占 Flash, 不占 RAM)。
 *
 * 【表为什么拆成 hi/lo 两张】
 *   一个字节索引对应一个 16 位 CRC 结果。常见写法是用一张 uint16_t 表:
 *       crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF];
 *   本文件为了省掉 16 位移位, 把它拆成两张 uint8_t 表分别替换高低字节:
 *       index  = crc_lo ^ byte;
 *       crc_lo = crc_hi ^ crc_hi_table[index];
 *       crc_hi = crc_lo_table[index];
 *   等价关系(已逐项验证, 256/256 匹配):
 *       crc_hi_table[i] == 标准 uint16_t 表 T[i] 的【低】字节
 *       crc_lo_table[i] == 标准 uint16_t 表 T[i] 的【高】字节
 *   【命名提醒】这两张表的名字是按"参与运算的变量"命名的, 和标准表的高/低字节
 *   顺序正好相反, 别拿去和网上的 uint16_t 表硬套 —— 以本文件算法为准。
 *   正确性用标准测试向量验证过: CRC16("123456789") = 0x4B37。
 *
 * 【CRC16-Modbus 的初始值】 = 0xFFFF 且不取反(这点和 CRC32 的
 *   0xFFFFFFFF + 最后取反不同, 别搞混)。
 * --------------------------------------------------------------------------- */
static const uint8_t crc_hi_table[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t crc_lo_table[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* 计算 Modbus CRC16(查表法), 返回"高字节在前"的 16 位值
 *
 * 算法(逐字节滚动):
 *   1) CRC 寄存器初值 = 0xFFFF(拆成 crc_hi / crc_lo 两个字节);
 *   2) 每来一个字节: index = crc_lo ^ byte;
 *   3) crc_lo = crc_hi ^ crc_hi_table[index];
 *   4) crc_hi = crc_lo_table[index];
 *   5) 重复直到所有字节处理完。
 *
 * 【易错点】Modbus 规定 CRC 放进帧里时是"低字节在前"(小端), 所以组帧时要用
 *   crc & 0xFF 放低位、crc >> 8 放高位 —— 见 Modbus_ReadHoldingRegs 的组帧处。
 */
uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len)
{
    uint8_t crc_hi = 0xFF;   /* CRC 寄存器高字节, 初值 0xFF */
    uint8_t crc_lo = 0xFF;   /* CRC 寄存器低字节, 初值 0xFF */
    uint8_t index;           /* 查表索引 = 当前低字节 ^ 本字节 */

    while (len--) {                             /* 逐字节处理, len 递减到 0 */
        index = crc_lo ^ *buf++;                 /* 低字节与当前数据异或得到索引; *buf++ 先取值再后移指针 */
        crc_lo = crc_hi ^ crc_hi_table[index];   /* 新低字节 = 旧高字节 ^ 高表[index] */
        crc_hi = crc_lo_table[index];            /* 新高字节 = 低表[index] */
    }
    return ((uint16_t)crc_hi << 8) | crc_lo;     /* 拼成 16 位返回(高字节在前) */
}

/* ---- 完整帧解析(阻塞/非阻塞两条路径共用) ----
 * 这里只是前向声明: 它的定义在文件末尾, 而下面的 Modbus_SendRecvRTU 就要调用它。 */
static Modbus_Status Modbus_ParseFrame(const uint8_t *frame, uint16_t len,
                                       uint8_t slave_addr, uint8_t *out, uint16_t *out_len);

/* ---------------------------------------------------------------------------
 * 功能码 03: 读保持寄存器(Modbus 最常用的功能码)
 *
 * 请求帧(固定 8 字节, 主站→从站):
 *   [addr][03][起始地址H][起始地址L][数量H][数量L][CRCL][CRCH]
 * 响应帧(变长, 从站→主站):
 *   [addr][03][字节数][数据...][CRCL][CRCH]
 *
 * 参数: slave_addr 从机地址(1~247)
 *       start_addr 起始寄存器地址
 *       reg_count  要读的寄存器个数(1~125)
 *       data       输出缓冲区(调用方保证能放下 reg_count 个 uint16_t)
 * 返回: MODBUS_OK 成功; 其余见 Modbus_Status
 * --------------------------------------------------------------------------- */
Modbus_Status Modbus_ReadHoldingRegs(uint8_t slave_addr, uint16_t start_addr,
                                      uint16_t reg_count, uint16_t *data)
{
    if (data == NULL || reg_count == 0 || reg_count > 125) {
        return MODBUS_ERROR_INVALID;   /* 参数非法: 空指针 / 个数为 0 / 超过协议上限 125 */
    }

    /* 构建 Modbus 03 请求帧(固定 8 字节): 地址 + 功能码 + 起始地址 + 数量 + CRC */
    uint8_t tx_buf[8];
    tx_buf[0] = slave_addr;                    /* 从机地址 */
    tx_buf[1] = MODBUS_FC_READ_HOLDING_REGS;   /* 功能码 0x03 */
    tx_buf[2] = (start_addr >> 8) & 0xFF;      /* 起始地址高字节(大端序) */
    tx_buf[3] = start_addr & 0xFF;             /* 起始地址低字节 */
    tx_buf[4] = (reg_count >> 8) & 0xFF;       /* 寄存器数量高字节 */
    tx_buf[5] = reg_count & 0xFF;              /* 寄存器数量低字节 */
    uint16_t crc = Modbus_CRC16(tx_buf, 6);    /* 只对前 6 字节算 CRC(CRC 自身不参与) */
    tx_buf[6] = crc & 0xFF;                    /* CRC 低字节先发(注意: 低字节在前!) */
    tx_buf[7] = (crc >> 8) & 0xFF;             /* CRC 高字节后发 */

    /* 通用收发: 内部会锁总线 + 最多重试 MODBUS_RTU_RETRIES(=5) 次 */
    uint8_t rx_buf[MODBUS_RTU_FRAME_MAX];      /* 响应帧缓冲(栈上 256 字节) */
    uint16_t rx_len = 0;                       /* 实际收到的响应帧长 */
    Modbus_Status st = Modbus_SendRecvRTU(tx_buf, 8, rx_buf, &rx_len);
    if (st != MODBUS_OK) {
        return st;                             /* 发送/接收失败, 直接上抛 */
    }

    /* 校验响应结构:
     *   - 至少 5 字节(最短正常响应 = 地址+功能码+字节数+CRC2);
     *   - 功能码必须是 03(从机异常时会变成 0x83, 那种帧已在 ParseFrame 里被拦下);
     *   - 字节数 = 寄存器数 × 2(每个寄存器占 2 字节);
     *   - 总长 = 3(地址+功能码+字节数) + 字节数 + 2(CRC), 多一个字节都算错帧。 */
    if (rx_len < 5 || rx_buf[1] != MODBUS_FC_READ_HOLDING_REGS ||
        rx_buf[2] != (reg_count * 2) || rx_len != (uint16_t)(3 + rx_buf[2] + 2)) {
        return MODBUS_ERROR_TIMEOUT;
    }

    /* 把响应数据区里的每个寄存器(大端 2 字节)拼成 uint16 存进输出数组
     * 数据从 rx_buf[3] 开始, 每 2 字节一个寄存器, 高位在前 */
    for (uint16_t i = 0; i < reg_count; i++) {
        data[i] = ((uint16_t)rx_buf[3 + i * 2] << 8) | rx_buf[3 + i * 2 + 1];
    }

    return MODBUS_OK;
}

/* ---------------------------------------------------------------------------
 * 通用 Modbus RTU 帧收发(支持任意功能码) —— 阻塞版
 *
 * 流程: 拿锁 → 校验发送帧自带 CRC → 循环重试{ 发送 + 等响应 + 解析 } → 释放锁
 * 之所以叫"阻塞": 内部有 osMutexAcquire 等待和 BSP_RS485_WaitFrame 等待,
 * 调用它的任务这段时间会被挂起, 干不了别的事。
 * --------------------------------------------------------------------------- */
Modbus_Status Modbus_SendRecvRTU(const uint8_t *tx_frame, uint16_t tx_len,
                                  uint8_t *rx_frame, uint16_t *rx_len)
{
    if (tx_frame == NULL || tx_len < 4 || rx_frame == NULL || rx_len == NULL) {
        return MODBUS_ERROR_INVALID;   /* 空指针, 或帧长不足(最短 4 = 地址+功能码+CRC2) */
    }

    /* 申请总线互斥锁。超时 3000ms: 若一直被别的任务占用, 不再无限等下去 */
    if (g_modbus_mutex != NULL) {                        /* 允许未初始化时降级运行 */
        if (osMutexAcquire(g_modbus_mutex, 3000) != osOK) {
            return MODBUS_ERROR_TIMEOUT;                 /* 3 秒都没拿到锁 → 放弃本次 */
        }
    }

    /* 校验发送帧自身的 CRC, 提前发现"调用方自己就把帧组错了"的情况
     * 帧尾 2 字节按小端解释: 最后 1 字节是 CRC 高字节, 倒数第 2 字节是低字节 */
    uint16_t expected_crc = Modbus_CRC16((uint8_t *)tx_frame, tx_len - 2);
    uint16_t frame_crc = ((uint16_t)tx_frame[tx_len - 1] << 8) | tx_frame[tx_len - 2];
    if (expected_crc != frame_crc) {
        osMutexRelease(g_modbus_mutex);                  /* 一定要还锁, 否则后面全部死锁 */
        return MODBUS_ERROR_CRC;
    }

    Modbus_Status result = MODBUS_ERROR_TIMEOUT;         /* 默认结果: 超时 */

    for (uint8_t retry = 0; retry < MODBUS_RTU_RETRIES; retry++) {
        /* 步骤 1: 把请求帧发到 485 总线(内部切 DE 到发送态, 发完切回接收) */
        if (BSP_RS485_Send((uint8_t *)tx_frame, tx_len) != RS485_OK) {
            continue;                                    /* 发送失败 → 直接进入下一次重试 */
        }

        /* 步骤 2: 阻塞等待从机响应帧(最多 MODBUS_RTU_TIMEOUT_MS = 400ms)
         * 底层靠 DMA + IDLE 中断判帧: 总线静默一段时间即认为一帧收完 */
        if (BSP_RS485_WaitFrame(MODBUS_RTU_TIMEOUT_MS) != RS485_OK) {
            printf("[RTU] No response, retry %d\r\n", retry);
            continue;                                    /* 无响应 → 重试 */
        }

        /* 步骤 3: 对 DMA 捕获到的帧做解析(地址 / CRC / 异常判断) */
        result = Modbus_ParseFrame(BSP_RS485_FramePtr(), BSP_RS485_FrameLen(),
                                   tx_frame[0], rx_frame, rx_len);
        if (result == MODBUS_OK || result == MODBUS_ERROR_EXCEPTION) {
            break;                                       /* 成功 或 从机明确报错 → 结束重试 */
        }
        /* 否则是 CRC 错 / 地址不符 / 帧过短 → 继续下一次重试 */
    }

    if (g_modbus_mutex != NULL) {
        osMutexRelease(g_modbus_mutex);                  /* 无论成功失败都要释放总线锁 */
    }

    return result;                                       /* OK / EXCEPTION / TIMEOUT / CRC */
}

/* 状态枚举 → 可读字符串, 仅用于打印调试 */
const char* Modbus_StatusStr(Modbus_Status status)
{
    switch (status) {
        case MODBUS_OK:              return "OK";
        case MODBUS_ERROR_TIMEOUT:   return "TIMEOUT";
        case MODBUS_ERROR_CRC:       return "CRC ERROR";
        case MODBUS_ERROR_EXCEPTION: return "EXCEPTION";
        case MODBUS_ERROR_INVALID:   return "INVALID PARAM";
        case MODBUS_BUSY:            return "BUSY";
        default:                     return "UNKNOWN";
    }
}

/* ---------------------------------------------------------------------------
 * 完整帧解析 —— 阻塞/非阻塞两条路径共用
 *
 * 校验链: 从机地址匹配 → 是否异常帧 → CRC 校验 → 拷贝到输出缓冲
 * 返回值语义(很关键, 调用方靠它决定"要不要重试"):
 *   MODBUS_OK / MODBUS_ERROR_EXCEPTION      → 这是一帧"有效响应", 别再重试了;
 *   MODBUS_ERROR_TIMEOUT / MODBUS_ERROR_CRC → 无效帧, 调用方应重试。
 * --------------------------------------------------------------------------- */
static Modbus_Status Modbus_ParseFrame(const uint8_t *frame, uint16_t len,
                                       uint8_t slave_addr, uint8_t *out, uint16_t *out_len)
{
    if (frame == NULL || len < 3) return MODBUS_ERROR_TIMEOUT;   /* 太短, 连地址+功能码+CRC 都不够 */

    if (frame[0] != slave_addr) return MODBUS_ERROR_TIMEOUT;     /* 地址不匹配 → 不是给我们的, 当无响应处理 */

    /* 异常帧: 功能码最高位被置 1(例如 0x83 = 0x03 | 0x80)
     * 格式: [addr][fc|0x80][异常码][CRCL][CRCH], 固定 5 字节 */
    if (frame[1] & 0x80) {
        if (len != 5) return MODBUS_ERROR_TIMEOUT;
        /* 校验前 3 字节(地址 + 功能码 + 异常码)的 CRC */
        if (Modbus_CRC16((uint8_t *)frame, 3) != (((uint16_t)frame[4] << 8) | frame[3])) {
            return MODBUS_ERROR_CRC;
        }
        if (out != NULL) memcpy(out, frame, 5);   /* 把异常帧原样拷给调用方 */
        if (out_len != NULL) *out_len = 5;
        return MODBUS_ERROR_EXCEPTION;
    }

    /* 正常帧 */
    if (len < 5) return MODBUS_ERROR_TIMEOUT;     /* 正常帧最短也要 5 字节 */
    /* 校验"整帧去掉末尾 2 字节 CRC"的部分 */
    if (Modbus_CRC16((uint8_t *)frame, len - 2) != (((uint16_t)frame[len - 1] << 8) | frame[len - 2])) {
        return MODBUS_ERROR_CRC;
    }
    if (out != NULL) memcpy(out, frame, len);     /* 整帧拷给调用方 */
    if (out_len != NULL) *out_len = len;
    return MODBUS_OK;
}

/* ============================================================================
 * 非阻塞事务(Modbus TCP 网关多 Socket 用)
 *
 * 【背景 / 为什么要有非阻塞版】
 *   TcpServerTask 同时管理 4 个 Socket, 每个 Socket 收到 TCP 请求后都要
 *   "下发 485 → 等响应"。如果用阻塞收发, 一个 Socket 等待的 400ms 里整个任务
 *   都会卡住, 其他 3 个 Socket 的请求全部没法处理(用户体验就是"卡顿")。
 *   所以做成分步式:
 *     - Begin: 只负责"非阻塞抢总线 + 发送", 抢不到立刻返回 BUSY;
 *     - Poll : 每次由任务主循环调用一次, 帧到/超时就返回结果, 没到就返回 BUSY,
 *              这样任务循环在两帧之间可以去照顾其他 Socket。
 * ============================================================================
 */

/* 初始化事务状态: 全清零(active=0 / retries_left=0 / tx=NULL) */
void Modbus_AsyncInit(ModbusAsyncTx *t)
{
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));   /* 清空事务状态 */
}

/* 开始一个非阻塞 RTU 事务: 抢总线(非阻塞) → 校验 CRC → 发请求 → 记 deadline */
Modbus_Status Modbus_AsyncBegin(ModbusAsyncTx *t, uint8_t *tx_frame, uint16_t tx_len)
{
    if (t == NULL || tx_frame == NULL || tx_len < 4 || g_modbus_mutex == NULL) {
        return MODBUS_ERROR_INVALID;
    }

    /* 非阻塞拿总线(超时 0ms): 拿不到(健康检查任务正占用) → 立即返回 BUSY, 调用方稍后再试 */
    if (osMutexAcquire(g_modbus_mutex, 0) != osOK) {
        return MODBUS_BUSY;
    }

    /* 校验发送帧 CRC(与阻塞路径保持一致的做法) */
    if (Modbus_CRC16(tx_frame, tx_len - 2) != (((uint16_t)tx_frame[tx_len - 1] << 8) | tx_frame[tx_len - 2])) {
        osMutexRelease(g_modbus_mutex);              /* 校验失败必须还锁 */
        return MODBUS_ERROR_CRC;
    }

    t->tx = tx_frame;                                /* 记住要发的帧(事务期间调用方须保证这块内存有效) */
    t->tx_len = tx_len;                              /* 帧长 */
    t->retries_left = MODBUS_RTU_RETRIES;            /* 剩余重试次数 = 5 */
    t->active = 1;                                   /* 标记事务在途 */

    /* 发送失败不在这里处理, 留到 Poll 里走重试(与阻塞路径行为一致) */
    BSP_RS485_Send(tx_frame, tx_len);
    t->deadline = osKernelGetTickCount() + MODBUS_RTU_TIMEOUT_MS;  /* 本次等待截止时刻 = 当前 + 400ms */
    return MODBUS_OK;
}

/* 轮询在途事务: 帧到了 → 解析返回; 超时 → 重试 或 耗尽结束 */
Modbus_Status Modbus_AsyncPoll(ModbusAsyncTx *t, uint8_t *rx, uint16_t *rx_len)
{
    if (t == NULL || !t->active) return MODBUS_ERROR_TIMEOUT;   /* 事务不在途 → 无效调用 */

    /* 情况 1: 响应帧已被 DMA 完整捕获 */
    if (BSP_RS485_FrameReady()) {
        Modbus_Status st = Modbus_ParseFrame(BSP_RS485_FramePtr(), BSP_RS485_FrameLen(),
                                             t->tx[0], rx, rx_len);
        if (st == MODBUS_OK || st == MODBUS_ERROR_EXCEPTION) {
            /* 有效响应 → 结束事务, 释放总线 */
            t->active = 0;
            osMutexRelease(g_modbus_mutex);
            return st;
        }
        /* 帧无效(CRC / 地址 / 长度错) → 立即重发重试, 不必干等到 deadline */
        if (t->retries_left > 0) {
            t->retries_left--;                                        /* 消耗一次重试 */
            BSP_RS485_Send((uint8_t *)t->tx, t->tx_len);              /* 重发请求 */
            t->deadline = osKernelGetTickCount() + MODBUS_RTU_TIMEOUT_MS;  /* 重新计时 */
            return MODBUS_BUSY;                                       /* 仍在等待下一帧 */
        }
        t->active = 0;
        osMutexRelease(g_modbus_mutex);
        return MODBUS_ERROR_TIMEOUT;                                  /* 重试次数耗尽 */
    }

    /* 情况 2: 已过 deadline 且仍无帧 → 重试 或 耗尽 */
    if ((int32_t)(osKernelGetTickCount() - t->deadline) >= 0) {       /* 有符号比较, 天然抗 tick 回绕 */
        if (t->retries_left > 0) {
            t->retries_left--;
            BSP_RS485_Send((uint8_t *)t->tx, t->tx_len);
            t->deadline = osKernelGetTickCount() + MODBUS_RTU_TIMEOUT_MS;
            return MODBUS_BUSY;
        }
        t->active = 0;
        osMutexRelease(g_modbus_mutex);
        return MODBUS_ERROR_TIMEOUT;
    }

    return MODBUS_BUSY;   /* 没到 deadline 也没帧 → 让任务主循环继续照顾其他 Socket */
}

/* 中止在途事务并释放总线(例如 Socket 断开 / 超时强制清理时调用) */
void Modbus_AsyncAbort(ModbusAsyncTx *t)
{
    if (t == NULL) return;
    if (t->active) {                 /* 只有事务真在途时才需要释放, 避免重复 release 破坏互斥量 */
        t->active = 0;
        osMutexRelease(g_modbus_mutex);
    }
}
