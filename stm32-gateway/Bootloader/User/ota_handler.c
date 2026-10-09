/* ============================================================================
 * ota_handler.c — OTA 协议处理实现
 *
 * 【本文件的角色】
 *   故障态下 Bootloader 开了一个 TCP 服务端 (Socket 4, 端口 506), 本文件就是
 *   这个服务端的"大脑": 收到上位机推来的字节流 → 解出协议帧 → 逐块写进
 *   W25Q64 暂存区 → 全部收完校验 CRC32 → 写控制块 → 复位安装。
 *
 * 【协议帧格式 (对称于 linux-host/src/ota.c 的发送端)】
 *   命令帧 OTACmd (16B): [magic"OTAC"][cmd][rsv][fw_version][total_size][total_crc]
 *   数据帧 OTAData (12+N): [magic"OTAP"][block_num][data_len][block_crc][data(N)]
 *   响应:  单字节 (ACK=0x06 / NACK=0x15 / BUSY=0x19 / ERR_xx)
 *   所有多字节字段都是网络序(大端), 用 NTOHL/NTOHS 转换。
 *
 * 【TCP 是"字节流", 不是"消息流"】
 *   上位机 send 的 512B 数据帧, 到这边可能被拆成几次 recv, 也可能两帧粘在一起。
 *   所以必须自己维护一个环形缓冲区, 并实现"粘包解帧"逻辑。
 *
 * 【环形缓冲区的经典写法 (牺牲 1 字节区分空/满)】
 *   读写指针 & (SIZE-1) 实现绕回 (要求 SIZE 是 2 的幂, 用位与代替取模更快)。
 *   "满"的判据是 (head+1)==tail —— 即 head 主动空出 1 个字节不写,
 *   这样"空"(head==tail) 和"满"就不会混淆。最多只能存 SIZE-1 字节。
 * ============================================================================
 */
#include "ota_handler.h"    // 本模块: 环形缓冲、帧解析、状态机 API 声明
#include "bsp_w25q64.h"     // 固件暂存区读写 (STAGING_ADDR 处)
#include "bsp_w5500.h"      // 网络收发 (socket 状态/收发包)
#include "bkp_fault.h"      // 写控制块失败时置备份寄存器故障标志
#include "hw_init.h"        /* g_tick, delay_ms, IWDG_ReloadCounter */

#include "ota_crc.h"       /* CRC32_ZIP_Update, CRC16_Modbus */
#include "ota_ctrl_block.h" /* OTA_WriteCtrlBlock, OTA_MarkFail */

#include <string.h>         // memset / memcmp
#include <stdio.h>          /* printf */

/* ======================== OTA 状态变量 ======================== */
OTAStage g_ota_stage = OTA_STAGE_IDLE;      // 当前阶段: 空闲 / 接收中 / 校验中
static uint32_t g_expected_block = 0;       // 下一个期望收到的数据块号 (0-based, 用于顺序控制)
static volatile uint32_t g_last_data_ts = 0; // 最近一次收到数据的时刻(ms), 用于超时判断
volatile int g_ota_client_connected = 0;    // 是否有客户端连着 (供其它模块观察)
static int g_finish_sent = 0;               // CMD_FINISH 是否已处理过 (防重复)
static uint32_t g_ota_total_size = 0;       // 本次固件总大小 (CMD_START 时确定)
static uint32_t g_ota_total_crc = 0;        // 本次固件期望 CRC32 (CMD_START 时确定)
static uint32_t g_fw_version = 0;           // 固件版本号 (CMD_START 时带过来)
static int g_erase_fatal = 0;               // 擦除出现不可恢复错误 (超时未完成)

/* ======================== 环形缓冲区 ======================== */
static uint8_t ota_rx_buf[OTA_RX_BUF_SIZE]; // 接收环形缓冲 (2048 字节)
static volatile uint16_t ota_rx_head = 0;   // 写指针 (ISR/主循环都在动, 加 volatile)
static volatile uint16_t ota_rx_tail = 0;   // 读指针

/* ======================== 擦除异步状态 ======================== */
/* 为什么擦除要"异步": W25Q64 擦一个 4KB 扇区要约 45ms, 48KB 要擦 12 个扇区 ≈ 0.5s。
   如果在收包流程里同步等待, TCP 接收窗口会被拖垮。于是设计成:
   每轮主循环只"发起一次擦除命令"或"查一次是否擦完", 把长耗时摊到多轮里 (状态机式擦除)。 */
static volatile uint32_t g_erase_addr = 0;  // 下一个待擦扇区的地址
static uint32_t g_erase_end = 0;            // 擦除结束地址 (到这就停)
static int g_erase_in_progress = 0;         // 是否有一笔擦除正在进行中

/* ======================== 函数声明 (内部) ======================== */
/* 下面 5 个是本文件内部使用的静态函数, 提前声明避免"先使用后定义"编译错误。 */
static void send_byte(uint8_t sn, uint8_t byte);        // 发 1 字节响应
static void OTA_HandleCmd(OTACmd *cmd);                 // 处理命令帧
static void OTA_HandleData(OTAData *pkt);               // 处理数据帧
static void OTA_StartAsyncErase(uint32_t size);         // 启动异步擦除
static void OTA_TickAsyncErase(void);                   // 推进异步擦除 (每轮调用)

/* ======================== 环形缓冲区 ======================== */

/* 返回缓冲区里已缓存的字节数。
   (head - tail) & (SIZE-1) 是无符号回绕的正确写法:
   即使 head 绕回到 0 而 tail 还很大, 位与之后仍是正确的"差值"。 */
static inline uint16_t ota_buf_avail(void)
{
    return (ota_rx_head - ota_rx_tail) & (OTA_RX_BUF_SIZE - 1);
}

/* 把网络收到的原始字节塞进环形缓冲区。
   data: 刚 recv 出来的一段字节; len: 长度 */
void OTA_OnData(uint8_t *data, uint16_t len)
{
    /* 关中断保护入环 — head 会被主循环读、这里写, 关中断避免读到的 head 是"半个"
       (虽然 Cortex-M 上 16 位读写是原子的, 但 head 的"更新+判断"整体需要互斥,
        否则解帧逻辑可能看到 head 已加但数据还没写进去的中间态)。 */
    uint32_t primask = __get_PRIMASK();     // 先保存当前中断屏蔽状态
    __disable_irq();                        // 关中断 (临界区开始)

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (ota_rx_head + 1) & (OTA_RX_BUF_SIZE - 1);  // 预计算下一个 head
        if (next == ota_rx_tail) break;  /* 满, 丢弃 — 主动空 1 字节, 保证满/空可区分 */
        ota_rx_buf[ota_rx_head] = data[i];  // 写入当前字节
        ota_rx_head = next;                 // 推进写指针
        g_last_data_ts = g_tick;    /* 入环即更新时间戳 — 用于收包超时检测 */
    }

    __set_PRIMASK(primask);                 // 恢复中断屏蔽状态 (临界区结束)
}

/* ======================== 辅助发送 ======================== */

/* 发送单字节响应 (ACK/NACK/BUSY/错误码)。 */
static void send_byte(uint8_t sn, uint8_t byte)
{
    BSP_W5500_SocketSend(sn, &byte, 1);     // sn=4, 长度 1
}

/* ======================== OTA 中止 ======================== */

/* 中止当前 OTA, 复位所有状态。连接断开、超时、出错、主动取消时都调用它。 */
void OTA_Abort(void)
{
    /* 等 W25Q64 硬件空闲 — 若有一笔擦除正卡着, 强行退出可能让状态错乱。
       给最多 2 秒时间等它完成; 2 秒还没好就把 g_erase_fatal 置 1 (致命)。 */
    if (g_erase_in_progress) {
        uint32_t timeout = g_tick + 2000;           // 2 秒后超时
        while (g_tick < timeout) {
            if (BSP_W25Q64_IsBusy() == 0) {         // 硬件已空闲
                g_erase_in_progress = 0;            // 标记擦除结束
                break;
            }
            IWDG_ReloadCounter();                   // 等待期间喂狗
            delay_ms(10);                           // 10ms 轮询一次
        }
        if (g_erase_in_progress) {
            g_erase_fatal = 1;                      // 2 秒都没擦完 → 认定芯片异常
        }
    }

    /* 重置所有状态 — 全部归零, 回到刚上电的状态 */
    g_ota_stage = OTA_STAGE_IDLE;
    g_expected_block = 0;
    g_last_data_ts = 0;
    g_finish_sent = 0;
    g_ota_total_size = 0;
    g_ota_total_crc = 0;
    g_fw_version = 0;
    g_erase_fatal = 0;
    g_erase_addr = 0;
    g_erase_end = 0;
    g_erase_in_progress = 0;
    g_ota_client_connected = 0;

    /* 清环形缓冲区 — 读写指针归零 + 内存清零 (丢弃所有未解出的半包) */
    ota_rx_head = 0;
    ota_rx_tail = 0;
    memset((void*)ota_rx_buf, 0, OTA_RX_BUF_SIZE);
}

/* ======================== 命令处理 ======================== */

/* 处理"命令帧" (CMD_START / CMD_FINISH / CMD_CANCEL / CMD_STATUS)。
   cmd: 已从缓冲区按网络序读出的 16 字节命令结构 */
static void OTA_HandleCmd(OTACmd *cmd)
{
    uint8_t cmd_type = cmd->cmd;

    switch (cmd_type) {
    case CMD_START: {       /* 开始传输: 上位机告知"要发多大、CRC 多少、版本多少" */
        uint32_t size = NTOHL(cmd->total_size);     // 大端 → 主机序
        uint32_t crc  = NTOHL(cmd->total_crc);

        /* 校验 — 参数不合理直接 NACK, 拒绝开始 */
        if (size < OTA_TOTAL_MIN || size > OTA_TOTAL_MAX) {     // 范围 [1KB, 48KB]
            send_byte(4, RSP_NACK);
            break;
        }
        if (size % 2 != 0) {                        // 必须偶数 (内部 Flash 按半字写)
            send_byte(4, RSP_NACK);
            break;
        }

        /* 记录参数, 进入接收态 */
        g_ota_total_size = size;
        g_ota_total_crc  = crc;
        g_fw_version = NTOHS(cmd->fw_version);      // 16 位版本号
        g_ota_stage = OTA_STAGE_RECEIVING;          // 状态: 接收中
        g_expected_block = 0;                       // 从第 0 块开始
        g_last_data_ts = g_tick;                    // 记时, 用于超时检测
        g_finish_sent = 0;
        g_erase_fatal = 0;

        send_byte(4, RSP_ACK);                      // 回 ACK

        /* 后台启动异步擦除 — ACK 之后再擦, 不阻塞这条响应 */
        OTA_StartAsyncErase(size);
        break;
    }

    case CMD_FINISH: {      /* 传输结束: 校验整包 CRC32 → 写控制块 → 复位安装 */
        if (g_finish_sent) {                        // 重复收到 FINISH
            send_byte(4, RSP_ACK);                  // 直接再回 ACK (幂等), 不重复干活
            break;
        }
        g_finish_sent = 1;

        /* 阶段 1: 立即 ACK — 先让上位机知道"收到了", 它才好收尾。
           真正的校验很耗时 (要读 48KB 算 CRC), 放在 ACK 之后做。 */
        send_byte(4, RSP_ACK);

        /* 阶段 2: 流式 CRC32 (纯固件, 无 Header)
           —— 把暂存区里的固件整段读出来算 CRC32, 与上位机给的 g_ota_total_crc 比对。 */
        uint32_t calc_crc = 0xFFFFFFFF;             // 初值
        uint8_t crc_buf[256];

        for (uint32_t off = 0; off < g_ota_total_size; off += 256) {
            uint32_t chunk = g_ota_total_size - off;    // 剩余
            if (chunk > 256) chunk = 256;               // 每次最多 256B
            BSP_W25Q64_Read(STAGING_ADDR + off, crc_buf, chunk);    // 从暂存区读
            calc_crc = CRC32_ZIP_Update(calc_crc, crc_buf, chunk);  // 增量更新
            if ((off / 256) % 16 == 0) IWDG_ReloadCounter();        // 每 4KB 喂狗
        }
        calc_crc ^= 0xFFFFFFFF;                     // 收尾取反

        if (calc_crc != g_ota_total_crc) {
            /* CRC 不匹配 — 收到的固件坏了, 记一次失败然后复位 (让 Bootloader 重新处理/回滚) */
            OTA_MarkFail(ERR_STAGING_CRC, g_ota_total_size, g_ota_total_crc);
            NVIC_SystemReset();
            break;
        }

        /* 写控制块 — 这是"通知 Bootloader 有固件待安装"的关键一步。
           控制块 magic 一旦写上, 下次上电 Bootloader 就会执行安装。 */
        OTACtrlBlock ctrl;
        memset(&ctrl, 0xFF, sizeof(ctrl));          // 先全填 0xFF (Flash 空白值)
        ctrl.magic = OTA_CTRL_MAGIC;                // 0xA5A5A5A5
        ctrl.firmware_size = g_ota_total_size;
        ctrl.firmware_crc32 = g_ota_total_crc;
        ctrl.fail_flag[0] = 0xFF;                   // 3 次机会都在
        ctrl.fail_flag[1] = 0xFF;
        ctrl.fail_flag[2] = 0xFF;
        ctrl.last_error = ERR_NONE;                 // 无错误
        ctrl.reserved = 0xFF;
        ctrl.ctrl_crc16 = CRC16_Modbus((uint8_t*)&ctrl, CTRL_CRC_LEN);  // CRC16 只算前 12 字节

        int wret = OTA_WriteCtrlBlock(&ctrl);       // 先擦扇区→写→回读校验
        if (wret != 0) {
            BKP_WriteFaultFlag();                   // 写失败: 置备份寄存器故障标志 (掉电不丢)
        }

        /* 等 TX 空再复位
           —— 上面 ACK/可能的响应还在 W5500 发送缓冲里, 立刻复位会让它们发不出去。
              等发送缓冲空出来(≥2048 空闲)再复位, 保证上位机能收到最后一个响应。 */
        for (int i = 0; i < 50; i++) {
            if (BSP_W5500_SocketTxFree(4) >= 2048) break;    // 缓冲已空
            delay_ms(1);                                     // 最多等 50ms
        }
        NVIC_SystemReset();                         // 软复位 → Bootloader 重新上电走安装流程
        break;
    }

    case CMD_CANCEL: {      /* 上位机主动取消本次传输 */
        send_byte(4, RSP_ACK);                      // 先回 ACK
        OTA_Abort();                                // 清状态
        BSP_W5500_SocketDisconnect(4);              // 断开连接
        BSP_W5500_SocketClose(4);                   // 关掉 socket (回到 CLOSED)
        break;
    }

    case CMD_STATUS: {      /* 查询状态: 返回当前阶段和进度百分比 */
        OTAStatusResp resp;
        resp.magic = HTONL(OTA_STATUS_MAGIC);       // "OTAS" 转网络序
        resp.stage = (uint8_t)g_ota_stage;          // 当前阶段
        resp.progress_pct = (g_ota_total_size > 0)
            ? (uint8_t)((g_expected_block * 512 * 100) / g_ota_total_size)  // 已收块数*512/总大小
            : 0;                                    // 总大小为 0 时进度记 0 (防除零)
        resp.last_error = 0;                        // 这里不回具体错误码
        resp.fw_version = HTONL(g_fw_version);      // 版本号

        BSP_W5500_SocketSend(4, (const uint8_t*)&resp, sizeof(resp));   // 发 12 字节状态响应
        break;
    }

    default:                /* 未知命令码 */
        send_byte(4, RSP_NACK);
        break;
    }
}

/* ======================== 数据处理 ======================== */

/* 处理"数据帧" (OTAData: block_num + data_len + block_crc + data)。
   每帧 512 字节固件, 校验通过后写进 W25Q64 暂存区。 */
static void OTA_HandleData(OTAData *pkt)
{
    if (g_ota_stage != OTA_STAGE_RECEIVING) {       // 没在接收态就收到数据 → 拒绝
        send_byte(4, RSP_NACK);
        return;
    }

    uint32_t block_num = NTOHL(pkt->block_num);     // 块号 (网络序 → 主机序)
    uint16_t data_len  = NTOHS(pkt->data_len);      // 本帧数据长度
    uint16_t block_crc = NTOHS(pkt->block_crc);     // 本帧 CRC16

    /* 越界检查 — 块号*512 已超出总大小, 说明上位机发多了/算错了, 中止 */
    if (block_num * OTA_PACKET_SIZE >= g_ota_total_size) {
        send_byte(4, RSP_ERR_OOB);                  // OOB = Out Of Bounds
        OTA_Abort();
        return;
    }

    if (g_erase_fatal) {                            // 之前擦除出过致命错误
        send_byte(4, RSP_ERR_ERASE);
        return;
    }

    /* 扇区擦除检查 — 目标扇区还没轮到擦除, 就让上位机稍后重发 (BUSY)。
       g_erase_addr 是"已擦到哪"的进度; 本帧目标扇区 >= 进度 → 说明还没擦到, 拒绝写。 */
    uint32_t w25_addr = STAGING_ADDR + block_num * OTA_PACKET_SIZE;         // 目标 W25Q64 地址
    uint32_t sector_base = w25_addr & ~(W25Q64_SECTOR_SIZE - 1);            // 对齐到扇区首地址
    if (sector_base >= g_erase_addr) {              // 目标扇区尚未擦除
        send_byte(4, RSP_BUSY);
        return;
    }

    /* CRC16 校验 — 逐帧校验, 数据错了立刻发现, 不必等到最后 */
    uint16_t calc = CRC16_Modbus(pkt->data, data_len);
    if (calc != block_crc) {
        send_byte(4, RSP_NACK);
        return;
    }

    /* 三态序号处理 (处理 TCP 重传/乱序) — "滑动窗口退化版":
         block_num < 期望值 → 重复帧, 之前已写过, 直接回 ACK (上位机可继续)
         block_num > 期望值 → 超前帧, 前面有缺, 丢弃不 ACK (等上位机重传缺失块)
         block_num == 期望值 → 正常帧, 往下继续处理 */
    if (block_num < g_expected_block) {
        send_byte(4, RSP_ACK);                      // 重复帧: 幂等回 ACK
        return;
    }
    if (block_num > g_expected_block) {
        return;  /* 丢弃, 等重传 — 不回任何响应 */
    }

    /* 异步擦除进行中，拒绝写入（用软件标志代替硬件 IsBusy，
       避免 SPI 事务干扰后续 WriteEnable）
       —— 如果此刻直接问 W5500 的 IsBusy, 会和正在跑的擦除命令"抢"SPI 总线,
          导致后面 WriteEnable 时序乱掉。所以用软件标志 g_erase_in_progress 挡。 */
    if (g_erase_in_progress) {
        send_byte(4, RSP_BUSY);
        return;
    }

    /* 写入 W25Q64（带重试）
       —— SPI Flash 偶发写失败不罕见, 必须"写→回读→比对", 最多重试 3 次。 */
    int write_ok = 0;
    for (int retry = 0; retry < 3; retry++) {
        BSP_W25Q64_Write(w25_addr, pkt->data, data_len);

        uint8_t verify[OTA_PACKET_SIZE];            // 回读缓冲
        BSP_W25Q64_Read(w25_addr, verify, data_len);
        if (memcmp(pkt->data, verify, data_len) == 0) {     // 回读一致 → 写成功
            write_ok = 1;
            break;
        }
        /* 写入不匹配，短暂等待后重试 — 纯空转延时, 给 Flash 内部状态机一点时间 */
        for (volatile int d = 0; d < 500; d++);
    }
    if (!write_ok) {                                // 3 次都失败 → 硬件有问题
        send_byte(4, RSP_ERR_WRITE);
        OTA_Abort();                                // 中止本次 OTA
        return;
    }

    g_expected_block++;                             // 期望块号 +1 (推进)
    send_byte(4, RSP_ACK);                          // 回 ACK, 让上位机发下一块
}

/* ======================== 异步擦除 ======================== */

/* 启动异步擦除 — 只是把"要擦的范围"记下来; 真正的擦除动作交给 OTA_TickAsyncErase
   在主循环里一轮一轮推进。这里不阻塞。 */
static void OTA_StartAsyncErase(uint32_t size)
{
    g_erase_addr = STAGING_ADDR;                    // 从暂存区起始开始擦
    g_erase_end = (STAGING_ADDR + size + W25Q64_SECTOR_SIZE - 1)    // 按 4KB 向上对齐
                 & ~(W25Q64_SECTOR_SIZE - 1);
    if (g_erase_end > LOG_AREA_ADDR) g_erase_end = LOG_AREA_ADDR;   // 不越过日志区上界
    g_erase_in_progress = 0;                        // 尚未开始 (下一轮 Tick 才发第一条命令)
}

/* 推进异步擦除 — 主循环每轮调一次, 每次只做"发一条命令"或"查一次完成"。
   这就是状态机式擦除: 把 0.5 秒的擦除摊成多次非阻塞调用, 不卡住收包。 */
static void OTA_TickAsyncErase(void)
{
    if (g_erase_addr >= g_erase_end) return;        // 全部擦完了

    if (!g_erase_in_progress) {                     // 没在擦 → 发一条擦除命令 (非阻塞)
        BSP_W25Q64_SendEraseCmd(g_erase_addr);      // 只发命令, 不等完成
        g_erase_in_progress = 1;                    // 标记"正在擦"
        return;
    }

    if (BSP_W25Q64_IsBusy() == 0) {                 // 上一笔擦除已完成
        g_erase_addr += W25Q64_SECTOR_SIZE;         // 进度推进一个扇区
        g_erase_in_progress = 0;                    // 允许发下一条
        IWDG_ReloadCounter();                       // 喂狗
    }
}

/* ======================== OTA Tick ======================== */

/* 主循环节拍函数: 推进异步擦除 + 检测收包超时。 */
void OTA_Tick(void)
{
    OTA_TickAsyncErase();                           // 先推进擦除

    if (g_ota_stage != OTA_STAGE_IDLE) {            // 非空闲态才有超时概念
        if (g_tick - g_last_data_ts > OTA_TIMEOUT_MS) {     // 超过 5 秒没收到数据
            OTA_Abort();                            // 认为连接断了, 清状态
            BSP_W5500_SocketDisconnect(4);          // 主动断开
        }
    }
}

/* ======================== 循环解帧辅助 ======================== */

/* 尝试从环形缓冲区头部解出一帧并处理 (核心的"粘包解帧"逻辑)。
   返回 1 表示"消费了数据"(可能是解出一帧, 也可能是丢弃了 1 字节垃圾);
   返回 0 表示"数据不够, 等下次再收"。

   解帧思路: 先看头 4 字节的 magic 是什么, 据此判断帧类型和帧长;
   数据不够就返回 0 等; 数据够了就整帧拷贝出来、移动 tail、交给处理器。 */
int ota_try_parse_frame(void)
{
    uint16_t avail = ota_buf_avail();               // 当前可读字节数
    if (avail < 8) return 0;                        // 连 magic(4)+最少字段都不够

    /* 读头 4 字节 magic — 逐字节从环形缓冲取, 处理绕回 */
    uint32_t magic;
    uint8_t *pm = (uint8_t *)&magic;                // 用字节指针逐字节组装 (不关心对齐)
    pm[0] = ota_rx_buf[ota_rx_tail];                                    // 第 0 字节
    pm[1] = ota_rx_buf[(ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1)];      // 第 1 字节 (可能绕回)
    pm[2] = ota_rx_buf[(ota_rx_tail + 2) & (OTA_RX_BUF_SIZE - 1)];      // 第 2 字节
    pm[3] = ota_rx_buf[(ota_rx_tail + 3) & (OTA_RX_BUF_SIZE - 1)];      // 第 3 字节

    if (magic == OTA_CMD_MAGIC) {                   // 是命令帧 "OTAC"
        if (avail < sizeof(OTACmd)) return 0;       // 帧还没收全 (16 字节)
        OTACmd cmd;
        for (uint32_t i = 0; i < sizeof(OTACmd); i++) {                 // 逐字节拷进结构体
            ((uint8_t*)&cmd)[i] = ota_rx_buf[(ota_rx_tail + i) & (OTA_RX_BUF_SIZE - 1)];
        }
        ota_rx_tail = (ota_rx_tail + sizeof(OTACmd)) & (OTA_RX_BUF_SIZE - 1);   // 移动读指针
        OTA_HandleCmd(&cmd);                        // 处理命令帧
        return 1;
    }

    if (magic == OTA_DATA_MAGIC) {                  // 是数据帧 "OTAP"
        if (avail < 12) return 0;                   // 头 12 字节还没到齐 (不含 payload)
        uint16_t dlen;
        uint8_t *pd = (uint8_t *)&dlen;
        pd[0] = ota_rx_buf[(ota_rx_tail + 8) & (OTA_RX_BUF_SIZE - 1)];      // data_len 高字节
        pd[1] = ota_rx_buf[(ota_rx_tail + 9) & (OTA_RX_BUF_SIZE - 1)];      // data_len 低字节
        dlen = NTOHS(dlen);                         // 网络序 → 主机序
        if (dlen > OTA_PACKET_SIZE) {               // 长度非法 (超过 512)
            ota_rx_tail = (ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1);    // 丢 1 字节, 尝试重新同步
            return 1;
        }
        uint32_t total_pkt = 12 + dlen;             // 整帧长度 = 头 12 + 数据
        if (avail < total_pkt) return 0;            // 整帧还没收全

        OTAData pkt;
        for (uint32_t i = 0; i < total_pkt; i++) {  // 逐字节拷整帧
            ((uint8_t*)&pkt)[i] = ota_rx_buf[(ota_rx_tail + i) & (OTA_RX_BUF_SIZE - 1)];
        }
        ota_rx_tail = (ota_rx_tail + total_pkt) & (OTA_RX_BUF_SIZE - 1);    // 移动读指针
        OTA_HandleData(&pkt);                       // 处理数据帧
        return 1;
    }

    /* magic 不匹配 — 说明错位了, 丢掉 1 字节继续找下一帧头 (滑动窗口重同步)。
       这就是"垃圾字节自动跳过"的机制。 */
    ota_rx_tail = (ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1);
    return 1;
}

/* ======================== Socket 4 轮询 ======================== */

/* 查询 Socket 4 的连接状态, 按状态做相应处理。
   W5500 的 socket 是一台"状态机", 这里就是它的驱动核心。 */
void OTA_PollSocket4(void)
{
    uint8_t status = BSP_W5500_SocketStatus(4);     // 读 Sn_SR (socket 状态寄存器)

    switch (status) {
    case W5500_Sn_SR_CLOSED:                        // 已关闭 (或尚未打开)
    case W5500_Sn_SR_INIT:                          // 刚初始化
        BSP_W5500_SocketInit(4, OTA_PORT);          // 重新配成 TCP + 端口 506
        BSP_W5500_SocketListen(4);                  // 进 LISTEN 等客户端
        break;

    case W5500_Sn_SR_ESTABLISHED:                   // 连接已建立 → 可以收数据
        g_ota_client_connected = 1;
        /* 接收数据 — 循环把当前所有可收字节都收干净 */
        while (1) {
            uint16_t avail = BSP_W5500_SocketRxAvail(4);    // 可读字节数
            if (avail == 0) break;                          // 收完了
            uint8_t buf[1024];                              // 栈上临时缓冲
            uint16_t recv = BSP_W5500_SocketRecv(4, buf,
                avail > sizeof(buf) ? sizeof(buf) : avail);// 一次最多收 1024
            OTA_OnData(buf, recv);                          // 入环形缓冲

            /* 解帧（可能有粘包）— 反复解直到解不出为止 */
            while (ota_try_parse_frame()) {}
        }
        break;

    case W5500_Sn_SR_CLOSE_WAIT:                    // 对端发起关闭 → 收尾
        g_ota_client_connected = 0;
        OTA_Abort();                                // 中止本次 OTA (未完成的就作废)
        BSP_W5500_SocketDisconnect(4);              // 我方也断开
        BSP_W5500_SocketClose(4);                   // 关闭 socket
        break;

    case W5500_Sn_SR_TIME_WAIT:                     // 等待 2MSL 后彻底关闭
        BSP_W5500_SocketClose(4);                   // 直接关闭 (嵌入式不等 2MSL)
        break;
    }
    /* 其它状态 (LISTEN/SYNSENT/...): 不动, 等它自己迁移 */
}
