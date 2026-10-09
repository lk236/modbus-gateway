/* ============================================================================
 * ota_receive.c — OTA 固件接收模块(升级流程的"接收端")
 * ============================================================================
 *
 * 上位机通过 W5500 的 Socket 4(端口 506)把新固件推给网关。网关不能直接擦写
 * 自己正在运行的代码区, 所以分两步走:
 *   ① 接收: 把固件先写进 W25Q64 的"暂存区"(STAGING_ADDR = 0x010000);
 *   ② 移交: 校验通过后在 W25Q64 写"控制块", 标记"暂存区有待升级固件",
 *           然后 NVIC_SystemReset() 复位。复位后由 Bootloader 接手, 检查
 *           控制块, 把暂存区固件烧进 App 区。
 *
 * 支持的命令:
 *   START  : 携带固件总大小 + CRC32 + 版本号 → 擦暂存区 → ACK
 *   DATA   : 512 字节/块, 每块带 CRC16 → 校验 → 写入 → 回读验证 → ACK
 *   FINISH : 从暂存区流式重算 CRC32 与 START 时的值比对 → 一致则写控制块 → 复位
 *   CANCEL : 放弃本次升级
 *   STATUS : 查询当前阶段与进度百分比
 *
 * 数据先入环形缓冲区: TCP 是"字节流", 没有消息边界。一次 SocketRecv 可能读到
 * 半个包、一个半包(粘包)或三个包(粘包),
 *   所以不能"读一次就当一个包处理", 必须先进环形缓冲, 再按 magic 逐帧切分。
 *   见 OTA_OnData 里的粘包解析逻辑。
 *
 * 【总线说明】W5500 走 SPI2, W25Q64 走 SPI1, 是两条独立的 SPI 总线, 互不干扰;
 *   W25Q64 的访问由 BSP_W25Q64 内部的互斥锁保护, 与日志任务共享是安全的。
 *
 * ───────────────────────── 一次完整升级的状态机 ─────────────────────────
 *   IDLE
 *     └─ 收到 START(含总大小/CRC32) → 同步擦除暂存区(4KB/扇区) → ACK
 *         └─ RECEIVING
 *              ├─ DATA×N(512B/块, 每块带 CRC16):
 *              │    校验 CRC → 越界/OOB检查 → 写 W25Q64 → 回读验证 → ACK
 *              │    每个 socket 拆包的 TCP 流先进环形缓冲, 按 magic 切包(防粘包)
 *              └─ FINISH → 从暂存区流式重算 CRC32 → 与 t_start 对比
 *                    ├─ 一致   → 写 OTA 控制块(标记有待升级固件) → 复位, 进 Bootloader
 *                    └─ 不一致 → 写控制块失败标记 → 复位, Bootloader 记录失败计数
 *
 * 数据入口 OTA_OnData 由 TcpServerTask 在 Socket 4 收到数据时调用。
 * ============================================================================
 */
#include "ota_receive.h"   /* 本模块接口 + OTA 协议公共定义 */
#include "ota_crc.h"       /* CRC16_Modbus / CRC32_ZIP_Update */
#include "bsp_w5500.h"     /* Socket 收发: Send/Recv/Disconnect/TxFree */
#include "bsp_w25q64.h"    /* Flash 读写擦 + 忙状态查询 */
#include <stdio.h>         /* printf 调试打印 */
#include <string.h>        /* memcpy / memset / memcmp */
#include "cmsis_os2.h"     /* osThreadXxx / osKernelGetTickCount */

/* ======================== 适配宏 ========================
 * ota_ctrl_block.h 是"平台无关"的公共头文件, 它不直接调用任何 Flash 驱动,
 * 而是通过下面这几个宏间接调用 —— 谁包含它, 就由谁提供宏定义。
 * 这样 Bootloader 和 App 可以共用同一份控制块代码, 各接各的驱动。
 * (这就是 C 语言里实现"依赖注入"的常用手法: 用宏代替函数指针, 零开销。) */

#define OTA_FLASH_READ(addr, buf, len)       BSP_W25Q64_Read(addr, buf, len)
#define OTA_FLASH_WRITE(addr, buf, len)      BSP_W25Q64_Write(addr, buf, len)
#define OTA_FLASH_SECTOR_ERASE(addr)         BSP_W25Q64_SectorErase(addr)
#define OTA_FLASH_IS_BUSY()                  BSP_W25Q64_IsBusy()

/* 在 App 中, BSP_W25Q64_WaitBusy 自带超时, 传 50ms 表示"每次等待最多阻塞 50ms" */
#define OTA_FLASH_WAIT_BUSY()                BSP_W25Q64_WaitBusy(50)

/* OTA 控制块操作(全是 static inline 函数, 必须等上面的 OTA_FLASH_* 宏定义好再包含) */
#include "ota_ctrl_block.h"

/* ======================== 参数 ======================== */
#define OTA_SOCKET          4        /* 专用于 OTA 的 Socket 编号 */
#define OTA_RX_BUF_SIZE     2048     /* 接收环形缓冲大小(必须为 2 的幂, 见取模优化) */
#define OTA_TIMEOUT_MS      5000     /* 5s 没收到任何数据 → 判定升级中断, 自动中止 */

/* ======================== 环形缓冲区 ========================
 * 用 head/tail 两个"游标"实现, 容量 OTA_RX_BUF_SIZE(牺牲 1 字节用于区分空/满)。
 * 【为什么加 volatile?】head/tail 会被 OTA_OnData(任务上下文)读写, 而缓冲内容
 * 也可能被中断看到, volatile 防止编译器把它们缓存到寄存器导致判断失效。 */
static volatile uint8_t  ota_rx_buf[OTA_RX_BUF_SIZE];
static volatile uint16_t ota_rx_head;   /* 写入位置(生产者: OTA_OnData 入队) */
static volatile uint16_t ota_rx_tail;   /* 读出位置(消费者: 帧解析出队) */

/* ======================== 状态 ======================== */
static OTAStage     g_stage = OTA_STAGE_IDLE;   /* 当前阶段: 空闲/接收中 */
static uint32_t     g_total_size;      /* 本次固件总字节数(来自 START) */
static uint32_t     g_total_crc;       /* 本次固件整体 CRC32(来自 START, FINISH 时比对) */
static uint32_t     g_fw_version;      /* 固件版本号 */
static uint32_t     g_expected_block;  /* 下一个期待的块号(用于顺序校验) */
static volatile uint32_t g_last_data_ts;   /* 最近一次收到数据的时刻(超时判断用) */
static volatile int g_cancel_flag;     /* 取消标志 */
static int          g_ota_in_progress; /* 是否正在升级(日志任务据此暂停写日志) */
static int          g_finish_sent;     /* FINISH 是否已处理过(保证重复 FINISH 幂等) */

/* ======================== 异步擦除状态 ======================== */
static uint32_t g_erase_addr;          /* 异步擦除的下一个扇区地址 */
static uint32_t g_erase_end;           /* 异步擦除的结束地址 */
static int      g_erase_in_progress;   /* 是否有一笔扇区擦除正在芯片里进行 */
static int      g_erase_fatal;         /* 擦除发生不可恢复错误(P_FAIL)标志 */

/* ======================== 喂狗 ======================== */
/* IWDG 喂狗(定义在 bsp_iwdg.c) */
extern void BSP_IWDG_Reload(void);
#define IWDG_Reload()  BSP_IWDG_Reload()

/* ======================== 静态函数声明 ======================== */
static void OTA_HandleCmd(uint8_t sn, const OTACmd *cmd);
static void OTA_HandleData(uint8_t sn, const OTAData *pkt);
static void OTA_ResetState(void);
static void OTA_EraseStagingSync(void);
static void OTA_TickAsyncErase(void);
static int  OTA_IsSectorErased(uint32_t addr);
static void OTA_Send1Byte(uint8_t sn, uint8_t code);

/* ======================== 初始化 ======================== */

/* 中止当前 OTA: 清状态 + 断开 OTA Socket */
void OTA_Abort(void)
{
    OTA_ResetState();                          /* 状态机复位 */
    BSP_W5500_SocketDisconnect(OTA_SOCKET);    /* 断开 506 连接 */
}

/* 模块初始化(TcpServerTask 启动时调用一次) */
void OTA_Init(void)
{
    ota_rx_head = 0;                           /* 环形缓冲游标归零 */
    ota_rx_tail = 0;
    memset((void*)ota_rx_buf, 0, OTA_RX_BUF_SIZE);   /* 缓冲清零(0 值便于调试观察) */
    OTA_ResetState();                          /* 状态机复位 */
    printf("[OTA] Module initialized\r\n");
}

/* ======================== 数据入口 ======================== */

/*
 * OTA_OnData — Socket 4 收到原始字节流时的入口
 *
 * 两个步骤:
 *   ① 把字节全部压进环形缓冲(用临界区保护, 因为可能被中断抢占);
 *   ② 循环从缓冲里"切包": 按 magic 识别帧头, 凑齐一帧就交给对应处理函数。
 *      凑不齐就退出, 等下一次 SocketRecv 补数据(这就是处理粘包的思路)。
 */
void OTA_OnData(uint8_t sn, const uint8_t *data, uint16_t len)
{
    printf("[DBG_OTA] OTA_OnData sn=%u len=%u\r\n", sn, len);

    /* ---- 步骤 ①: 数据入环形缓冲区(临界区保护) ----
     * __get_PRIMASK() 先保存当前中断屏蔽状态, __disable_irq() 关中断,
     * 结束后 __set_PRIMASK(saved) 精确恢复 —— 比"直接开中断"更安全, 因为
     * 本函数可能在"本来就关着中断"的上下文里被调用, 那样直接开中断会出问题。 */
    uint32_t saved = __get_PRIMASK();
    __disable_irq();

    for (uint16_t i = 0; i < len; i++) {
        uint16_t next = (ota_rx_head + 1) & (OTA_RX_BUF_SIZE - 1);
        if (next == ota_rx_tail) break;   /* 下一个写位置撞上读位置 → 缓冲满, 丢弃剩余字节 */
        ota_rx_buf[ota_rx_head] = data[i];
        ota_rx_head = next;               /* 写入后指针后移(到尾自动回绕) */
        g_last_data_ts = osKernelGetTickCount();  /* 只要收到字节就刷新"活跃时间", 防误判超时 */
    }

    __set_PRIMASK(saved);                 /* 恢复原来的中断屏蔽状态 */

    /* ---- 步骤 ②: 循环解析帧(处理粘包/半包) ---- */
    while (1) {
        /* 缓冲区里现存多少字节未解析。
         * 用 & (SIZE-1) 代替 % SIZE: 因为 SIZE 是 2 的幂, 位与比取模快得多,
         * 而且能自然地处理回绕(head < tail 的情况)。 */
        uint16_t avail = (ota_rx_head - ota_rx_tail) & (OTA_RX_BUF_SIZE - 1);
        if (avail < 8) break;   /* 连最小的帧头都不够 → 等下次数据 */

        /* 先"看一眼"帧头 magic, 但【不弹出】缓冲(如果帧不完整, 下次还要用) */
        uint32_t magic;
        memcpy(&magic, (void*)&ota_rx_buf[ota_rx_tail], 4);
        magic = NTOHL(magic);  /* TCP 是大端, STM32 是小端, 需要反转字节序再比较 */

        if (magic == OTA_CMD_MAGIC) {
            /* ---- 命令帧: 固定 16 字节 ---- */
            if (avail < sizeof(OTACmd)) break;   /* 还不完整, 退出等更多数据 */

            OTACmd cmd;
            /* 逐字节从环形缓冲搬到结构体(不能直接 memcpy, 因为跨回绕点不连续) */
            for (uint16_t i = 0; i < sizeof(OTACmd); i++) {
                *((uint8_t*)&cmd + i) = ota_rx_buf[ota_rx_tail];
                ota_rx_tail = (ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1);
            }
            OTA_HandleCmd(sn, &cmd);   /* 交给命令处理 */
        }
        else if (magic == OTA_DATA_MAGIC) {
            /* ---- 数据帧: 变长, 必须先从帧头读出 data_len 才知道整帧多长 ---- */
            if (avail < 12) break;   /* 至少要有 12 字节包头才能解析出 data_len */

            /* 【历史遗留】下面这段用 memcpy 从偏移 +10 读 2 字节, 偏移是错的
             *   (data_len 实际在 +8), 读出来的值随即被下一段"安全方式"覆盖,
             *   所以是无效代码。保留原样不动, 以安全方式为准。 */
            uint16_t dlen;
            memcpy(&dlen, (void*)&ota_rx_buf[(ota_rx_tail + 10) & (OTA_RX_BUF_SIZE - 1)], 2);

            /* 正确方式: 逐字节取 +8/+9 两个字节, 大端拼成 data_len。
             * 用"逐字节"而不是 memcpy, 是为了处理缓冲回绕(不保证连续)。 */
            uint8_t dlen_buf[2];
            dlen_buf[0] = ota_rx_buf[(ota_rx_tail + 8) & (OTA_RX_BUF_SIZE - 1)];
            dlen_buf[1] = ota_rx_buf[(ota_rx_tail + 9) & (OTA_RX_BUF_SIZE - 1)];
            dlen = (dlen_buf[0] << 8) | dlen_buf[1];   /* 大端 → 主机序 */

            uint16_t total = 12 + dlen;                /* 整帧长度 = 包头 12 + 数据 dlen */
            if (avail < total) break;                  /* 整帧还没到齐 → 等下次 */

            OTAData pkt;   /* 注意: 这个结构体含 512 字节 data[], 是栈占用大户 */
            for (uint16_t i = 0; i < 12; i++) {
                *((uint8_t*)&pkt + i) = ota_rx_buf[ota_rx_tail];
                ota_rx_tail = (ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1);
            }
            for (uint16_t i = 0; i < dlen; i++) {
                pkt.data[i] = ota_rx_buf[ota_rx_tail];
                ota_rx_tail = (ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1);
            }
            OTA_HandleData(sn, &pkt);   /* 交给数据处理 */
        }
        else {
            /* 帧头不认识: 说明之前可能丢了字节(dropped), 导致字节流"错位"。
             * 做法是滑窗前进 1 字节, 重新去寻找下一个正确的 magic —— 
             * 这是从 TCP 字节流中重新同步(re-sync)的标准手法。 */
            ota_rx_tail = (ota_rx_tail + 1) & (OTA_RX_BUF_SIZE - 1);
        }
    }
}

/* ======================== 发送 ======================== */

/* 发送 1 字节响应码(ACK/NACK/BUSY/各种错误码) */
void OTA_SendResponse(uint8_t sn, uint8_t code)
{
    uint8_t buf[1] = { code };
    BSP_W5500_SocketSend(sn, buf, 1);
}

/* 内部用的短名包装(等价于 OTA_SendResponse, 让调用处更简洁) */
static void OTA_Send1Byte(uint8_t sn, uint8_t code)
{
    OTA_SendResponse(sn, code);
}

/* ======================== 命令处理 ======================== */

static void OTA_HandleCmd(uint8_t sn, const OTACmd *cmd)
{
    printf("[DBG_OTA] OTA_HandleCmd sn=%u cmd=0x%02X\r\n", sn, cmd->cmd);

    /* 命令帧里的多字节字段都是网络序(大端), 先统一转成主机序 */
    uint32_t total_size = NTOHL(cmd->total_size);
    uint32_t total_crc  = NTOHL(cmd->total_crc);

    switch (cmd->cmd) {
    case CMD_START:
        printf("[OTA] START size=%lu crc=0x%08lX\r\n", total_size, total_crc);

        /* ---- 参数合法性校验 ---- */
        if (total_size < OTA_TOTAL_MIN || total_size > OTA_TOTAL_MAX) {
            /* 小于 1KB 或大于 48KB(暂存区容量上限) → 拒绝 */
            OTA_Send1Byte(sn, RSP_NACK);
            printf("[OTA] REJECT: size %lu out of range\r\n", total_size);
            break;
        }
        if (total_size % 2 != 0) {
            /* 固件必须半字对齐(STM32 按 16 位取指, 奇数长度不利于对齐处理) → 拒绝 */
            OTA_Send1Byte(sn, RSP_NACK);
            printf("[OTA] REJECT: odd size\r\n");
            break;
        }

        /* 如果之前有未完成的 OTA, 先中止并清理 */
        OTA_ResetState();

        /* ---- 记录本次升级的参数, 进入接收阶段 ---- */
        g_total_size = total_size;
        g_total_crc = total_crc;
        g_fw_version = NTOHS(cmd->fw_version);
        printf("[OTA] START fw_version=0x%04X\r\n", (unsigned)g_fw_version);
        g_expected_block = 0;        /* 期待第 0 块 */
        g_ota_in_progress = 1;       /* 标记"升级进行中" */
        g_finish_sent = 0;
        g_cancel_flag = 0;
        g_erase_fatal = 0;
        g_stage = OTA_STAGE_RECEIVING;   /* 阶段切到"接收中" */
        g_last_data_ts = osKernelGetTickCount();   /* 刷新活跃时间 */

        /* 【关键设计: 为什么在 START 阶段就同步擦除整片暂存区, 而不是边收边擦?】
         *   实测发现: 如果在传输期间做扇区擦除, 会出现"擦除窗口内发出的页编程
         *   命令被芯片静默丢弃", 导致整块回读都是 0xFF, 而且重试也没用。
         *   所以这里改成"一开始就把整片擦干净, 传输期间绝不擦除":
         *     - 不再有 BUSY 风暴;
         *     - 也没有"擦除窗口写入被丢弃"的问题。
         *   代价是 START 响应会慢(擦 48KB 约需数秒), 所以主机侧 START 应答
         *   超时必须 ≥5s(ota.c 里已改为 10s)。 */
        printf("[OTA] START accepted, erasing synchronously...\r\n");
        OTA_EraseStagingSync();      /* 同步擦除整片暂存区 */
        OTA_Send1Byte(sn, RSP_ACK);  /* 擦完才 ACK, 告知主机"可以开始发数据了" */
        break;

    case CMD_FINISH:
        printf("[OTA] FINISH\r\n");
        if (g_finish_sent) {
            OTA_Send1Byte(sn, RSP_ACK);  /* 幂等: 重复的 FINISH 直接回 ACK, 不再重复校验 */
            break;
        }
        if (g_stage != OTA_STAGE_RECEIVING) {
            /* 不在接收阶段 → 说明没先发 START, 拒绝 */
            OTA_Send1Byte(sn, RSP_NACK);
            break;
        }

        g_finish_sent = 1;   /* 标记 FINISH 已处理 */

        /* ---- 流式 CRC32 校验 ----
         * 把暂存区里的固件按 256 字节一块读出来, 增量计算 CRC32,
         * 最后与 START 时主机给的 g_total_crc 比对。
         * 之所以叫"流式", 是因为不需要把整个固件读进 RAM(48KB 太大),
         * 而是边读边算, 只占 256 字节的缓冲。 */
        {
            /* 先读前 16 字节, 确认 W25Q64 里确实有数据(调试用) */
            uint8_t head[16];
            W25Q64_Status rd_st = BSP_W25Q64_Read(STAGING_ADDR, head, 16);
            printf("[DBG_CRC] W25Q64 read @0x%06lX ret=%d", STAGING_ADDR, rd_st);
            for (int k = 0; k < 16; k++) printf(" %02X", head[k]);
            printf("\r\n");
        }

        uint32_t calc_crc = 0xFFFFFFFF;   /* CRC32-ZIP 的初值 */
        for (uint32_t off = 0; off < g_total_size; off += 256) {
            uint8_t buf[256];
            /* 最后一块可能不足 256 字节, 取剩余长度 */
            uint32_t chunk = (g_total_size - off > 256) ? 256 : (g_total_size - off);
            W25Q64_Status rd_st = BSP_W25Q64_Read(STAGING_ADDR + off, buf, chunk);
            if (rd_st != W25Q64_OK) {
                printf("[DBG_CRC] Read FAIL at off=%lu ret=%d\r\n", off, rd_st);
                break;   /* 读失败: 跳出, calc_crc 会是个不匹配的值 → 走失败分支 */
            }
            calc_crc = CRC32_ZIP_Update(calc_crc, buf, chunk);   /* 增量更新 */
            if (off == 0) {
                printf("[DBG_CRC] after 1st chunk: crc=0x%08lX (expect change from FFFFFFFF)\r\n", calc_crc);
            }
            if ((off / 256) % 16 == 0) {
                IWDG_Reload();  /* 每 16 块(即 4KB)喂一次狗, 防止长时间校验触发看门狗复位 */
            }
        }
        calc_crc ^= 0xFFFFFFFF;   /* CRC32-ZIP 的最后一步: 结果取反 */

        /* === CRC DEBUG: 把前 256 字节 dump 出来, 便于和主机侧比对 === */
        printf("[DBG_DUMP] ");
        uint8_t dump_buf[256];
        BSP_W25Q64_Read(STAGING_ADDR, dump_buf, 256);
        for (int k = 0; k < 256; k++) 
        {
            printf("%02X", dump_buf[k]);
        }
            printf("\r\n");    
        if (calc_crc != g_total_crc) {
            /* ---- CRC 校验失败: 说明固件在传输/存储环节出了问题 ---- */
            printf("[OTA] CRC mismatch: calc=0x%08lX expect=0x%08lX\r\n",
                   calc_crc, g_total_crc);
            OTA_Send1Byte(sn, RSP_NACK);   /* 先告诉主机失败 */
            osDelay(10);                   /* 留点时间让 TCP 把 NACK 发出去 */
            /* 标记失败(不清暂存区), 然后复位让 Bootloader 记录 fail_flag。
             * 这样即使这次升级失败, 旧固件仍然可以正常运行。 */
            if (OTA_MarkFail(ERR_STAGING_CRC, g_total_size, g_total_crc) != 0) {
                printf("[OTA] MarkFail write error\r\n");
            }
            NVIC_SystemReset();   /* 复位, 进入 Bootloader */
            break;
        }

        printf("[OTA] CRC OK, writing control block...\r\n");

        /* CRC 通过, 通知上位机"校验成功" */
        OTA_Send1Byte(sn, RSP_ACK);

        /* ---- 构造 OTA 控制块并写入 W25Q64 ----
         * 控制块是"交接棒": 它告诉 Bootloader "暂存区里有一份校验通过的固件,
         * 请把它烧进 App 区"。magic = 0xA5A5A5A5 表示有效。 */
        OTACtrlBlock ctrl;
        memset(&ctrl, 0xFF, sizeof(ctrl));   /* 先全填 0xFF(Flash 擦除后的状态) */
        ctrl.magic = OTA_CTRL_MAGIC;
        ctrl.firmware_size = g_total_size;
        ctrl.firmware_crc32 = g_total_crc;
        ctrl.fail_flag[0] = 0xFF;            /* 三次机会初始都是 0xFF(未用) */
        ctrl.fail_flag[1] = 0xFF;
        ctrl.fail_flag[2] = 0xFF;
        ctrl.last_error = ERR_NONE;
        ctrl.reserved = 0xFF;
        /* 前 12 字节的 CRC16(不含 fail_flag/last_error, 因为那两个要单独 1→0 修改) */
        ctrl.ctrl_crc16 = CRC16_Modbus((uint8_t*)&ctrl, CTRL_CRC_LEN);

        /* 写控制块(含 擦除→写字段→最后写magic→memcmp回读→失败清magic 的完整流程) */
        if (OTA_WriteCtrlBlock(&ctrl) != 0) {
            printf("[OTA] WriteCtrlBlock failed\r\n");
            OTA_ResetState();
            break;
        }

        printf("[OTA] Upgrade triggered! Rebooting...\r\n");

        /* 延时 20ms 让 ACK 有时间发出去, 否则复位太快主机收不到响应 */
        osDelay(20);
        /* 再等 Socket 的发送缓冲真正清空(TxFree 恢复到 ≥2KB 说明数据已发走) */
        for (int i = 0; i < 50; i++) {
            if (BSP_W5500_SocketTxFree(sn) >= 2048) break;
            osDelay(1);
        }

        /* W5500 软复位(让网络芯片也干净地重启) + 系统复位 → 交给 Bootloader */
        BSP_W5500_WriteReg(W5500_BLOCK_COMMON, W5500_MR, W5500_MR_RST);
        osDelay(10);
        NVIC_SystemReset();   /* 触发 Cortex-M3 的系统复位 */
        break;

    case CMD_CANCEL:
        printf("[OTA] CANCEL\r\n");
        OTA_ResetState();              /* 清状态, 丢弃本次升级 */
        OTA_Send1Byte(sn, RSP_ACK);    /* 回复"已取消" */
        BSP_W5500_SocketDisconnect(sn);/* 断开连接 */
        break;

    case CMD_STATUS:
        printf("[OTA] STATUS\r\n");
        {
            OTAStatusResp resp;
            resp.magic = HTONL(OTA_STATUS_MAGIC);   /* 魔术字转网络序 */
            resp.stage = (uint8_t)g_stage;          /* 当前阶段 */
            uint32_t total_blocks = g_total_size / OTA_PACKET_SIZE + 1;   /* 总块数(估算) */
            /* 进度百分比: 已收块数 / 总块数 */
            uint8_t pct = total_blocks ? (uint8_t)(g_expected_block * 100 / total_blocks) : 0;
            resp.progress_pct = pct;
            resp.last_error = 0;
            resp.fw_version = HTONL(g_fw_version);
            BSP_W5500_SocketSend(sn, (const uint8_t*)&resp, sizeof(resp));
        }
        break;

    default:
        printf("[OTA] Unknown cmd: %d\r\n", cmd->cmd);
        OTA_Send1Byte(sn, RSP_NACK);
        break;
    }
}

/* ======================== 数据处理 ======================== */

/*
 * OTA_HandleData — 处理一个 512 字节的数据块
 *
 * 处理顺序很重要, 每一步失败都有对应的错误码回给主机:
 *   擦除致命错误 → 阶段检查 → 越界检查 → 取消检查 → 扇区是否已擦 →
 *   CRC 校验 → 包序号(重复/乱序) → 写入 + 回读验证 → ACK
 */
static void OTA_HandleData(uint8_t sn, const OTAData *pkt)
{
    /* 包头字段从网络序转主机序 */
    uint32_t block_num = NTOHL(pkt->block_num);
    uint16_t data_len  = NTOHS(pkt->data_len);
    uint16_t block_crc = NTOHS(pkt->block_crc);

    /* 1) 擦除过程中出过不可恢复的错误 → 整个 OTA 没法继续 */
    if (g_erase_fatal) {
        OTA_Send1Byte(sn, RSP_ERR_ERASE);
        OTA_ResetState();
        return;
    }

    /* 2) 不在接收阶段(没发 START 就发数据) → 拒绝 */
    if (g_stage != OTA_STAGE_RECEIVING) {
        OTA_Send1Byte(sn, RSP_NACK);
        return;
    }

    /* 3) 越界检查: 该块的地址不能超出本次固件大小(防止恶意/错乱数据写坏别处) */
    if (block_num * OTA_PACKET_SIZE >= g_total_size) {
        OTA_Send1Byte(sn, RSP_ERR_OOB);
        OTA_ResetState();
        return;
    }

    /* 4) 有取消请求 → 直接复位状态放弃 */
    if (g_cancel_flag) {
        OTA_ResetState();
        return;
    }

    /* 5) 该块所在扇区是否已经擦好了? 没擦完 → 回 BUSY, 主机等 200ms 重发 */
    uint32_t w25_addr = STAGING_ADDR + block_num * OTA_PACKET_SIZE;
    if (!OTA_IsSectorErased(w25_addr)) {
        OTA_Send1Byte(sn, RSP_BUSY);
        return;
    }

    /* 6) 异步擦除正在进行中 → 拒绝写入。
     *    【实测结论】擦除窗口内发出的页编程命令会被芯片静默丢弃, 整块回读 0xFF,
     *    重试也无效。这道闸门是唯一实测有效的保护。
     *    START 阶段已做同步全量擦除, 所以传输期间这里通常不会触发。 */
    if (g_erase_in_progress) {
        OTA_Send1Byte(sn, RSP_BUSY);
        return;
    }

    /* 7) CRC16-Modbus 校验: 比对主机给的块 CRC 和本地算出来的 */
    uint16_t calc_crc = CRC16_Modbus(pkt->data, data_len);
    if (calc_crc != block_crc) {
        OTA_Send1Byte(sn, RSP_NACK);   /* 让主机重发这一块 */
        printf("[OTA] Block %lu CRC mismatch\r\n", block_num);
        return;
    }

    /* 8) 三态包序号处理(应对 TCP 重传/乱序) */
    if (block_num < g_expected_block) {
        OTA_Send1Byte(sn, RSP_ACK);   /* 重复包(已经写过了) → 直接 ACK, 不重复写 */
        return;
    }
    if (block_num > g_expected_block) {
        return;                        /* 乱序(超前了) → 静默丢弃, 等主机重传缺失的块 */
    }

    /* 9) 写入 W25Q64 + 回读验证, 最多重试 3 次(与 Bootloader 同款策略) */
    uint8_t verify_buf[OTA_PACKET_SIZE];
    int write_ok = 0;
    for (int attempt = 0; attempt < 3 && !write_ok; attempt++) {
        uint32_t _t0 = osKernelGetTickCount();
        W25Q64_Status wr_st = BSP_W25Q64_Write(w25_addr, (uint8_t*)pkt->data, data_len);
        uint32_t _dt = osKernelGetTickCount() - _t0;
        if (_dt > 50) {
            printf("[OTA] SLOW WRITE: block %lu took %lums\r\n", block_num, _dt);   /* 超过 50ms 记一笔 */
        }
        if (wr_st != W25Q64_OK) {
            printf("[OTA] Write FAILED at block %lu (ret=%d)\r\n", block_num, wr_st);
            OTA_Send1Byte(sn, RSP_ERR_WRITE);
            OTA_ResetState();
            return;
        }

        /* 写完立刻回读, 逐字节比对 —— Flash 写入可能因各种原因失败, 必须验证 */
        W25Q64_Status rd_st = BSP_W25Q64_Read(w25_addr, verify_buf, data_len);
        if (rd_st != W25Q64_OK) {
            printf("[OTA] Read FAILED at block %lu (ret=%d)\r\n", block_num, rd_st);
            OTA_Send1Byte(sn, RSP_ERR_WRITE);
            OTA_ResetState();
            return;
        }

        write_ok = (memcmp(pkt->data, verify_buf, data_len) == 0);   /* 一致才算成功 */
        if (!write_ok) {
            printf("[OTA] Verify retry %d at block %lu\r\n", attempt, block_num);
            osDelay(20);  /* 等芯片把可能正在进行的擦除彻底做完, 再重写 */
        }
    }
    if (!write_ok) {
        /* 三次都验证失败 → 打印第一个不一致的字节, 便于排查是数据问题还是硬件问题 */
        printf("[OTA] Verify FAIL at block %lu addr=0x%08lX erasing=%d\r\n",
               block_num, w25_addr, g_erase_in_progress);
        for (int k = 0; k < 16; k++) {
            if (pkt->data[k] != verify_buf[k]) {
                printf("  [%d] wrote=0x%02X read=0x%02X\r\n",
                       k, pkt->data[k], verify_buf[k]);
                break;  /* 只显示第一个不一致的字节 */
            }
        }
        OTA_Send1Byte(sn, RSP_ERR_WRITE);
        OTA_ResetState();
        return;
    }

    /* 10) 全部成功: 期待块号 +1, 回 ACK, 主机继续发下一块 */
    g_expected_block++;
    OTA_Send1Byte(sn, RSP_ACK);

    /* 每 16 块打印一次进度(约 8KB), 不刷屏 */
    if (block_num % 16 == 0) {
        printf("[OTA] Block %lu/%lu\r\n", block_num,
               (unsigned)(g_total_size / OTA_PACKET_SIZE));
    }
}

/* ======================== 定时器 / 超时 ======================== */

/* 由 TcpServerTask 主循环周期调用(非阻塞): 推进异步擦除 + 检测传输超时 */
void OTA_Tick(void)
{
    OTA_TickAsyncErase();   /* 推进异步擦除状态机(如果有擦除任务在跑) */

    if (g_stage != OTA_STAGE_IDLE) {
        uint32_t now = osKernelGetTickCount();
        if (now - g_last_data_ts > OTA_TIMEOUT_MS) {
            /* 5s 没收到任何数据 → 主机可能掉线了, 中止升级并重建 Socket */
            printf("[OTA] Timeout, aborting\r\n");
            OTA_ResetState();
            BSP_W5500_SocketDisconnect(OTA_SOCKET);
            BSP_W5500_SocketInit(OTA_SOCKET, 1, OTA_PORT);   /* 1 = TCP 模式 */
            BSP_W5500_SocketListen(OTA_SOCKET);              /* 重新监听 506 */
        }
    }
}

/* ======================== 异步擦除 ======================== */

/*
 * OTA_EraseStagingSync — 同步擦除整个暂存区
 *
 * 从 STAGING_ADDR 一直擦到"本次固件末尾所在的扇区边界", 但不超过日志区起点
 * (LOG_AREA_ADDR), 防止把日志区擦掉。
 * 每擦完一个扇区喂一次狗 —— 因为擦除很慢, 连续擦几十个扇区可能超过 IWDG 时间。
 */
static void OTA_EraseStagingSync(void)
{
    /* 向上取整到 4KB 扇区边界(& ~4095 = 清掉低 12 位) */
    uint32_t end = (STAGING_ADDR + g_total_size + 4095) & ~4095UL;
    if (end > LOG_AREA_ADDR)
        end = LOG_AREA_ADDR;   /* 安全上限: 绝不越过日志区 */

    for (uint32_t addr = STAGING_ADDR; addr < end; addr += 4096) {
        W25Q64_Status st = BSP_W25Q64_SectorErase(addr);   /* 擦一个 4KB 扇区 */
        if (st != W25Q64_OK) {
            printf("[OTA] Sync erase FAIL at 0x%06lX (ret=%d)\r\n", addr, st);
            break;             /* 擦除失败就停下(后续写入会因扇区不干净而失败) */
        }
        IWDG_Reload();         /* 每擦一个扇区喂狗 */
    }

    /* 把异步擦除状态也置成"已完成", 这样传输期间的擦除闸门永不触发 */
    g_erase_addr = end;
    g_erase_end = end;
    g_erase_in_progress = 0;
}

/*
 * OTA_TickAsyncErase — 异步擦除状态机(每次 Tick 走一小步, 不阻塞)
 *
 * 因为扇区擦除要几十毫秒, 不能傻等, 所以拆成:
 *   收到命令 → 下一次 Tick 检查芯片是否忙 → 不忙就再发下一个扇区…
 * 这样主循环(要管 4 个 Modbus Socket + OTA)不会被一次长时间的擦除卡住。
 */
static void OTA_TickAsyncErase(void)
{
    if (g_erase_addr == 0 || g_erase_addr >= g_erase_end) return;   /* 没有待擦除区间 */

    if (!g_erase_in_progress) {
        BSP_W25Q64_SendEraseCmd(g_erase_addr);   /* 发出擦除命令(不等待完成) */
        g_erase_in_progress = 1;
        return;
    }

    if (!BSP_W25Q64_IsBusy()) {
        /* 芯片不忙了, 说明上一个扇区擦完了 → 检查擦除是否出错 */
        uint8_t sr2;
        BSP_W25Q64_ReadStatus2(&sr2);
        if (sr2 & 0x20) {
            /* SR2 的 bit5 = P_FAIL(擦除/编程失败) → 致命错误, 终止本次 OTA */
            g_erase_fatal = 1;
            g_erase_addr = 0;
            g_erase_end = 0;
            g_erase_in_progress = 0;
            return;
        }
        g_erase_addr += 4096;        /* 推进到下一个扇区 */
        g_erase_in_progress = 0;
        IWDG_Reload();               /* 每擦完一个扇区喂狗 */
    }
    /* 芯片还 BUSY: 不等待, 等下一次 Tick 再查(这就是"非阻塞擦除") */
}

/*
 * OTA_IsSectorErased — 判断某个地址所在扇区是否已经擦好
 *
 * 原理: 异步擦除是"从 STAGING_ADDR 开始, 由低到高推进"的,
 *   所以只要"扇区基址 < g_erase_addr", 就说明这个扇区已经被擦过了。
 */
static int OTA_IsSectorErased(uint32_t addr)
{
    if (g_erase_addr == 0) return 0;           /* 还没开始擦 → 视为未擦 */
    uint32_t sector_base = addr & ~4095;       /* 清低 12 位得到扇区基址 */
    return (sector_base < g_erase_addr);        /* 基址小于当前擦除进度 → 已擦完 */
}

/* ======================== 状态重置 ======================== */

/* 把 OTA 相关的所有状态清回初始值(每次 START/CANCEL/失败/超时 都要调) */
static void OTA_ResetState(void)
{
    g_stage = OTA_STAGE_IDLE;
    g_total_size = 0;
    g_total_crc = 0;
    g_fw_version = 0;
    g_expected_block = 0;
    g_ota_in_progress = 0;
    g_finish_sent = 0;
    g_cancel_flag = 0;
    g_erase_addr = 0;
    g_erase_end = 0;
    g_erase_in_progress = 0;
    g_erase_fatal = 0;

    /* 清空环形缓冲区(游标归零即可, 内容不再有意义) */
    ota_rx_head = 0;
    ota_rx_tail = 0;
}

/* 查询是否正在升级(日志任务据此暂停写 Flash, 避免和 OTA 抢 Flash 带宽/锁) */
int OTA_IsInProgress(void)
{
    return g_ota_in_progress;
}
