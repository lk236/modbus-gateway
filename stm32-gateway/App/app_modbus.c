/*
 * app_modbus.c — Modbus TCP 多 Socket 服务器任务(TcpServerTask)
 *
 * W5500 上开 4 个 TCP Socket(502~505)提供 Modbus-TCP 服务: 解析 MBAP →
 * 转 RTU 帧发到 RS-485 → 从机响应转回 TCP。Socket 4 作 OTA 端口(506)。
 *
 * 采用非阻塞状态机(SOCK_PHASE_IDLE / SOCK_PHASE_TX_WAIT)而非"一发一收":
 * 4 个 Socket 共享一条 RS-485 总线, 阻塞写法会让等响应的 Socket 卡住其余 3 个。
 *
 * 三层保护:
 *   g_buf_mutex    保护共享缓冲 g_rx_buf / g_resp_buf(完整事务期间持有);
 *   g_rtu_busy     快速门闩, 同一时刻只有一个 Socket 跑 RTU 事务;
 *   g_modbus_mutex (modbus_rtu.c) 保护 RS-485 物理总线, 兼防与健康检查任务撞车。
 */
#include "app_modbus.h"    /* 本任务对外接口 APP_Modbus_Init() */
#include "bsp_w5500.h"     /* W5500 底层: Socket 收发、状态、寄存器读写 */
#include "modbus_tcp.h"    /* MBAP 解析 / TCP-RTU 转换 */
#include "modbus_rtu.h"    /* 非阻塞 RTU 事务 Modbus_AsyncXxx */
#include "app_task.h"      /* g_gateway_config 全局配置(IP / 端口) */
#include "ota_receive.h"   /* OTA_OnData / OTA_Tick / OTA_Abort */
#include <string.h>
#include <stdio.h>         /* printf 调试打印 */

#define TCP_SOCKET_COUNT    4        /* Modbus 服务用 Socket 0~3 */
#define OTA_SOCKET          4        /* Socket 4 专门给 OTA(端口 506) */
#define TCP_IDLE_TIMEOUT_MS 30000    /* 客户端 30 秒没动静 → 主动断开 */
#define TCP_SEND_FAIL_MAX   3        /* 连续发送失败 3 次 → 重连 Socket */

/* Socket 事务阶段: 空闲 / 已发请求等响应 */
typedef enum { SOCK_PHASE_IDLE = 0, SOCK_PHASE_TX_WAIT = 1 } SockPhase;

/* 每个 TCP Socket 的上下文(状态 + 统计) */
typedef struct {
    uint8_t  sn;                /* Socket 编号 0~7 */
    uint16_t port;              /* 监听端口 */
    uint32_t last_activity;     /* 最近一次活动 tick(用于空闲超时判断) */
    uint32_t rx_count;          /* 累计收到的请求数(统计用) */
    uint32_t tx_count;          /* 累计成功应答数(统计用) */
    uint32_t tx_fail;           /* 累计发送失败数(统计用) */
    uint8_t  was_connected;     /* 上一轮是否处于已连接(用于检测"刚连上") */
    /* ---- RTU 非阻塞事务状态 ---- */
    uint8_t  phase;             /* 当前阶段: SOCK_PHASE_* */
    ModbusAsyncTx async;        /* 非阻塞 RTU 事务句柄 */
    MBAP_Header mbap;           /* 当前事务对应的 MBAP 头(应答时要用 trans_id) */
    uint8_t  func_code;         /* 当前事务的功能码(网关构造异常应答时要用) */
} TcpSocketCtx;

static osThreadId_t g_tcp_task_handle;              /* 任务句柄 */
static TcpSocketCtx g_sockets[TCP_SOCKET_COUNT];    /* 4 个 Socket 的上下文数组 */

/* 共享收发缓冲(4 个 Socket 共用同一组, 省 ~1.5KB RAM)。
 * 完整事务期间一直持有 g_buf_mutex, 事务结束后才轮到下一个 Socket。
 * 注意: Modbus_AsyncBegin 记录的 tx 指针指向 g_resp_buf, 锁必须持有到帧收完,
 * 否则别的 Socket 会把这帧覆盖掉。 */
static uint8_t g_rx_buf[MODBUS_TCP_FRAME_MAX];      /* 收 TCP 请求用 */
static uint8_t g_resp_buf[MODBUS_TCP_FRAME_MAX];    /* 转 RTU 请求 / 组 TCP 应答共用 */

/* 共享缓冲互斥锁: 锁住"收 TCP → 发 RTU → 等响应 → 回 TCP"这整个完整事务 */
static osMutexId_t g_buf_mutex;
/* 有 Socket 正在占用 RS-485 总线(快速门闩, 防止两个 Socket 同时发起事务) */
static volatile uint8_t g_rtu_busy;

/* 客户端连上时的处理: 清统计、打日志、记一条 Flash 日志 */
static void Socket_OnConnected(TcpSocketCtx *ctx)
{
    ctx->was_connected = 1;                        /* 标记已连接 */
    ctx->last_activity = osKernelGetTickCount();   /* 刷新活动时间 */
    ctx->rx_count = 0;                             /* 本次连接统计清零 */
    ctx->tx_count = 0;
    ctx->tx_fail = 0;
    printf("[TCP:S%d] Client connected on port %d\r\n", ctx->sn, ctx->port);
    Log_Push(W25Q64_LOG_TCP_CONNECT, ctx->port);   /* 往 W25Q64 写一条"客户端接入"日志 */
}

/* 客户端断开时的处理: 打日志、记 Flash 日志 */
static void Socket_OnDisconnected(TcpSocketCtx *ctx)
{
    printf("[TCP:S%d] Client disconnected\r\n", ctx->sn);
    ctx->was_connected = 0;                        /* 标记未连接 */
    Log_Push(W25Q64_LOG_TCP_DISCONNECT, ctx->sn);  /* 记一条"断开"日志 */
}

/* 重置 Socket: 关闭 → 等 100ms → 重新初始化为 TCP → 重新监听。
 * 这 100ms 是等 W5500 硬件回到 INIT 态, 立刻重开可能失败。 */
static void Socket_Reinit(TcpSocketCtx *ctx)
{
    BSP_W5500_SocketClose(ctx->sn);                                /* 关闭当前 Socket */
    osDelay(100);                                                  /* 等硬件回到 INIT */
    BSP_W5500_SocketInit(ctx->sn, W5500_Sn_MR_TCP, ctx->port);     /* 重新初始化 TCP Socket */
    BSP_W5500_SocketListen(ctx->sn);                               /* 重新进入监听 */
    ctx->was_connected = 0;                                        /* 状态复位 */
    printf("[TCP:S%d] Socket reinit, listen on port %d\r\n", ctx->sn, ctx->port);
}

/* 发送 TCP 应答并统计。
 * MODBUS_OK / EXCEPTION → RTU 响应原样转成 TCP 回传;
 * 其他(超时 / CRC 失败) → 网关自造异常应答 [MBAP][fc|0x80][0x0B]。
 * 连续失败 TCP_SEND_FAIL_MAX(3) 次就断开重连, 防 Socket 假死。 */
static void Socket_SendTcpReply(TcpSocketCtx *ctx, Modbus_Status st,
                                const uint8_t *rtu_rx, uint16_t rtu_rx_len)
{
    uint16_t resp_len = 0;                                     /* 待发送字节数 */
    if (st == MODBUS_OK || st == MODBUS_ERROR_EXCEPTION) {
        /* 有有效 RTU 响应(正常或从机异常帧) → 转成 TCP 帧 */
        resp_len = ModbusTCP_RTUtoTCP(rtu_rx, rtu_rx_len, g_resp_buf,
                                      ctx->mbap.transaction_id, ctx->mbap.unit_id);
    } else {
        /* 通信失败 → 网关异常响应: [MBAP][fc|0x80][0x0B] */
        resp_len = ModbusTCP_BuildMBAP(g_resp_buf, ctx->mbap.transaction_id, 3,
                                       ctx->mbap.unit_id);
        g_resp_buf[resp_len]     = ctx->func_code | 0x80;      /* 异常标志位 */
        g_resp_buf[resp_len + 1] = 0x0B;                       /* 异常码 0x0B */
        resp_len += 2;                                         /* 这两字节也算进长度 */
    }

    if (resp_len > 0) {
        /* 通过 W5500 发出去; 返回 0 表示成功(注意: 这里 0 = OK) */
        int8_t send_result = BSP_W5500_SocketSend(ctx->sn, g_resp_buf, resp_len);
        if (send_result == 0) {
            ctx->tx_count++;                                   /* 成功计数 */
        } else {
            ctx->tx_fail++;                                    /* 失败计数 */
            printf("[TCP:S%d] Send FAIL, result=%d [fail=%lu]\r\n",
                   ctx->sn, send_result, ctx->tx_fail);
            if (ctx->tx_fail >= TCP_SEND_FAIL_MAX) {
                /* 连续失败太多次 → 判定 Socket 异常, 断开重连 */
                printf("[TCP:S%d] Too many send failures, reinit\r\n", ctx->sn);
                Socket_OnDisconnected(ctx);
                Socket_Reinit(ctx);
            }
        }
    }
}

/* 结束事务: 释放共享缓冲锁 + 清 RTU 忙标志, 回到空闲阶段 */
static void Socket_EndTransaction(TcpSocketCtx *ctx)
{
    ctx->phase = SOCK_PHASE_IDLE;      /* 回到空闲 */
    g_rtu_busy = 0;                    /* 松开总线门闩, 允许其他 Socket 发起 */
    if (g_buf_mutex != NULL) {
        osMutexRelease(g_buf_mutex);   /* 释放共享缓冲(此刻起别的 Socket 可复用缓冲) */
    }
}

/* 尝试开始新事务(非阻塞): 收 TCP 帧 → 转 RTU → 发起异步发送。
 * 任一环节失败(帧非法 / 总线忙)就丢弃本帧, 让客户端靠超时重试。
 * 成功则 phase 切到 TX_WAIT, 且不释放 buf 锁, 交给 EndTransaction 释放。 */
static void Socket_TryStartTransaction(TcpSocketCtx *ctx)
{
    if (g_rtu_busy) return;                        /* 已有别的 Socket 在跑事务 → 本轮跳过 */

    uint16_t rx_len = BSP_W5500_SocketRxAvail(ctx->sn);   /* 看这个 Socket 收到多少字节 */
    if (rx_len == 0) return;                       /* 没数据 → 本轮无事可做 */

    /* 完整事务期间持有共享缓冲锁(防止两个 Socket 同时改 g_rx_buf/g_resp_buf) */
    if (g_buf_mutex != NULL) {
        if (osMutexAcquire(g_buf_mutex, 0) != osOK) return;   /* 非阻塞拿锁, 拿不到就跳过 */
    }

    ctx->last_activity = osKernelGetTickCount();   /* 有活动, 刷新空闲计时 */
    uint16_t recv_len = BSP_W5500_SocketRecv(ctx->sn, g_rx_buf,
                           rx_len > MODBUS_TCP_FRAME_MAX ? MODBUS_TCP_FRAME_MAX : rx_len);
                           /* 读进 g_rx_buf, 长度取 min(可读量, 缓冲上限), 防溢出 */
    ctx->rx_count++;                               /* 收到一帧请求, 计数 */

    /* ---- 合法性校验, 任何一项不过就 discard 丢帧 ---- */
    if (recv_len < MBAP_HEADER_LEN) goto discard;                              /* 7 字节 MBAP 都不够 */
    if (!ModbusTCP_ParseMBAP(g_rx_buf, recv_len, &ctx->mbap)) goto discard;    /* 解析 MBAP 失败 */
    if (ctx->mbap.protocol_id != 0) goto discard;                              /* 协议 ID 必须为 0 */
    if ((recv_len - MBAP_HEADER_LEN) < 1 || (recv_len - MBAP_HEADER_LEN) > 253) goto discard;  /* PDU 长度非法 */

    ctx->func_code = g_rx_buf[MBAP_HEADER_LEN];    /* 记下功能码(后面造异常应答要用) */

    /* TCP → RTU: 转换结果暂存在共享的 g_resp_buf 里(事务期间由 buf 锁保护) */
    uint16_t rtu_len = ModbusTCP_TCPtoRTU(g_rx_buf, recv_len, g_resp_buf);
    if (rtu_len == 0) goto discard;                /* 转换失败(PDU 非法) */

    /* 发起非阻塞 RTU 事务: 抢到总线并发出请求 → 进入等待阶段 */
    if (Modbus_AsyncBegin(&ctx->async, g_resp_buf, rtu_len) == MODBUS_OK) {
        ctx->phase = SOCK_PHASE_TX_WAIT;           /* 切到"等响应"阶段 */
        g_rtu_busy = 1;                            /* 占住总线门闩 */
        return;                                    /* buf 锁不在这里释放, 留到事务结束 */
    }
    /* 走到这里说明: 总线被健康检查任务占用(MODBUS_BUSY) 或 帧 CRC 错 → 丢弃本帧 */
discard:
    if (g_buf_mutex != NULL) {
        osMutexRelease(g_buf_mutex);               /* 丢帧路径必须还锁 */
    }
}

/* 轮询在途 RTU 事务: 帧到了 → 回 TCP 应答; 超时(内部会重试) → 最终失败也回异常应答 */
static void Socket_PollTransaction(TcpSocketCtx *ctx)
{
    uint8_t rtu_rx[MODBUS_RTU_FRAME_MAX];          /* 栈缓冲, 接从机响应帧(256 字节) */
    uint16_t rtu_rx_len = 0;

    Modbus_Status st = Modbus_AsyncPoll(&ctx->async, rtu_rx, &rtu_rx_len);
    if (st == MODBUS_BUSY) return;                 /* 还在等响应, 直接返回照顾别的 Socket */

    /* 事务结束(OK / EXCEPTION / TIMEOUT / CRC) → 构造应答并释放事务 */
    Socket_SendTcpReply(ctx, st, rtu_rx, rtu_rx_len);
    Socket_EndTransaction(ctx);
}

/* 检查是否该断开: 收到"对端断开"中断, 或空闲超时 */
static void Socket_CheckDisconnect(TcpSocketCtx *ctx)
{
    uint8_t sn = ctx->sn;
    uint8_t block = W5500_SOCK_REG(sn);            /* Socket 寄存器块基址 */

    /* 情况 1: W5500 中断寄存器报了 DISCON(对端主动断开) */
    uint8_t ir = BSP_W5500_ReadReg(block, W5500_Sn_IR);
    if (ir & W5500_Sn_IR_DISCON) {
        BSP_W5500_WriteReg(block, W5500_Sn_IR, W5500_Sn_IR_DISCON);   /* 写 1 清标志 */
        if (ctx->phase != SOCK_PHASE_IDLE) {
            Modbus_AsyncAbort(&ctx->async);        /* 事务在途 → 中止, 释放 RTU 总线 */
            Socket_EndTransaction(ctx);
        }
        Socket_OnDisconnected(ctx);
        return;
    }

    /* 情况 2: 连接着但 30 秒无活动 → 主动踢掉(防半开连接占坑) */
    if (ctx->was_connected &&
        (osKernelGetTickCount() - ctx->last_activity) > TCP_IDLE_TIMEOUT_MS) {
        if (ctx->phase != SOCK_PHASE_IDLE) {
            Modbus_AsyncAbort(&ctx->async);
            Socket_EndTransaction(ctx);
        }
        BSP_W5500_SocketDisconnect(sn);
        Socket_OnDisconnected(ctx);
    }
}

/* 单个 Socket 状态机: 按 W5500 上报的 Sn_SR 状态分派处理 */
static void SocketStateMachine(TcpSocketCtx *ctx)
{
    uint8_t sn = ctx->sn;
    uint8_t status = BSP_W5500_SocketStatus(sn);   /* 读 Socket 状态寄存器 */

    switch (status) {
        case W5500_Sn_SR_ESTABLISHED:              /* 已建立连接(正常收发态) */
            if (!ctx->was_connected) {
                Socket_OnConnected(ctx);           /* 新连上 → 记录 + 打日志 */
            }
            if (ctx->phase == SOCK_PHASE_IDLE) {
                Socket_TryStartTransaction(ctx);   /* 空闲 → 试收请求起事务(总线忙则跳过) */
            } else {
                Socket_PollTransaction(ctx);       /* 事务在途 → 轮询 RTU 响应 */
            }
            Socket_CheckDisconnect(ctx);           /* 顺便看看要不要断开 */
            break;

        case W5500_Sn_SR_CLOSE_WAIT:               /* 对端发了 FIN, 等本端关闭 */
            if (ctx->phase != SOCK_PHASE_IDLE) {
                Modbus_AsyncAbort(&ctx->async);    /* 有事务在途先中止 */
                Socket_EndTransaction(ctx);
            }
            BSP_W5500_SocketDisconnect(sn);        /* 关闭本端 */
            osDelay(10);                           /* 略等硬件状态迁移 */
            if (ctx->was_connected) {
                Socket_OnDisconnected(ctx);
            }
            break;

        case W5500_Sn_SR_CLOSED:                   /* Socket 已关闭 */
            if (ctx->phase != SOCK_PHASE_IDLE) {
                Modbus_AsyncAbort(&ctx->async);
                Socket_EndTransaction(ctx);
            }
            Socket_Reinit(ctx);
            break;

        case W5500_Sn_SR_INIT:                     /* 初始化完成, 还没监听 */
            BSP_W5500_SocketListen(sn);
            break;

        case W5500_Sn_SR_LISTEN:                   /* 以下都是中间过渡态, 等其自行迁移 */
        case W5500_Sn_SR_SYNSENT:
        case W5500_Sn_SR_SYNRECV:
        case W5500_Sn_SR_FIN_WAIT:
        case W5500_Sn_SR_CLOSING:
        case W5500_Sn_SR_TIME_WAIT:
        case W5500_Sn_SR_LAST_ACK:
            break;

        default:                                   /* 未知状态: 兜底重置 */
            printf("[TCP:S%d] Unknown status=0x%02X, reinit\r\n", sn, status);
            if (ctx->phase != SOCK_PHASE_IDLE) {
                Modbus_AsyncAbort(&ctx->async);
                Socket_EndTransaction(ctx);
            }
            Socket_OnDisconnected(ctx);
            Socket_Reinit(ctx);
            break;
    }
}

/* TcpServerTask 主函数: 初始化 W5500 → 建 4 个监听 → 死循环跑状态机 */
static void APP_TCP_Task(void *argument)
{
    printf("[TCP] Multi-socket task started, %d sockets\r\n", TCP_SOCKET_COUNT);

    /* 1. 初始化 W5500(配 IP / 子网掩码 / 网关); SPI 复位等已由 BSP 层完成 */
    BSP_W5500_Init(g_gateway_config.gateway_ip,
                   g_gateway_config.subnet_mask,
                   g_gateway_config.default_gw);

    /* 2. 初始化 4 个 Socket 上下文; 端口 = 基础端口 + 序号(502 → 502~505) */
    for (uint8_t i = 0; i < TCP_SOCKET_COUNT; i++) {
        g_sockets[i].sn            = i;
        g_sockets[i].port          = g_gateway_config.tcp_port + i;
        g_sockets[i].last_activity = 0;
        g_sockets[i].rx_count      = 0;
        g_sockets[i].tx_count      = 0;
        g_sockets[i].tx_fail       = 0;
        g_sockets[i].was_connected = 0;
        g_sockets[i].phase         = SOCK_PHASE_IDLE;
        Modbus_AsyncInit(&g_sockets[i].async);
    }

    /* 3. Socket 0~3 收发缓冲各 2KB(W5500 共 16KB TX + 16KB RX, 单位是 KB) */
    for (uint8_t i = 0; i < TCP_SOCKET_COUNT; i++) {
        BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_TXBUF_SIZE, 2);
        BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_RXBUF_SIZE, 2);
    }

    /* 4. Socket 4(OTA) 保留 2KB, 其余 5~7 不用的分配 0 */
    for (uint8_t i = TCP_SOCKET_COUNT; i < 8; i++) {
        if (i == OTA_SOCKET) {
            BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_TXBUF_SIZE, 2);
            BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_RXBUF_SIZE, 2);
        } else {
            BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_TXBUF_SIZE, 0);
            BSP_W5500_WriteReg(W5500_SOCK_REG(i), W5500_Sn_RXBUF_SIZE, 0);
        }
    }

    /* 5. 4 个 Socket 初始化为 TCP 服务端并开始监听 */
    for (uint8_t i = 0; i < TCP_SOCKET_COUNT; i++) {
        BSP_W5500_SocketInit(i, W5500_Sn_MR_TCP, g_sockets[i].port);
        BSP_W5500_SocketListen(i);
        printf("[TCP:S%d] Listen on port %d\r\n", i, g_sockets[i].port);
    }

    uint32_t print_tick = 0;                             /* 上次打印统计的时刻 */
    for (;;) {
        Wdg_Heartbeat(WDG_TASK_ID_TCP);                  /* 喂看门狗: 本任务活着 */

        /* ---- 6a. 轮流跑 4 个 Modbus Socket 状态机 ---- */
        for (uint8_t i = 0; i < TCP_SOCKET_COUNT; i++) {
            SocketStateMachine(&g_sockets[i]);
            Wdg_Heartbeat(WDG_TASK_ID_TCP);              /* 本轮可能耗时久, 分摊喂狗节奏 */
        }

        /* ---- 6b. Socket 4: OTA 端口 506 状态机 ---- */
        {
            uint8_t status = BSP_W5500_SocketStatus(OTA_SOCKET);
            switch (status) {
            case W5500_Sn_SR_CLOSED:
            case W5500_Sn_SR_INIT:
                /* 没连或刚复位 → 重新初始化并监听 506 */
                BSP_W5500_SocketInit(OTA_SOCKET, W5500_Sn_MR_TCP, OTA_PORT);
                BSP_W5500_SocketListen(OTA_SOCKET);
                break;
            case W5500_Sn_SR_ESTABLISHED:
                {
                    /* 有连接 → 有数据就读出来交给 OTA 状态机 */
                    uint16_t len = BSP_W5500_SocketRxAvail(OTA_SOCKET);
                    if (len > 0) {
                        uint8_t buf[1024];               /* 单次最多读 1KB(栈上) */
                        uint16_t rlen = BSP_W5500_SocketRecv(OTA_SOCKET, buf, len > 1024 ? 1024 : len);
                        printf("[DBG_OTA] W5500 recv len=%u", rlen);
                        if (rlen >= 4) printf(" magic=0x%02X%02X%02X%02X", buf[0], buf[1], buf[2], buf[3]);
                        printf("\r\n");                  /* 打印前 4 字节(OTA 包魔数), 便于抓包排查 */
                        OTA_OnData(OTA_SOCKET, buf, rlen);
                    }
                }
                break;
            case W5500_Sn_SR_CLOSE_WAIT:
                OTA_Abort();                             /* 对端半关 → 中止 OTA 再关 Socket */
                BSP_W5500_SocketDisconnect(OTA_SOCKET);
                break;
            default:
                break;
            }
        }

        OTA_Tick();                                      /* OTA 超时/状态推进的周期心跳 */

        /* ---- 6c. 每 5 秒打印一次各 Socket 状态 ---- */
        if ((osKernelGetTickCount() - print_tick) > 5000) {
            for (uint8_t i = 0; i < TCP_SOCKET_COUNT; i++) {
                TcpSocketCtx *ctx = &g_sockets[i];
                uint8_t status = BSP_W5500_SocketStatus(ctx->sn);
                uint16_t rx_avail = BSP_W5500_SocketRxAvail(ctx->sn);
                /* %d=sn, %02X=status, %d=rx_avail, %u=was_connected, %lu=rx_count, %lu=tx_count */
                printf("[TCP:S%d] SR=0x%02X conn=%d rx_avail=%u rx=%lu tx=%lu\r\n",
                       ctx->sn, status, rx_avail, ctx->was_connected,
                       ctx->rx_count, ctx->tx_count);
            }
            print_tick = osKernelGetTickCount();
        }

        osDelay(1);                                      /* 让出 1 tick, 给其他任务运行机会 */
    }
}

/* 初始化: 建互斥锁 + 创建 TcpServerTask(在 freertos.c 的 StartDefaultTask 中被调用) */
void APP_Modbus_Init(void)
{
    printf("[APP] Modbus TCP multi-socket init...\r\n");

    /* 共享收发缓冲互斥锁(完整事务期间持有) */
    osMutexAttr_t mutex_attr = { NULL, osMutexPrioInherit, NULL, 0 };
    g_buf_mutex = osMutexNew(&mutex_attr);
    g_rtu_busy = 0;

    const osThreadAttr_t attr = {
        .name       = "TcpServerTask",
        /* 栈 4KB: OTA 热路径占 buf[1024] + pkt(524B) + verify_buf(512B) + 调用帧
         * ≈ 2.6KB。原 3KB 紧贴上限, 溢栈会静默破坏相邻堆内存。 */
        .stack_size = 1024 * 4,
        .priority   = osPriorityNormal,
    };
    g_tcp_task_handle = osThreadNew(APP_TCP_Task, NULL, &attr);

    if (g_tcp_task_handle == NULL) {
        printf("[APP] ERROR: TCP task create failed!\r\n");      /* 堆不足等导致创建失败 */
    } else {
        printf("[APP] TCP task created OK\r\n");
    }
}
