/* ============================================================================
 * gateway.c — 单个网关运行时上下文实现
 *
 * 一个 GatewayCtx 代表"一个被采集的下位机网关"，绑定三样东西:
 *   - ModbusMaster：到该网关的独立 TCP 连接
 *   - timer_fd    ：该网关独立的采集节拍（Linux 定时器文件描述符）
 *   - 统计/报警状态：采集次数、成功失败、上次数据、4 通道告警去抖状态
 * 多个 GatewayCtx 由 poller.c 统一管理，所有 timer_fd 挂到同一个 epoll 上，
 * 实现多网关并发采集、互不阻塞。
 *
 * 每次采集新开一个 SQLite 短连接（Storage_Open → Insert → Close）：
 * 换取"无跨线程共享状态"，简单且安全。秒级采集频率下开销可接受。
 * ============================================================================
 */
#include "gateway.h"
#include "logger.h"
#include "storage.h"
#include "alarm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>       /* close */
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>  /* timerfd_create / timerfd_settime */

/* ---- 初始化网关上下文 ----
 * 【注意调用顺序约定】本函数内部会 memset(ctx)，会把之前设置的字段清零，
 *   所以调用方必须在调用本函数"之后"再设置 ctx->index（见 poller.c 的注释）。 */
int Gateway_Init(GatewayCtx *ctx, GatewayConfig *cfg)
{
    memset(ctx, 0, sizeof(*ctx));     /* 先把整个结构体清零，杜绝随机值 */
    Alarm_Init(&ctx->alarm);          /* 初始化报警检测状态机（4 通道计数器清零） */
    ctx->cfg         = cfg;           /* 持有网关配置指针（不拥有，指向 poller 的 cfg 数组） */
    ctx->timer_fd    = -1;            /* -1 = 非法 fd，表示"尚未创建"（不能在 0 处误用） */
    ctx->running     = 0;
    ctx->poll_interval = cfg->poll_interval_sec;  /* 采集周期(秒) */

    if (ctx->poll_interval < 1) ctx->poll_interval = 5;  /* 周期下限保护：配置写 0 会变忙等 */

    /* 初始化 Modbus 连接 (TCP 连到下位机网关) */
    memset(&ctx->mb, 0, sizeof(ctx->mb));
    if (Modbus_Connect(&ctx->mb, cfg->host, cfg->port) != 0) {
        /* 【容错设计】首次连不上不让程序启动失败。
         * 因为网关可能还没上电，但我们要先把服务跑起来，等它上线后自动连上。
         * 这也是"采集类服务"该有的健壮性：不能因为一个设备没开就不干活。 */
        LOG_WARN("Gateway '%s' initial connect failed, will retry in background",
                 cfg->name);
        /* 不返回错误，让重试机制处理：首次连不上不阻塞整个程序启动 */
    }

    LOG_INFO("Gateway '%s' initialized (%s:%u, interval=%us)",
             cfg->name, cfg->host, cfg->port, ctx->poll_interval);
    return 0;
}

/* ---- 创建并启动该网关的 timerfd ----
 * timerfd 是 Linux 提供的"定时器文件描述符"：定时到期时 epoll 会把它标记为可读，
 * 从而让一个 epoll 主循环同时管理多个不同周期的定时任务。
 * 【对比】传统做法是开 N 个线程各睡 N 秒；这里一个线程 + N 个 timerfd 就搞定，
 *   线程开销、上下文切换全部省掉——这是 Linux 高性能服务器常用的模式。 */
int Gateway_StartTimer(GatewayCtx *ctx)
{
    if (ctx->timer_fd >= 0) return 0;   /* 已创建则幂等返回 */

    /* 创建单调时钟的定时器（CLOCK_MONOTONIC 不受系统改时间影响），非阻塞模式 */
    /* 【为什么用 CLOCK_MONOTONIC？】用 CLOCK_REALTIME 的话，运维改一下系统时间
     * 定时器就可能立刻触发或长时间不触发。单调时钟只受开机时长影响，最稳。 */
    ctx->timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (ctx->timer_fd == -1) {
        LOG_ERROR("Gateway '%s': timerfd_create failed: %s",
                  ctx->cfg->name, strerror(errno));
        return -1;
    }

    /* 设置定时参数：
     *   it_interval = 周期 (每隔 poll_interval 秒触发一次)
     *   it_value    = 首次触发时刻 (这里故意错开 1~5 秒)
     * 错开首触发时间是关键：若多个网关 interval 相同且都从 0 开始，
     * 它们会在同一瞬间一起采集，瞬时拖垮 CPU/网络/数据库。
     * 这种"把负载在时间上摊开"的做法叫"错峰/抖动(jitter)"，是工程上的常见技巧。 */
    struct itimerspec ts = {
        .it_interval = {
            .tv_sec = ctx->poll_interval,   /* tv_nsec 未写 = 0 */
        },
        .it_value = {
            .tv_sec = 1 + ctx->index % 5,  /* 按索引错开 1~5 秒 */
        },
    };

    if (timerfd_settime(ctx->timer_fd, 0, &ts, NULL) == -1) {
        LOG_ERROR("Gateway '%s': timerfd_settime failed: %s",
                  ctx->cfg->name, strerror(errno));
        close(ctx->timer_fd);    /* 失败要关掉刚创建的 fd，否则泄漏 */
        ctx->timer_fd = -1;
        return -1;
    }

    ctx->running = 1;
    return 0;
}

/* ---- 执行一次完整采集 ----
 * timerfd 到期 → epoll 主循环调用本函数 → 完成 "读 Modbus → 存 DB → 查报警" 全流程。
 * 【重要】本函数会"阻塞"（最长约 1s，取决于 Modbus 超时），
 *   但它只阻塞当前这一个网关的事件处理，不影响其它网关——
 *   因为其它网关的 timerfd 事件还在内核队列里排队等主循环处理。 */
int Gateway_PollOnce(GatewayCtx *ctx, const char *db_path)
{
    uint16_t raw[4];   /* 原始寄存器值。读 2 个，留 4 个余量，防配置写大时越界 */
    int ret;

    /* 1. 连接不可用则先重连（断点续采的前置步骤） */
    if (!Modbus_IsConnected(&ctx->mb)) {
        LOG_INFO("Gateway '%s': disconnected, attempting reconnect...",
                 ctx->cfg->name);
        if (Modbus_Reconnect(&ctx->mb) != 0) {
            LOG_WARN("Gateway '%s': reconnect failed, will retry next cycle",
                     ctx->cfg->name);
            ctx->fail_count++;
            return -1;   /* 本次放弃，等下个周期再试（不在本函数里死等，避免拖住 epoll 循环） */
        }
    }

    /* 2. 发送 Modbus 读保持寄存器请求（读传感器温度/湿度） */
    ret = Modbus_ReadHoldingRegs(&ctx->mb,
                                  ctx->cfg->slave_id,
                                  ctx->cfg->reg_start,
                                  ctx->cfg->reg_count,
                                  raw);
    if (ret != 0) {
        LOG_WARN("Gateway '%s': poll failed (ret=%d), err_count=%d",
                 ctx->cfg->name, ret, ctx->mb.err_count);
        ctx->fail_count++;

        /* 连续错误过多 → 强制断开连接，避免握着过期的 socket
         * 【为什么？】网线被拔、设备重启后，旧的 fd 可能"看起来还活着"，
         * 但每次 read 都超时；一直复用只会白等。主动断开，下轮走完整重连更干脆。 */
        if (ctx->mb.err_count >= 3) {
            LOG_WARN("Gateway '%s': too many errors, forcing disconnect",
                     ctx->cfg->name);
            Modbus_Disconnect(&ctx->mb);
        }

        /* 机会性重连：立即尝试重建，避免下一周期因旧 fd 重复失败 */
        if (!Modbus_IsConnected(&ctx->mb)) {
            Modbus_Reconnect(&ctx->mb);
        }
        return -1;
    }

    /* 3. 解析数据：寄存器存的是 int16 放大 10 倍的值（235 = 23.5℃）
     * 【为什么先强转 int16_t】
     *   温度可能是负数（-5.3℃ → 寄存器 0xFFCB）。
     *   若按 uint16_t 解释会得到 65483，除以 10 变成 6548.3℃，明显错误。
     *   强转成有符号的 int16_t 才能得到 -53，再 /10 得 -5.3℃。 */
    int16_t temp_raw = (int16_t)raw[0];
    int16_t hum_raw  = (int16_t)raw[1];
    double  temp_c   = temp_raw / 10.0;   /* 定标换算：放大 10 倍存储 → 除以 10 还原 */
    double  hum_pct  = hum_raw / 10.0;

    ctx->last_temp = temp_raw;   /* 缓存原始值（放大 10 倍的整数），供打印/统计 */
    ctx->last_hum  = hum_raw;
    ctx->last_ts   = time(NULL);   /* 记录本次采集时间戳 */

    /* 4. 写入数据库（每个采集周期独立开一个 DB 连接并关闭，
     *    避免多个网关线程共享同一连接产生竞争，简单且安全） */
    StorageDB sdb;
    if (Storage_Open(&sdb, db_path) == 0) {
        SensorRecord rec;
        rec.ts           = ctx->last_ts;
        rec.temperature  = temp_c;
        rec.humidity     = hum_pct;
        rec.gateway_id   = ctx->index;            /* 区分来自哪个网关（多网关数据分离的关键） */
        snprintf(rec.gateway_name, sizeof(rec.gateway_name), "%s", ctx->cfg->name);
        Storage_Insert(&sdb, &rec);
        Storage_Close(&sdb);   /* 立即关闭，不留长连接 */
    }

    /* 5. 报警检查（超温/超湿等触发阈值报警，可写 DB/日志）
     *    【顺序关键】必须在"数据成功入库"之后做，这样告警与数据的时间线才一致 */
    Alarm_Check(&ctx->alarm, ctx->cfg, ctx->index,
                temp_c, hum_pct, db_path);

    /* 6. 日志输出 */
    /* %lu 对应 uint64_t 会有平台差异，这里传的是 success_count+1（uint64_t），
     * 在 64 位 Linux 上 unsigned long 也是 64 位，所以能对上。 */
    LOG_INFO("[%s] POLL #%lu OK: T=%.1f℃ H=%.1f%% (raw=0x%04X 0x%04X)",
             ctx->cfg->name,
             ctx->success_count + 1, temp_c, hum_pct, raw[0], raw[1]);

    ctx->success_count++;
    return 0;
}

/* ---- 获取 timer_fd ----
 * 供 poller 把它加入 epoll 监听。封装成函数而不是直接访问字段，
 * 是为了将来若改内部实现（比如换成 signalfd），外部代码不用改。 */
int Gateway_GetTimerFd(GatewayCtx *ctx)
{
    return ctx->timer_fd;
}

/* ---- 停止并清理 ----
 * 关闭 timerfd、断开 Modbus 连接，并打印累计统计。 */
void Gateway_Stop(GatewayCtx *ctx)
{
    ctx->running = 0;

    if (ctx->timer_fd >= 0) {
        close(ctx->timer_fd);   /* 关闭定时器 fd（内核会自动把它从 epoll 上摘掉） */
        ctx->timer_fd = -1;
    }

    Modbus_Disconnect(&ctx->mb);

    /* 退出时打一份"总账"：采集了多少次、成功多少、失败多少，便于复盘 */
    LOG_INFO("Gateway '%s' stopped (polls=%lu ok=%lu fail=%lu)",
             ctx->cfg->name, ctx->poll_count,
             ctx->success_count, ctx->fail_count);
}
