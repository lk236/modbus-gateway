/* ============================================================================
 * poller.c — 多网关并发轮询引擎实现 v2
 *
 * 核心设计: 1 个 epoll + N 个 timerfd = N 个网关并发轮询
 *
 * epoll 管理的 fd:
 *   - N 个 timerfd: 每个网关一个，独立采集周期
 *   - 1 个 eventfd: 用于从其他线程通知主循环退出
 *
 * 一个网关断线/超时只影响它自己的 timerfd 事件，其余网关按各自周期继续采集。
 * 相较"每网关一个线程"：线程栈(默认 8MB)与上下文切换开销随网关数线性增长，
 * 且多线程写同一个 SQLite 必须加锁；epoll + timerfd 把"定时"变成"IO 就绪事件"，
 * 单线程天然串行处理，无锁竞争且能精确控制每个网关的节拍。
 * ============================================================================
 */
#include "poller.h"
#include "gateway.h"
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>  /* timerfd_* */
#include <sys/epoll.h>    /* epoll_create1 / epoll_ctl / epoll_wait */
#include <sys/eventfd.h>  /* eventfd：轻量级"线程间通知"机制 */

/* ---- 初始化 ----
 * 【注意】Poller_Init 只做"准备"，不创建 epoll/timerfd（那是 Poller_Start 的活）。
 *   分开的好处：init 失败容易回滚，start 才真正开始对外产出。 */
int Poller_Init(Poller *plr, AppConfig *cfg)
{
    memset(plr, 0, sizeof(*plr));       /* 全清零 */
    plr->cfg          = cfg;            /* 只保存指针，不拷贝配置（cfg 生命周期由 main 保证） */
    plr->gateway_count = cfg->gateway_count;
    plr->epoll_fd     = -1;             /* -1 = 还没创建 */
    plr->stop_fd      = -1;
    plr->running      = 0;

    /* 初始化每个网关 */
    for (int i = 0; i < plr->gateway_count; i++) {
        if (Gateway_Init(&plr->gateways[i], &cfg->gateways[i]) != 0) {
            LOG_WARN("Gateway %d init failed (will retry in background)", i + 1);
            /* 单个网关初始化失败不影响其它网关，继续循环 */
        }
        /* 【易错点，注释也在原码里】Gateway_Init 内部 memset 会清零整个 ctx，
         * 所以 index 必须在它之后赋值，否则会被清成 0。 */
        plr->gateways[i].index = i;
    }

    /* 打开共享数据库
     * 【注意】这里打开的是 poller 自己用的长连接（用于定期清理旧数据、查统计）。
     *   而每个网关采集时另开短连接，两者互不干扰。 */
    if (Storage_Open(&plr->db, cfg->db_path) != 0) {
        LOG_ERROR("Failed to open database: %s", cfg->db_path);
        return -1;   /* DB 打不开属于致命错误：数据没地方存，直接启动失败 */
    }

    LOG_INFO("Poller initialized: %d gateways", plr->gateway_count);
    return 0;
}

/* ---- 启动所有网关的定时器 ---- */
int Poller_Start(Poller *plr)
{
    if (plr->running) return 0;   /* 幂等：已在运行就直接返回 */

    /* 创建 epoll 实例
     * 参数 0 表示默认（不特殊指定大小，内核会自动扩容）。
     * 【对比】用 epoll_create(size) 传 size 的写法是历史遗留，新代码都用 0。 */
    plr->epoll_fd = epoll_create1(0);
    if (plr->epoll_fd == -1) {
        LOG_ERROR("epoll_create1 failed: %s", strerror(errno));
        return -1;
    }

    /* 创建 stop eventfd
     * eventfd 是一个"纯计数器型 fd"：write 一个 8 字节整数，它会变可读，
     * 是 Linux 上做"线程/进程间事件通知"的最轻量方案（比 pipe 省一个 fd）。 */
    plr->stop_fd = eventfd(0, EFD_NONBLOCK);
    if (plr->stop_fd == -1) {
        LOG_ERROR("eventfd failed: %s", strerror(errno));
        close(plr->epoll_fd);      /* 【资源回滚】失败路径要释放已申请的资源 */
        plr->epoll_fd = -1;
        return -1;
    }

    /* 事件结构体：EPOLLIN 表示"关心可读事件"。
     * timerfd 到期、eventfd 被写，都会表现为"可读"。 */
    struct epoll_event ev = {
        .events  = EPOLLIN,
    };

    /* 将 stop_fd 加入 epoll */
    ev.data.fd = plr->stop_fd;   /* 【关键】把 fd 存进 data，事件到来时才知是谁 */
    if (epoll_ctl(plr->epoll_fd, EPOLL_CTL_ADD, plr->stop_fd, &ev) == -1) {
        LOG_ERROR("epoll_ctl ADD stop_fd failed: %s", strerror(errno));
        close(plr->stop_fd);
        close(plr->epoll_fd);
        plr->stop_fd  = -1;
        plr->epoll_fd = -1;
        return -1;
    }

    /* 启动每个网关的 timerfd 并加入 epoll */
    for (int i = 0; i < plr->gateway_count; i++) {
        GatewayCtx *gw = &plr->gateways[i];

        if (Gateway_StartTimer(gw) != 0) {
            LOG_WARN("Gateway %d (%s) timer start failed, skipping",
                     i + 1, gw->cfg->name);
            continue;   /* 跳过这一个，其它网关照常启动（部分可用好过全不可用） */
        }

        ev.data.fd = gw->timer_fd;

        if (epoll_ctl(plr->epoll_fd, EPOLL_CTL_ADD, gw->timer_fd, &ev) == -1) {
            LOG_ERROR("epoll_ctl ADD timer_fd for GW%d failed: %s",
                      i + 1, strerror(errno));
            close(gw->timer_fd);
            gw->timer_fd = -1;
            continue;
        }

        LOG_INFO("Gateway %d (%s) timer started, interval=%us",
                 i + 1, gw->cfg->name, gw->cfg->poll_interval_sec);
    }

    plr->running = 1;
    LOG_INFO("Poller started: %d gateways, epoll_fd=%d",
             plr->gateway_count, plr->epoll_fd);
    return 0;
}

/* ---- 获取 epoll fd ---- */
int Poller_GetFd(Poller *plr)
{
    return plr->epoll_fd;
}

/* ---- 获取 stop fd ---- */
int Poller_GetStopFd(Poller *plr)
{
    return plr->stop_fd;
}

/* ---- 通知停止 ----
 * 【使用场景】如果以后把主循环放到别的线程，就可以在别处调用本函数，
 * 写一个 8 字节的 1 到 eventfd，让主循环的 epoll_wait 立刻返回并退出。 */
int Poller_SignalStop(Poller *plr)
{
    if (plr->stop_fd >= 0) {
        uint64_t val = 1;    /* eventfd 要求写入 8 字节整数（累加到内部计数器） */
        ssize_t s = write(plr->stop_fd, &val, sizeof(val));
        return (s == sizeof(val)) ? 0 : -1;   /* 写满 8 字节才算成功 */
    }
    return -1;
}

/* ---- 非阻塞处理一次 epoll 事件 ----
 * 这是整个采集引擎的"心跳函数"，主循环会不停调用它。
 * 返回 >0 = 本次处理的事件数，0 = 超时（无事件），-1 = 出错。 */
int Poller_RunOnce(Poller *plr, int timeout_ms)
{
    if (!plr->running || plr->epoll_fd == -1) return -1;

    /* events 数组大小 = 网关数 + 1（那个 1 是 stop_fd）。
     * 【为什么数组要开够？】若一次就绪的 fd 比数组大，epoll_wait 会返回全部数量，
     * 但只填满数组；剩下的要下次调用再取——所以数组建议开够以避免多轮处理。 */
    struct epoll_event events[POLLER_MAX_GATEWAYS + 1];
    /* 阻塞最多 timeout_ms：这期间哪个 fd 就绪就返回；一个周期处理完的所有网关事件都在这 */
    int nfds = epoll_wait(plr->epoll_fd, events,
                          POLLER_MAX_GATEWAYS + 1, timeout_ms);
    if (nfds == -1) {
        /* 【易错点】被信号（如 SIGHUP/Ctrl+C）打断时 errno=EINTR，
         *   这不是错误！应返回 0 让主循环继续跑（否则热加载会把程序搞崩）。 */
        if (errno == EINTR) return 0;  /* 被信号打断 (如 SIGHUP)，不算错误 */
        LOG_ERROR("epoll_wait failed: %s", strerror(errno));
        return -1;
    }
    if (nfds == 0) return 0;  /* 超时无事件 */

    int handled = 0;   /* 本次实际处理的事件数 */

    for (int i = 0; i < nfds; i++) {
        int fd = events[i].data.fd;   /* 取回当初存进去的 fd */

        /* ---- stop eventfd：外部线程请求退出 ---- */
        if (fd == plr->stop_fd) {
            uint64_t val;
            ssize_t s = read(plr->stop_fd, &val, sizeof(val));   /* 【必须读掉】否则会一直可读 */
            if (s > 0) {
                LOG_INFO("Stop event received");
                plr->running = 0;   /* 置退出标志，主循环据此结束 */
            }
            handled++;
            continue;
        }

        /* ---- 查找是哪个网关的 timerfd 到期 ----
         * 【说明】这里是 O(N) 线性查找。网关数量上限 16，完全可以接受。
         *   若要支持成百上千个网关，可改用 epoll_data.ptr 直接存 GatewayCtx*，
         *   把查找变成 O(1)。 */
        for (int g = 0; g < plr->gateway_count; g++) {
            GatewayCtx *gw = &plr->gateways[g];
            if (gw->timer_fd == fd) {
                /* 先把 timerfd 里的到期计数读掉（必须读，否则会一直可读导致 busy-loop）
                 * 【易错点】timerfd 的读是"清账"：不读，它就一直处于就绪状态，
                 *   epoll_wait 每次都立刻返回，CPU 直接跑满 100%。 */
                uint64_t expirations;
                ssize_t s = read(gw->timer_fd, &expirations, sizeof(expirations));
                (void)s; /* 非阻塞模式，忽略读失败 */

                /* 执行该网关的采集（只阻塞这一个网关，不影响其他） */
                gw->poll_count++;
                Gateway_PollOnce(gw, plr->cfg->db_path);

                handled++;
                break;   /* 已找到对应网关，跳出内层循环 */
            }
        }
    }

    /* ---- 定期清理旧数据（每 100 次总采集后清理一次） ----
     * 【为什么用这个判断而不是 s_total_polls % 100 == 0？】
     *   因为 handled 可能 >1，一次累加可能直接跨过 100 的整数倍（如 98+3=101），
     *   用 == 0 会漏掉这次清理。写成 "% 100 < handled" 表示"区间 [旧值, 新值) 内
     *   是否跨过了一个 100 的倍数"，这样无论一次加多少都不会漏。 */
    static uint64_t s_total_polls = 0;
    s_total_polls += handled;
    if (s_total_polls % 100 < (uint64_t)handled && plr->cfg->retain_days > 0) {
        int deleted = Storage_CleanOld(&plr->db, plr->cfg->retain_days);
        if (deleted > 0) {
            LOG_INFO("Auto-clean: removed %d old records", deleted);
        }
    }

    return handled;
}

/* ---- 立即采集所有网关 ----
 * 非定时触发的一次性全量采集。用于"启动后立刻采一轮"或 API 手动触发。 */
int Poller_PollAllNow(Poller *plr)
{
    int count = 0;
    for (int i = 0; i < plr->gateway_count; i++) {
        plr->gateways[i].poll_count++;
        if (Gateway_PollOnce(&plr->gateways[i], plr->cfg->db_path) == 0) {
            count++;   /* 只统计成功的 */
        }
    }
    return count;
}

/* ---- 打印所有网关统计 ---- */
void Poller_PrintStats(Poller *plr)
{
    printf("\n========== Poller Stats (%d gateways) ==========\n", plr->gateway_count);
    uint64_t total_poll = 0, total_ok = 0, total_fail = 0;

    for (int i = 0; i < plr->gateway_count; i++) {
        GatewayCtx *gw = &plr->gateways[i];
        printf("  [GW%d] %s %s:%u\n",
               i + 1, gw->cfg->name, gw->cfg->host, gw->cfg->port);
        printf("         polls=%lu ok=%lu fail=%lu\n",
               gw->poll_count, gw->success_count, gw->fail_count);
        if (gw->success_count > 0) {
            /* 把 Unix 时间戳格式化成人类可读的时间串 */
            char ts_buf[32] = "N/A";
            if (gw->last_ts > 0) {
                struct tm *tm = localtime(&gw->last_ts);
                strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%d %H:%M:%S", tm);
            }
            /* last_temp 存的是放大 10 倍的整数，打印时再强转 int16 并 /10.0 */
            printf("         last: T=%.1f℃ H=%.1f%% @ %s\n",
                   (int16_t)gw->last_temp / 10.0,
                   (int16_t)gw->last_hum / 10.0,
                   ts_buf);
        }
        printf("         status: %s\n",
               Modbus_IsConnected(&gw->mb) ? "connected" : "disconnected");

        total_poll += gw->poll_count;
        total_ok   += gw->success_count;
        total_fail += gw->fail_count;
    }
    printf("  ---------------------------------\n");
    printf("  TOTAL : polls=%lu ok=%lu fail=%lu\n", total_poll, total_ok, total_fail);
    printf("  DB    : inserts=%u records=%d\n",
           plr->db.insert_count, Storage_GetCount(&plr->db));
    printf("===============================================\n");
}

/* ---- 比较两个网关配置是否一致 ----
 * 用于热加载时判断"这个网关的配置有没有变"：
 *   没变 → 保持现有连接和定时器（避免无谓的断线重连、丢数据）；
 *   变了 → 重建该网关。
 * 【易错点】浮点字段（temp_high 等）用 != 直接比较：
 *   只要不是"配置文本完全一样"，解析出来的 double 就是同一个值，
 *   这里比较的是"是否相同"而非"是否近似"，所以用 != 是合适的。 */
static int GatewayConfig_Equals(const GatewayConfig *a, const GatewayConfig *b)
{
    if (!a || !b) return 0;                                       /* 空指针视为不等 */
    if (strcmp(a->host, b->host) != 0) return 0;                  /* 字符串用 strcmp */
    if (a->port != b->port) return 0;
    if (strcmp(a->name, b->name) != 0) return 0;
    if (a->poll_interval_sec != b->poll_interval_sec) return 0;
    if (a->slave_id != b->slave_id) return 0;
    if (a->reg_start != b->reg_start) return 0;
    if (a->reg_count != b->reg_count) return 0;
    if (a->retry_count != b->retry_count) return 0;
    if (a->retry_interval_ms != b->retry_interval_ms) return 0;
    if (a->temp_high != b->temp_high) return 0;                   /* 阈值参与比较 */
    if (a->temp_low != b->temp_low) return 0;
    if (a->hum_high != b->hum_high) return 0;
    if (a->hum_low != b->hum_low) return 0;
    if (a->alarm_debounce != b->alarm_debounce) return 0;
    return 1;   /* 全部字段一致 */
}

/* ---- 热加载网关配置（SIGHUP） ----
 * 【这个函数的设计目标】改配置不用重启进程，采集"几乎不中断"。
 *   做法是分三段处理：共有的网关逐个比对、多出来的删除、新增的建立。
 *   DB 连接和 epoll 实例全程保留（这就是"中断 < 1ms"的原因）。
 * 【注意生命周期】new_cfg 由调用方（main 的 do_reload）堆分配并负责释放，
 *   本函数只把指针挂上去，不拥有它。 */
int Poller_Reload(Poller *plr, AppConfig *new_cfg)
{
    int old_count = plr->gateway_count;
    int new_count = new_cfg->gateway_count;
    /* 新旧数量的较小值 = 两边都存在的"公共段"，这些需要逐个比对 */
    int keep_idx  = (old_count < new_count) ? old_count : new_count;

    LOG_INFO("========================================");
    LOG_INFO("Hot reload: %d gateways -> %d gateways",
             old_count, new_count);

    /* ---- 1. 逐一对比：配置不变则保持连接，变了才重建 ---- */
    struct epoll_event ev = { .events = EPOLLIN };   /* 复用一个事件结构体 */

    for (int i = 0; i < keep_idx; i++) {
        GatewayCtx *gw = &plr->gateways[i];
        if (GatewayConfig_Equals(gw->cfg, &new_cfg->gateways[i])) {
            /* 配置未变 → 保持连接和 timer */
            LOG_INFO("Reload: GW%d '%s' unchanged, keeping connection",
                     i + 1, gw->cfg->name);
            /* 【关键】只需把 cfg 指针指向新配置内存即可（保持连接不断） */
            gw->cfg = &new_cfg->gateways[i];  /* 指向新配置内存 */
        } else {
            /* 配置变了 → 重建该网关
             * 顺序：先从 epoll 摘除 → 关 timerfd → 断 Modbus → 整体清零 → 重新 Init */
            LOG_INFO("Reload: GW%d '%s' config changed, reconnecting",
                     i + 1, gw->cfg->name);
            if (gw->timer_fd >= 0) {
                epoll_ctl(plr->epoll_fd, EPOLL_CTL_DEL, gw->timer_fd, NULL);  /* 必须先摘 */
                close(gw->timer_fd);
                gw->timer_fd = -1;
            }
            Modbus_Disconnect(&gw->mb);
            memset(gw, 0, sizeof(GatewayCtx));          /* 彻底重置，避免残留旧配置引用 */
            Gateway_Init(gw, &new_cfg->gateways[i]);
            gw->index = i;                              /* 再次强调：Init 之后才设 index */
            if (Gateway_StartTimer(gw) == 0) {
                ev.data.fd = gw->timer_fd;
                if (epoll_ctl(plr->epoll_fd, EPOLL_CTL_ADD,
                              gw->timer_fd, &ev) == -1) {
                    LOG_ERROR("Reload: epoll_ctl ADD GW%d failed: %s",
                              i + 1, strerror(errno));
                    close(gw->timer_fd);
                    gw->timer_fd = -1;
                }
            }
        }
    }

    /* ---- 2. 删除的网关：清理 timer 和连接 ----
     * 新配置的网关数比旧的少 → [new_count, old_count) 这一段是被删掉的 */
    for (int i = new_count; i < old_count; i++) {
        GatewayCtx *gw = &plr->gateways[i];
        LOG_INFO("Reload: GW%d '%s' removed, stopping",
                 i + 1, gw->cfg ? gw->cfg->name : "?");
        if (gw->timer_fd >= 0) {
            epoll_ctl(plr->epoll_fd, EPOLL_CTL_DEL, gw->timer_fd, NULL);
            close(gw->timer_fd);
            gw->timer_fd = -1;
        }
        Modbus_Disconnect(&gw->mb);
        memset(gw, 0, sizeof(GatewayCtx));
        gw->timer_fd = -1;   /* 清零后要把 fd 重新置 -1，表示"无定时器" */
    }

    /* ---- 3. 新增的网关：全新初始化 ----
     * 新配置的网关数比旧的多 → [old_count, new_count) 是新增的 */
    for (int i = old_count; i < new_count; i++) {
        LOG_INFO("Reload: GW%d '%s' new, initializing",
                 i + 1, new_cfg->gateways[i].name);
        GatewayCtx *gw = &plr->gateways[i];
        memset(gw, 0, sizeof(GatewayCtx));
        Gateway_Init(gw, &new_cfg->gateways[i]);
        gw->index = i;
        if (Gateway_StartTimer(gw) == 0) {
            ev.data.fd = gw->timer_fd;
            if (epoll_ctl(plr->epoll_fd, EPOLL_CTL_ADD,
                          gw->timer_fd, &ev) == -1) {
                LOG_ERROR("Reload: epoll_ctl ADD GW%d failed: %s",
                          i + 1, strerror(errno));
                close(gw->timer_fd);
                gw->timer_fd = -1;
            }
        }
    }

    /* ---- 4. 更新配置指针和计数 ---- */
    plr->cfg = new_cfg;
    plr->gateway_count = new_count;

    /* 清理高槽位无用数据（把 [new_count, MAX) 的残留清掉，避免误用） */
    for (int i = new_count; i < POLLER_MAX_GATEWAYS; i++) {
        memset(&plr->gateways[i], 0, sizeof(GatewayCtx));
        plr->gateways[i].timer_fd = -1;
    }

    /* ---- 5. 统计启动数量 ---- */
    int started = 0;
    for (int i = 0; i < new_count; i++) {
        if (plr->gateways[i].timer_fd >= 0) started++;   /* fd 有效即认为已启动 */
    }

    /* 打一份"热加载结果清单"，便于确认增删改是否正确生效 */
    LOG_INFO("Hot reload complete: %d/%d gateways started", started, new_count);
    LOG_INFO("  Gateways: %d -> %d", old_count, new_count);
    for (int i = 0; i < new_count; i++) {
        LOG_INFO("    [GW%d] %s (%s:%u, interval=%us)",
                 i + 1, new_cfg->gateways[i].name,
                 new_cfg->gateways[i].host, new_cfg->gateways[i].port,
                 new_cfg->gateways[i].poll_interval_sec);
    }
    LOG_INFO("========================================");
    return 0;
}

/* ---- 停止所有 ----
 * 关闭顺序：先网关（含 timerfd 和 Modbus）→ 再 epoll/eventfd → 最后关 DB。 */
void Poller_Stop(Poller *plr)
{
    plr->running = 0;

    /* 停止每个网关（内部会关 timerfd、断开 Modbus、打印统计） */
    for (int i = 0; i < plr->gateway_count; i++) {
        Gateway_Stop(&plr->gateways[i]);
    }

    if (plr->epoll_fd != -1) {
        close(plr->epoll_fd);
        plr->epoll_fd = -1;
    }
    if (plr->stop_fd != -1) {
        close(plr->stop_fd);
        plr->stop_fd = -1;
    }

    Storage_Close(&plr->db);
    LOG_INFO("Poller stopped");
}

/* ---- 获取网关数据 ----
 * 带边界检查的只读访问器，返回 const 指针防止调用方误改。 */
const GatewayCtx *Poller_GetGateway(const Poller *plr, int idx)
{
    if (idx < 0 || idx >= plr->gateway_count) return NULL;   /* 越界返回 NULL */
    return &plr->gateways[idx];
}
