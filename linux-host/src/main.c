/* ============================================================================
 * main.c — Modbus Backend 主入口 v2（多网关 + HTTP API）
 *
 * 程序由三块组成，main.c 是三块的总装配线:
 *   1) Poller（采集引擎）  —— 定时通过 Modbus-TCP 读温湿度，写进 SQLite
 *   2) Storage（存储层）   —— 数据/告警持久化到 SQLite
 *   3) HttpServer（Web层） —— 独立线程对外提供 REST API + 仪表盘
 *
 * 启动流程:
 *   1) 解析命令行参数（getopt）
 *   2) 加载配置 config.ini（支持多网关）
 *   3) 初始化日志（终端彩色 + 文件）
 *   4) 初始化轮询引擎（N 个网关 + 共享 DB）
 *   5) 启动定时轮询（每网关一个 timerfd，统一挂 epoll）
 *   6) 启动 HTTP 服务器线程
 *   7) epoll 主循环（等 timerfd 事件 或 stop 信号）
 *   8) SIGINT/SIGTERM 优雅退出
 *
 * 采集用单线程 epoll + 独立 Web 线程：采集准实时、周期性强、量小，单线程即可
 * 扛住几十个网关且无需加锁；HTTP 请求突发、可能阻塞，塞进主线程会拖慢采集节拍。
 * 两者只通过 SQLite（读写文件，自带并发控制）交换数据。
 * ============================================================================
 */
#include <stdio.h>      /* printf / snprintf / fprintf */
#include <stdlib.h>     /* malloc / free / atoi */
#include <string.h>     /* memset / strcmp */
#include <signal.h>     /* sigaction / signal / SIGINT 等 */
#include <unistd.h>     /* getopt / close */
#include <errno.h>      /* errno：系统调用错误码 */
#include <sys/epoll.h>  /* epoll：Linux 高并发 IO 多路复用，主循环核心 */

#include "config.h"      /* AppConfig / Config_Load / Config_Print */
#include "logger.h"      /* LOG_INFO / LOG_ERROR 等宏 */
#include "poller.h"      /* Poller：采集引擎 */
#include "http_server.h" /* HTTPServer：Web 服务 */

/* ---- 全局标志 ---- */
/* volatile 告诉编译器：这些变量可能被"信号处理函数"或"其他线程"异步修改，
 * 每次使用都必须从内存重新读取，不能缓存在寄存器里。
 * 这是信号处理里最容易踩的坑：不加 volatile 可能陷入死循环。 */
static volatile int g_running = 1;   /* 1=继续运行, 0=收到退出信号 */
static volatile int g_reload  = 0;   /* SIGHUP 热加载标志：1=需要重读配置 */
static const char *g_config_path;    /* 配置文件路径（信号处理里要用） */

/* ---- 信号处理 ----
 * 【重要】信号处理函数里只能做"异步信号安全"的操作：
 *   不能 malloc、不能 printf、不能加锁、不能调用不可重入函数。
 *   所以我们只改一个标志位，真正的重活留给主循环去干。 */
static void signal_handler(int sig)
{
    if (sig == SIGHUP) {
        g_reload = 1;       /* 主循环检测到后会执行热加载（重读配置、增删网关） */
    } else {
        g_running = 0;      /* SIGINT(Ctrl+C) / SIGTERM(kill) → 通知主循环退出 */
    }
}

/* ---- 注册信号 ---- */
static int setup_signals(void)
{
    struct sigaction sa;              /* 比老式 signal() 更可控的信号动作结构体 */
    memset(&sa, 0, sizeof(sa));       /* 清零，避免残留字段导致未定义行为 */
    sa.sa_handler = signal_handler;   /* 指定处理函数 */
    sigemptyset(&sa.sa_mask);         /* 处理期间不额外屏蔽其它信号 */
    sa.sa_flags = 0;                  /* 默认标志（不加 SA_RESTART，让 epoll_wait 能被信号打断） */

    /* 逐个注册。任一个失败就返回 -1，让 main 报错退出 */
    if (sigaction(SIGINT,  &sa, NULL) == -1) return -1;  /* Ctrl+C */
    if (sigaction(SIGTERM, &sa, NULL) == -1) return -1;  /* kill 默认信号 */
    if (sigaction(SIGHUP,  &sa, NULL) == -1) return -1;  /* 新增：kill -HUP 触发配置热加载 */

    /* 忽略 SIGPIPE：向"已被对端关闭"的 socket 写数据时，内核默认会发 SIGPIPE 杀死进程。
     * HTTP 客户端提前断开连接是常态，若不忽略，服务会被这种正常情况意外干掉。
     * 忽略后 write() 会返回 -1 并置 errno=EPIPE，由代码自行处理，安全得多。 */
    signal(SIGPIPE, SIG_IGN);

    return 0;
}

/* ---- 打印使用方法 ---- */
static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n\n", prog);
    printf("Options:\n");
    printf("  -c <file>   Config file path (default: config.ini)\n");
    printf("  -h          Show this help\n\n");
    printf("Signals:\n");
    printf("  SIGINT/SIGTERM  Graceful shutdown\n");
    printf("  SIGHUP          Hot reload config (add/remove/modify gateways without restart)\n\n");
    printf("HTTP API (when running):\n");
    printf("  GET  /api/latest               All gateways latest data\n");
    printf("  GET  /api/history?gateway=N&hours=24  History data\n");
    printf("  GET  /api/alarms?gateway=N&status=active  Alarm events\n");
    printf("  POST /api/ota/upload?gateway=N Upload .ota firmware & start upgrade\n");
    printf("  GET  /api/ota/status?gateway=N OTA upgrade status\n");
    printf("  POST /api/ota/cancel?gateway=N Cancel OTA upgrade\n");
    printf("  GET  /                         Dashboard (Chart.js)\n");
}

/* ---- 执行配置热加载 ---- */
/* 【为什么用堆分配的 cfg？】
 *   Poller 内部保存的是 AppConfig* 指针（plr->cfg）。若用栈上的局部变量，
 *   函数返回后指针就悬空了。热加载要"新配置生效后再释放旧配置"，
 *   所以新旧配置都必须放在堆上，由本文件负责生命周期管理。 */
static AppConfig *g_prev_cfg = NULL;  /* 上一次热加载分配的配置，下一次 reload 时释放 */

static void do_reload(Poller *plr, AppConfig *cfg)
{
    LOG_INFO("SIGHUP received, reloading config: %s", g_config_path);

    /* 堆分配新配置，Poller_Reload 会将 plr->cfg 指向它 */
    AppConfig *new_cfg = (AppConfig *)malloc(sizeof(AppConfig));
    if (!new_cfg) {
        LOG_ERROR("Reload: malloc failed");
        return;   /* 分配失败就放弃本次热加载，继续用旧配置，最稳妥 */
    }

    /* 解析失败也不动现有配置，保证"配置写错不会导致服务崩" */
    if (Config_Load(g_config_path, new_cfg) != 0) {
        LOG_WARN("Config reload FAILED, keeping current config");
        free(new_cfg);   /* 失败要释放，避免内存泄漏 */
        return;
    }

    /* 更新日志级别（运行时生效，这样调 DEBUG 不用重启进程） */
    LogLevel new_lv = (LogLevel)Config_ParseLogLevel(new_cfg->log_level_str);
    LOG_SetLevel(new_lv);

    /* 执行 poller 热加载 — plr->cfg 现在指向堆上的 new_cfg */
    Poller_Reload(plr, new_cfg);

    /* 释放上一次堆分配的配置（首次 reload 时 g_prev_cfg==NULL 跳过）。
     * 注意顺序：必须等 Poller_Reload 完成、没有任何人再引用旧配置之后才能 free。 */
    if (g_prev_cfg) {
        free(g_prev_cfg);
    }
    g_prev_cfg = new_cfg;   /* 记住本次分配，供下次回收 */

    /* 更新主配置副本（main 栈上的 cfg，用于打印和"新配置查不到时兜底"） */
    *cfg = *new_cfg;

    Config_Print(cfg);
    LOG_INFO("Config hot reload complete");
}

/* ============================================================================
 * main()
 * ============================================================================
 */
int main(int argc, char **argv)
{
    const char *config_path = "config.ini";  /* 默认配置文件名，可被 -c 覆盖 */

    /* ---- 解析命令行 ----
     * getopt 第三参 "c:h" 中，冒号表示该选项需要一个参数（-c file）。
     * 返回 -1 表示所有选项解析完毕。 */
    int opt;
    while ((opt = getopt(argc, argv, "c:h")) != -1) {
        switch (opt) {
            case 'c': config_path = optarg; break;  /* optarg 指向 -c 后面的值 */
            case 'h':
            default:
                print_usage(argv[0]);
                return 0;
        }
    }

    /* ---- 加载配置 ---- */
    /* cfg 放在栈上：它是"主线程独占"的副本，不需要跨线程共享 */
    AppConfig cfg;
    if (Config_Load(config_path, &cfg) != 0) {
        /* 配置文件打不开/格式错时用内置默认值继续跑，让程序"尽力而为"而不是直接退出。
         * 对运维友好：忘了放配置文件也能起来，再通过 API 观察。 */
        fprintf(stderr, "Failed to load config '%s', using defaults.\n", config_path);
        Config_Default(&cfg);
    }
    Config_Print(&cfg);   /* 启动时把生效配置打印一遍，方便排查"我改的到底有没有生效" */

    /* ---- 初始化日志 ---- */
    /* log_file[0] 为空表示"只输出到终端"；非空则同时写文件 */
    LogLevel log_lv = (LogLevel)Config_ParseLogLevel(cfg.log_level_str);
    LOG_Init(cfg.log_file[0] ? cfg.log_file : NULL, log_lv, cfg.log_color);

    /* 保存配置文件路径（供信号处理使用——热加载时要重新读这个文件） */
    g_config_path = config_path;

    /* ---- 注册信号 ---- */
    if (setup_signals() != 0) {
        LOG_ERROR("Failed to setup signal handlers");
        return 1;
    }

    /* ---- 初始化轮询引擎（多网关） ----
     * 注意 cfg 是栈变量，Poller 只保存它的指针。
     * main 的生命周期覆盖整个进程，所以这个指针一直有效。 */
    Poller plr;
    if (Poller_Init(&plr, &cfg) != 0) {
        LOG_ERROR("Poller init failed");
        return 1;
    }

    /* ---- 启动定时轮询 ---- */
    if (Poller_Start(&plr) != 0) {
        LOG_ERROR("Poller start failed");
        return 1;
    }

    /* ---- 启动横幅：把关键信息一次性打出来，方便运维快速确认 ----
     * 包含每个网关的 名称 / 地址 / 端口 / 采集周期 */
    LOG_INFO("========================================");
    LOG_INFO("Modbus Backend v2 started: %d gateways", cfg.gateway_count);
    for (int i = 0; i < cfg.gateway_count; i++) {
        LOG_INFO("  GW%d: %s (%s:%u, interval=%us)",
                 i + 1, cfg.gateways[i].name,
                 cfg.gateways[i].host, cfg.gateways[i].port,
                 cfg.gateways[i].poll_interval_sec);
    }
    LOG_INFO("HTTP API: http://localhost:%u", cfg.http_port);
    LOG_INFO("Dashboard: http://localhost:%u/", cfg.http_port);
    LOG_INFO("========================================");

    /* ---- 启动 HTTP 服务器（独立线程） ----
     * 非致命：端口被占用/权限不足时只是没有 Web 界面，采集功能照常工作。
     * 所以这里只 WARN，不 return。 */
    HTTPServer http_srv;
    if (HttpServer_Start(&http_srv, cfg.http_port, cfg.db_path, &cfg) != 0) {
        LOG_WARN("HTTP server failed to start (non-fatal)");
    }

    /* ---- 主循环 ----
     * 主线程专注 epoll 事件分发：Poller_RunOnce 内部 epoll_wait 等待 timerfd 到期。
     * 1 秒超时不是为了采集精度，而是给主循环一个"喘息点"去检查 g_running/g_reload，
     * 否则若一直没事件，epoll_wait 会无限阻塞，连 Ctrl+C 都反应不及时。 */
    while (g_running) {
        /*
         * Poller_RunOnce 内部调用 epoll_wait(plr->epoll_fd, ..., 1000ms)
         * 管理 N 个 timerfd：
         *   哪个 timerfd 到期，就采集对应的网关
         *   一个网关超时不阻塞其他网关
         */
        int ret = Poller_RunOnce(&plr, 1000);

        if (ret == -1) {
            LOG_ERROR("Poller_RunOnce error");
            break;   /* 轮询引擎出现不可恢复错误，跳出主循环走退出流程 */
        }

        /* ---- 检测 SIGHUP 热加载信号 ----
         * 信号处理只置标志，真正的重活在这里做——避免在信号上下文里做危险操作 */
        if (g_reload) {
            g_reload = 0;             /* 先清标志，防止重入 */
            do_reload(&plr, &cfg);
        }
    }

    /* ---- 关闭 ----
     * 顺序很重要：先停 Web（不再有新请求进来），再停采集（不再有新数据写入），
     * 最后关日志（保证上面的关闭日志能落盘）。 */
    LOG_INFO("Shutting down...");

    HttpServer_Stop(&http_srv);
    Poller_Stop(&plr);
    LOG_CLOSE();

    printf("\nBye.\n");   /* 直接 printf 而非 LOG：此时日志系统已关闭 */
    return 0;
}
