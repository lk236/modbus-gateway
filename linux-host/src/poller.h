#ifndef __POLLER_H
#define __POLLER_H

/*
 * poller.h — 多网关并发轮询引擎接口 v2
 *
 * 基于 epoll 统一管理 N 个网关各自的 timerfd：
 *   每个网关独立定时，互不阻塞。
 *   一个网关断线不影响其他网关的采集周期。
 *
 * epoll 管理 N + 1 个 fd：
 *   - N 个 timerfd（每个网关一个）
 *   - 1 个 stop_event_fd（用于优雅退出）
 *
 * 使用示例:
 *   Poller plr;
 *   Poller_Start(&plr, &cfg);  // 启动 N 个网关的轮询
 *   // 主循环中:
 *   while (running) {
 *       Poller_RunOnce(&plr, 1000);
 *   }
 *   Poller_Stop(&plr);         // 停止所有
 */

#include "config.h"
#include "gateway.h"
#include "storage.h"

#define POLLER_MAX_GATEWAYS CONFIG_MAX_GATEWAYS

/* 轮询引擎上下文 */
typedef struct {
    /* 多网关 */
    GatewayCtx   gateways[POLLER_MAX_GATEWAYS];
    int          gateway_count;
    AppConfig   *cfg;           /* 配置指针（引用外部） */

    /* 数据库（全局共享一个连接） */
    StorageDB    db;

    /* 内部状态 */
    int          running;
    int          epoll_fd;      /* epoll 文件描述符 */
    int          stop_fd;       /* eventfd 用于触发停止 */
} Poller;

/**
 * @brief 初始化轮询引擎
 * @param plr  轮询引擎
 * @param cfg  应用配置
 * @return 0=成功, -1=失败
 */
int  Poller_Init(Poller *plr, AppConfig *cfg);

/**
 * @brief 启动定时轮询（创建 N 个 timerfd + 1 个 epoll）
 */
int  Poller_Start(Poller *plr);

/**
 * @brief 非阻塞处理一次 epoll 事件
 * @param timeout_ms  epoll_wait 超时
 * @return >0=处理了事件数, 0=超时, -1=错误
 */
int  Poller_RunOnce(Poller *plr, int timeout_ms);

/**
 * @brief 获取 epoll_fd（供外部集成到主循环）
 */
int  Poller_GetFd(Poller *plr);

/**
 * @brief 获取 stop_fd（eventfd 读触发代表需要停止）
 */
int  Poller_GetStopFd(Poller *plr);

/**
 * @brief 向 stop_fd 发信号，通知主循环退出
 */
int  Poller_SignalStop(Poller *plr);

/**
 * @brief 立即执行一次所有网关的采集（非定时触发）
 */
int  Poller_PollAllNow(Poller *plr);

/**
 * @brief 运行时热加载配置（SIGHUP 触发）
 *        停旧 timerfd → 重建网关 → 启新 timerfd
 *        DB 连接和 epoll 全程保持，采集中断 < 1ms
 * @param plr     轮询引擎
 * @param new_cfg 新配置（调用方确保生命周期长于本次调用）
 * @return 0=成功, -1=失败
 */
int  Poller_Reload(Poller *plr, AppConfig *new_cfg);

/**
 * @brief 停止所有轮询
 */
void Poller_Stop(Poller *plr);

/**
 * @brief 打印统计信息（所有网关汇总）
 */
void Poller_PrintStats(Poller *plr);

/**
 * @brief 获取某个网关的最新数据
 */
const GatewayCtx *Poller_GetGateway(const Poller *plr, int idx);

#endif /* __POLLER_H */
