#ifndef __GATEWAY_H
#define __GATEWAY_H

/*
 * gateway.h — 单个网关的运行时上下文
 *
 * 每个网关拥有独立的:
 *   - ModbusMaster (TCP 连接)
 *   - timer_fd (独立采集周期)
 *   - 统计信息 (采集次数 / 成功 / 失败)
 *   - 最后一次采集的数据
 *
 * 所有网关的 timer_fd 注册到 poller 的同一个 epoll 中，
 * 实现并发轮询：一个网关超时/断开不影响其他网关。
 */

#include "config.h"
#include "modbus_master.h"
#include "alarm.h"
#include <stdint.h>
#include <time.h> 
/* 单个网关运行时上下文 */
typedef struct {
    /* 配置（引用外部，不拥有） */
    GatewayConfig *cfg;
    int            index;          /* 在 poller 数组中的索引 (0-based) */

    /* Modbus 主站（每个网关独立连接） */
    ModbusMaster   mb;

    /* 定时器 */
    int            timer_fd;       /* 该网关的 timerfd */
    int            poll_interval;  /* 实际使用的轮询间隔（秒）*/

    /* 统计 */
    uint64_t       poll_count;
    uint64_t       success_count;
    uint64_t       fail_count;
    int32_t        last_temp;      /* 上次温度 (0.1℃) */
    int32_t        last_hum;       /* 上次湿度 (0.1%RH) */
    time_t         last_ts;        /* 上次成功采集时间戳 */

    /* 报警引擎状态（4 通道去抖计数器） */
    AlarmState     alarm;

    /* 状态 */
    int            running;
} GatewayCtx;

/**
 * @brief 初始化网关上下文
 * @param ctx  网关上下文（调用者分配）
 * @param cfg  网关配置
 * @return 0=成功, -1=失败
 */
int Gateway_Init(GatewayCtx *ctx, GatewayConfig *cfg);

/**
 * @brief 创建该网关的 timerfd 并启动定时器
 * @param ctx 网关上下文
 * @return 0=成功, -1=失败
 */
int Gateway_StartTimer(GatewayCtx *ctx);

/**
 * @brief 执行一次采集（Modbus 读取 + DB 写入）
 * @param ctx     网关上下文
 * @param db_path 数据库路径（用于打开/写入）
 * @return 0=成功, -1=失败
 */
int Gateway_PollOnce(GatewayCtx *ctx, const char *db_path);

/**
 * @brief 获取该网关的 timer_fd（供 epoll 管理）
 */
int Gateway_GetTimerFd(GatewayCtx *ctx);

/**
 * @brief 停止并清理
 */
void Gateway_Stop(GatewayCtx *ctx);

#endif /* __GATEWAY_H */
