#ifndef __HTTP_SERVER_H
#define __HTTP_SERVER_H

/*
 * http_server.h — 嵌入式 HTTP 服务器接口
 *
 * 基于 Linux socket + epoll 的自实现 HTTP 服务器。
 * 提供一个独立线程运行的 HTTP 服务，通过共享数据库读取数据。
 *
 * REST API:
 *   GET /api/latest               → 各网关最新数据 JSON
 *   GET /api/history?gateway=N&hours=24 → 历史数据 JSON
 *   GET /                         → 静态 dashboard.html
 *
 * 使用示例:
 *   HTTPServer srv;
 *   HttpServer_Start(&srv, 8080, "data/modbus_data.db", &cfg);
 *   // 主线程继续做其他事
 *   HttpServer_Stop(&srv);  // 停止
 */

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* HTTP 服务器上下文 */
typedef struct {
    int          port;
    int          server_fd;
    int          running;
    char         db_path[256];
    void        *thread;               /* pthread_t */
    int          thread_started;
    void        *thread_args;          /* HttpThreadArgs* — 用于 Stop 时发信号 */
} HTTPServer;

/**
 * @brief 初始化并启动 HTTP 服务器（在独立线程中运行）
 * @param srv     服务器上下文
 * @param port    监听端口
 * @param db_path 数据库路径（服务器内部会独立打开）
 * @return 0=成功, -1=失败
 */
int HttpServer_Start(HTTPServer *srv, uint16_t port, const char *db_path,
                       AppConfig *cfg);

/**
 * @brief 停止 HTTP 服务器
 */
void HttpServer_Stop(HTTPServer *srv);

/**
 * @brief 检查服务器是否正在运行
 */
int HttpServer_IsRunning(HTTPServer *srv);

#endif /* __HTTP_SERVER_H */
