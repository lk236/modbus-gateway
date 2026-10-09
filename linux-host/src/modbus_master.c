/* ============================================================================
 * modbus_master.c — Modbus TCP 主站封装实现
 *
 * 上位机是 Modbus 主站（Master/Client），下位机网关是从站（Slave/Server，
 * 监听 502）。要"问"它要数据（读保持寄存器 03）。
 *
 * 在 libmodbus 之上封一层，补齐它不管的事——网络抖动、设备断电重启是工程上
 * 真正折磨人的部分:
 *   - 自动重连机制（Modbus_Reconnect）
 *   - 错误计数与追踪（err_count，连续错 N 次就主动断开）
 *   - 连接状态管理（connected 标志，业务层据此决定要不要走重连）
 * ============================================================================
 */
#include "modbus_master.h"
#include "logger.h"
#include <modbus/modbus.h>   /* libmodbus 主头文件，链接需要 -lmodbus */
#include <stdio.h>      // snprintf
#include <string.h>
#include <errno.h>
#include <poll.h>       /* poll()：等待非阻塞 socket 可写（用于处理 EINPROGRESS） */
#include <sys/types.h>
#include <sys/socket.h> /* getsockopt：查 socket 的真正错误码 */

/* ---- 连接 ----
 * 建立到「下位机网关」的 Modbus-TCP 连接。
 * 返回 0=成功，-1=失败。 */
int Modbus_Connect(ModbusMaster *mb, const char *host, int port)
{
    /* 若已存在旧上下文，先干净地断开，避免句柄泄漏 */
    if (mb->ctx) {
        Modbus_Disconnect(mb);
    }

    /* 记录下地址，重连时（Modbus_Reconnect）需要用 */
    snprintf(mb->host, sizeof(mb->host), "%s", host);
    mb->port = port;

    /* 创建 Modbus TCP 上下文（此时还没真正建 TCP 连接，只是准备好句柄） */
    mb->ctx = modbus_new_tcp(host, port);
    if (mb->ctx == NULL) {
        /* 通常是内存不足。errno 可能不准确，但带上总比不带有用 */
        LOG_ERROR("modbus_new_tcp(%s:%d) failed: %s", host, port, strerror(errno));
        return -1;
    }

    /* 设置超时 (1s)
     * 【为什么必须设超时？】若不设，libmodbus 默认超时可能很长（甚至无限等待），
     * 一个掉线的网关就能把这个采集线程卡死，其它网关全遭殃。
     * 1s 是采集类场景的常用取值：够一次正常往返，又不至于长时间阻塞。 */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    modbus_set_response_timeout(mb->ctx, tv.tv_sec, tv.tv_usec);  /* 等待"响应首字节"的超时 */
    modbus_set_byte_timeout(mb->ctx, tv.tv_sec, tv.tv_usec);      /* 响应字节间的超时 */

    /* 真正建立 TCP 连接 */
    if (modbus_connect(mb->ctx) == -1) {
        /* 注意用的是 modbus_strerror（libmodbus 有自己的错误码体系），不是 strerror */
        LOG_ERROR("modbus_connect(%s:%d) failed: %s", host, port, modbus_strerror(errno));
        modbus_free(mb->ctx);   /* 失败必须释放，否则内存泄漏 */
        mb->ctx = NULL;
        return -1;
    }

    mb->connected   = 1;    /* 标记已连接 */
    mb->err_count   = 0;    /* 计数清零 */
    mb->last_err_ms = 0;

    LOG_INFO("Connected to Modbus gateway %s:%d", host, port);
    return 0;
}

/* ---- 断开 ----
 * 【注意】libmodbus 有两步：modbus_close 关底层 socket，modbus_free 释放上下文。
 * 两步都要做，只做一半就是资源泄漏。 */
void Modbus_Disconnect(ModbusMaster *mb)
{
    if (mb->ctx) {
        modbus_close(mb->ctx);   /* 关闭 TCP 连接 */
        modbus_free(mb->ctx);    /* 释放 libmodbus 内部结构 */
        mb->ctx = NULL;          /* 置空，防止"野指针"被再次使用 */
    }
    mb->connected = 0;
    LOG_INFO("Disconnected from %s:%d", mb->host, mb->port);
}

/* ---- 重连 ----
 * 幂等设计：已连接时直接返回成功，调用方不用先判断状态，用起来简单。 */
int Modbus_Reconnect(ModbusMaster *mb)
{
    if (mb->connected && mb->ctx) {
        return 0;  /* 已连接 */
    }

    LOG_INFO("Reconnecting to %s:%d ...", mb->host, mb->port);

    /* 清理旧的 ctx（可能是半死不活的状态：ctx 非空但连接已断） */
    if (mb->ctx) {
        modbus_close(mb->ctx);
        modbus_free(mb->ctx);
        mb->ctx = NULL;
    }

    mb->ctx = modbus_new_tcp(mb->host, mb->port);
    if (mb->ctx == NULL) {
        LOG_ERROR("Reconnect: modbus_new_tcp failed: %s", strerror(errno));
        return -1;
    }

    /* 重连同样要设超时——否则一次重连就能把线程卡住 */
    struct timeval tv = { .tv_sec = 1, .tv_usec = 0 };
    modbus_set_response_timeout(mb->ctx, tv.tv_sec, tv.tv_usec);
    modbus_set_byte_timeout(mb->ctx, tv.tv_sec, tv.tv_usec);

    if (modbus_connect(mb->ctx) == -1) {
        /* 【EINPROGRESS 是什么】
         * 当 socket 处于"非阻塞模式"时，TCP 三次握手不会等它完成，
         * connect 会立刻返回 -1 并把 errno 置为 EINPROGRESS（"连接正在进行中"）。
         * 这不代表失败！正确做法是用 poll() 等待该 fd 变为"可写"，
         * 再用 getsockopt(SO_ERROR) 查询握手到底成没成。 */
        if (errno == EINPROGRESS) {
            int fd = modbus_get_socket(mb->ctx);
            if (fd >= 0) {
                struct pollfd pfd;
                pfd.fd = fd;
                pfd.events = POLLOUT;              /* 关心"可写"事件 */
                int pret = poll(&pfd, 1, 1000);    /* 最多等 1s */
                /* pret>0 说明有事件；再确认是 POLLOUT（连接就绪）而非出错 */
                if (pret > 0 && (pfd.revents & POLLOUT)) {
                    int sock_err = 0;
                    socklen_t len = sizeof(sock_err);
                    /* 【关键】可写不等于连上！必须用 SO_ERROR 取回"延迟的错误码"，
                     * 为 0 才是真正握手成功（比如对端拒绝连接时这里会是 ECONNREFUSED）。 */
                    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &sock_err, &len) == 0 && sock_err == 0) {
                        mb->connected = 1;
                        mb->err_count = 0;   /* 连上就清零错误计数 */
                        LOG_INFO("Reconnected to %s:%d", mb->host, mb->port);
                        return 0;
                    }
                }
            }
        }
        /* 走到这里说明重连确实失败 */
        LOG_WARN("Reconnect to %s:%d failed: %s",
                 mb->host, mb->port, modbus_strerror(errno));
        modbus_free(mb->ctx);
        mb->ctx = NULL;
        mb->connected = 0;
        return -1;
    }

    mb->connected = 1;
    mb->err_count = 0;
    LOG_INFO("Reconnected to %s:%d", mb->host, mb->port);
    return 0;
}

/* ---- 通用错误处理 ----
 * 【设计思路】把"哪些 errno 代表连接已废"集中判断一次：
 *   超时/连接被重置/管道破裂/连接被拒/未连接 —— 都说明这条连接不能再用，
 *   置 connected=0，让上层下次采集时先重连。
 * 其它错误（如协议层异常）只记日志、不改连接状态（连接本身还活着）。
 * 返回 -1 供调用方直接 return。 */
static int handle_error(ModbusMaster *mb, const char *op)
{
    int err = errno;   /* 【易错点】errno 会被后续调用覆盖，必须先存下来 */

    /* libmodbus 的错误码 */
    if (err == ETIMEDOUT || err == ECONNRESET || err == EPIPE ||
        err == ECONNREFUSED || err == ENOTCONN) {
        LOG_WARN("%s failed: %s, marking disconnected", op, modbus_strerror(err));
        mb->connected = 0;
    } else {
        LOG_ERROR("%s failed: %s", op, modbus_strerror(err));
    }

    mb->err_count++;   /* 无论哪种错误都累加，供上层做"连续错 N 次就断开"判定 */
    return -1;
}

/* ---- 读保持寄存器 (03) ----
 * 这是本程序最核心的一次 Modbus 交互：读下位机的温度/湿度。
 * @param slave 从站地址（下位机网关的 slave_id，配置里默认 1）
 * @param addr  起始寄存器地址（0x0300，见 modbus_map.h）
 * @param count 读取个数（2：温度 + 湿度）
 * @param dest  输出缓冲（至少能放 count 个 uint16_t） */
int Modbus_ReadHoldingRegs(ModbusMaster *mb, int slave,
                           int addr, int count, uint16_t *dest)
{
    /* 前置检查：连接不可用就不必浪费时间发请求 */
    if (!mb->connected || !mb->ctx) {
        LOG_ERROR("ReadHoldingRegs: not connected");
        return -1;
    }

    modbus_set_slave(mb->ctx, slave);   /* 告诉 libmodbus 这次要找哪个从站 */
    int rc = modbus_read_registers(mb->ctx, addr, count, dest);
    /* 【判断要点】libmodbus 成功时返回"实际读到的寄存器个数"，
     * 所以必须用 rc != count 判断，不能只判断 <0。 */
    if (rc != count) {
        return handle_error(mb, "modbus_read_registers");
    }

    mb->err_count = 0;   /* 【重要】一次成功就把连续错误计数清零——这是"连续"二字的体现 */
    return 0;
}

/* ---- 读输入寄存器 (04) ----
 * 与 03 的区别：04 读"输入寄存器"（只读，常用于采集量），03 读"保持寄存器"（可读可写）。
 * 本网关用 03，这里是通用性保留。 */
int Modbus_ReadInputRegs(ModbusMaster *mb, int slave,
                         int addr, int count, uint16_t *dest)
{
    if (!mb->connected || !mb->ctx) {
        LOG_ERROR("ReadInputRegs: not connected");
        return -1;
    }

    modbus_set_slave(mb->ctx, slave);
    int rc = modbus_read_input_registers(mb->ctx, addr, count, dest);
    if (rc != count) {
        return handle_error(mb, "modbus_read_input_registers");
    }

    mb->err_count = 0;
    return 0;
}

/* ---- 写单寄存器 (06) ----
 * 当前业务未用到（网关只做采集），保留作为通用能力。 */
int Modbus_WriteSingleReg(ModbusMaster *mb, int slave,
                          int addr, uint16_t value)
{
    if (!mb->connected || !mb->ctx) {
        LOG_ERROR("WriteSingleReg: not connected");
        return -1;
    }

    modbus_set_slave(mb->ctx, slave);
    int rc = modbus_write_register(mb->ctx, addr, value);
    if (rc != 1) {   /* 写单个寄存器成功返回 1 */
        return handle_error(mb, "modbus_write_register");
    }

    mb->err_count = 0;
    return 0;
}

/* ---- 检查连接状态 ----
 * 同时看两个条件：connected 标志 + ctx 非空。
 * 只信标志位是不够的——可能标志是 1 但 ctx 已经因错误被释放成 NULL。 */
int Modbus_IsConnected(ModbusMaster *mb)
{
    return mb->connected && mb->ctx;
}

/* ---- 获取错误信息 ---- */
const char *Modbus_LastError(ModbusMaster *mb)
{
    (void)mb;   /* 参数未用，显式 void 掉，避免编译告警 */
    return modbus_strerror(errno);
}
