/* ============================================================================
 * ota.c — OTA 固件下发引擎
 *
 * OTA (Over-The-Air)：远程给下位机升级固件。上位机扮演推送方，通过 TCP 506
 * 把 .ota 固件包发给网关 Bootloader；Bootloader 收齐、校验、写 Flash、重启跳转。
 *
 * 用独立 pthread 而非塞进主循环：整个 OTA 最坏几十秒（擦除 + 传输 + 轮询重启），
 * 放主循环会阻塞所有网关的采集。与主循环只通过 g_ota_ctx（受互斥锁保护）交换状态。
 *
 * 协议: START → DATA×N → FINISH (两阶段 ACK)
 *        BUSY: 延时 200ms 重发, 不计重试, 30s 总超时
 *        CANCEL: 1s×2 重试
 * BUSY 表示"设备正在擦除 Flash"，属正常忙等而非错误；若计入"3 次重试"，
 * 一次擦除就可能耗光额度。故 BUSY 用"总时长 30s"兜底：30s 内变空闲则继续。
 * ============================================================================
 */
#include "ota.h"
#include "ota_crc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>        /* 独立线程 + 互斥锁 */
#include <sys/socket.h>
#include <sys/stat.h>       /* stat：检查固件文件是否存在 */
#include <fcntl.h>          /* open/O_RDONLY */
#include <netinet/in.h>     /* sockaddr_in */
#include <arpa/inet.h>      /* inet_pton */
#include <netdb.h>

/* ======================== 全局状态 ======================== */
/* 同一时刻只支持一个 OTA 任务，所以用单个全局上下文即可。
 * 所有对 g_ota_ctx 的访问都必须持有 g_ota_mutex（写线程 + HTTP 读线程并发）。 */
static pthread_t g_ota_thread;
static OTAContext g_ota_ctx;
static pthread_mutex_t g_ota_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ======================== 静函数 ======================== */
/* 前置声明：这些辅助函数定义在文件后面，先声明再使用 */
static void *ota_thread_func(void *arg);
static int tcp_connect_timeout(const char *host, int port, int timeout_ms);
static int send_all(int fd, const void *buf, int len, int timeout_ms);
static int recv_byte(int fd, uint8_t *byte, int timeout_ms);

/* ======================== 公开 API ======================== */

/* 异步启动 OTA：只做"检查 + 准备上下文 + 拉起线程"，立即返回。
 * 真正的传输在后台线程跑，所以 HTTP 接口能马上返回"已开始"。 */
int OTA_StartAsync(int gateway_id, const char *fw_path)
{
    pthread_mutex_lock(&g_ota_mutex);   /* 保护 g_ota_ctx */

    /* 仅拦截运行态 (SENDING/VERIFYING);
     * FAILED / CANCELLED / PENDING_VERIFY 是历史终态,
     * 允许被新 OTA 任务覆盖重启 (下方 memset 会清空旧上下文) */
    if (g_ota_ctx.status == OTA_STATUS_SENDING ||
        g_ota_ctx.status == OTA_STATUS_VERIFYING) {
        pthread_mutex_unlock(&g_ota_mutex);   /* 【易错点】每个 return 前都要解锁，否则死锁 */
        return -1;  /* 已有 OTA 进行中 */
    }

    /* 检查固件文件 */
    struct stat st;
    if (stat(fw_path, &st) != 0) {
        pthread_mutex_unlock(&g_ota_mutex);
        return -2;   /* 文件不存在/无权限 */
    }

    /* 读取 FirmwareHeader
     * .ota 文件结构 = [FirmwareHeader][纯固件数据...]。
     * 先读头拿到 magic/size/crc32/version，用于和 Bootloader 握手校验。 */
    int fd = open(fw_path, O_RDONLY);
    if (fd < 0) {
        pthread_mutex_unlock(&g_ota_mutex);
        return -2;
    }
    FirmwareHeader hdr;
    /* 【易错点】read 可能"短读"（返回小于请求的字节数），必须校验返回值等于 sizeof(hdr)，
     *   不能假设一次就读完。 */
    if (read(fd, &hdr, sizeof(hdr)) != sizeof(hdr)) {
        close(fd);
        pthread_mutex_unlock(&g_ota_mutex);
        return -2;
    }
    close(fd);

    /* 魔数校验：确认这确实是一个合法的 .ota 文件，而不是随便改了扩展名的文件 */
    if (ntohl(hdr.magic) != FW_HEADER_MAGIC) {   /* 网络序转主机序再比较 */
        pthread_mutex_unlock(&g_ota_mutex);
        return -2;
    }

    /* 填充上下文 */
    memset(&g_ota_ctx, 0, sizeof(g_ota_ctx));   /* 清掉上一次任务的残留状态 */
    g_ota_ctx.gateway_id = gateway_id;
    /* TODO: 目标 IP/端口当前硬编码, 未从 config.ini 的 [ota] 段读取 */
    snprintf(g_ota_ctx.host, sizeof(g_ota_ctx.host), "%s", "192.168.10.100");
    g_ota_ctx.port = 506;
    snprintf(g_ota_ctx.fw_path, sizeof(g_ota_ctx.fw_path), "%s", fw_path);
    g_ota_ctx.fw_size = ntohl(hdr.size);         /* 头部字段都是网络字节序 */
    g_ota_ctx.fw_crc32 = ntohl(hdr.crc32);
    g_ota_ctx.fw_version = ntohl(hdr.version);
    g_ota_ctx.status = OTA_STATUS_SENDING;
    /* 计算总块数：每块 512 字节，向上取整。
     * (size + 511) / 512 是整数运算里"向上取整"的经典写法
     * （等价于 ceil(size/512.0)，但避免了浮点）。 */
    g_ota_ctx.block_total = (g_ota_ctx.fw_size + 511) / 512;

    pthread_mutex_unlock(&g_ota_mutex);

    /* 启动独立线程（默认属性即可：可 join 但这里不 join，属于"分离式"用法的简化处理） */
    pthread_create(&g_ota_thread, NULL, ota_thread_func, NULL);

    return 0;
}

/* 查询 OTA 状态：把全局上下文拷贝一份给调用方（HTTP 线程用）。
 * 【为什么拷贝而不是返回指针？】返回指针会让锁的保护失效（调用方拿到指针后
 *   可能在没有锁的情况下读，与写线程竞争）。拷贝则天然是一份快照，安全。 */
int OTA_GetStatus(int gateway_id, OTAContext *out)
{
    pthread_mutex_lock(&g_ota_mutex);
    if (out) memcpy(out, &g_ota_ctx, sizeof(OTAContext));   /* 锁内拷贝，保证快照一致 */
    int has_data = (g_ota_ctx.status != OTA_STATUS_IDLE) ? 0 : -1;
    pthread_mutex_unlock(&g_ota_mutex);
    return has_data;
}

/* 取消进行中的 OTA：只改状态标志，真正的线程检测到后会自行退出。
 * 这是"协作式取消"的典型做法，比 pthread_cancel（可能在线程持锁时被强杀）安全得多。 */
int OTA_Cancel(int gateway_id)
{
    pthread_mutex_lock(&g_ota_mutex);
    if (g_ota_ctx.status == OTA_STATUS_IDLE ||
        g_ota_ctx.status == OTA_STATUS_SUCCESS) {
        pthread_mutex_unlock(&g_ota_mutex);
        return -1;   /* 没有进行中的任务，取消失败 */
    }
    g_ota_ctx.status = OTA_STATUS_CANCELLED;   /* 只是打个"请取消"的标记 */
    pthread_mutex_unlock(&g_ota_mutex);
    return 0;
}

/* ======================== 发送线程 ======================== */
/* 这个函数在独立线程里跑，负责完整的 OTA 流程。
 * 【注意】它访问 g_ota_ctx 时并没有全程持锁（因为整个流程要几十秒，
 *   持锁会让 HTTP 查询状态被卡死）。这里采用的是"关键字段单次写入"的宽松策略：
 *   状态字段的写入是原子的（int 赋值），足以让查询方看到大致进展。 */
static void *ota_thread_func(void *arg)
{
    (void)arg;   /* 未使用线程参数 */

    /* 1. 打开固件文件 (跳过 Header) */
    int fw_fd = open(g_ota_ctx.fw_path, O_RDONLY);
    if (fw_fd < 0) {
        g_ota_ctx.status = OTA_STATUS_FAILED;
        snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                 "Cannot open firmware file");
        return NULL;   /* 线程函数返回 NULL 即结束线程 */
    }
    /* 用 lseek 跳到 Header 之后：后面读到的就是"纯固件数据"，不含头 */
    lseek(fw_fd, sizeof(FirmwareHeader), SEEK_SET);  /* 跳过 Header */

    /* 2. TCP 连接（带 5s 超时，避免连接不上时无限等待） */
    int sock = tcp_connect_timeout(g_ota_ctx.host, g_ota_ctx.port, 5000);
    if (sock < 0) {
        close(fw_fd);    /* 出错路径要关掉已打开的文件描述符 */
        g_ota_ctx.status = OTA_STATUS_FAILED;
        snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                 "Connection failed");
        return NULL;
    }

    /* 3. 发 START */
    OTACmd start;
    start.magic = HTONL(OTA_CMD_MAGIC);   /* 【关键】所有多字节字段都要转网络字节序(大端) */
    start.cmd = CMD_START;
    start.reserved = 0;
    /* 版本号只取低 16 位（协议字段是 uint16_t 的网络序） */
    start.fw_version = HTONS((uint16_t)(g_ota_ctx.fw_version & 0xFFFF));
    start.total_size = HTONL(g_ota_ctx.fw_size);
    start.total_crc = HTONL(g_ota_ctx.fw_crc32);

    if (send_all(sock, &start, sizeof(start), 3000) != 0) {
        goto FAIL;   /* 统一走 FAIL 清理路径，避免到处复制 close 逻辑 */
    }
    uint8_t ack;
    /* 设备端 START 现在同步擦除暂存区 (典型 <1s, 最坏 ~4.4s) 后才回 ACK */
    /* 【等待时间为什么给 10s？】因为设备收到 START 后要擦除 Flash 暂存区，
     *   最坏约 4.4s。若超时给太短会误判失败，故留足到 10s。 */
    if (recv_byte(sock, &ack, 10000) != 0 || ack != RSP_ACK) {
        snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                 "START failed (ack=0x%02X)", ack);
        goto FAIL;
    }
    printf("[OTA] START accepted\n");

    /* 4. 发 DATA 包 */
    uint8_t buf[512];              /* 待发送的数据缓冲（一个块 512 字节） */
    int busy_total_ms = 0;         /* BUSY 累计等待时长（跨块累计到 30s 为止） */

    for (int i = 0; i < g_ota_ctx.block_total; i++) {
        int nread = read(fw_fd, buf, 512);
        if (nread <= 0) break;     /* 文件读完（最后一块可能不足 512）或读错 */

        /* 每块单独算 CRC16，设备用它校验这一块是否传对 */
        uint16_t crc = CRC16_Modbus(buf, nread);

        /* 组装 DATA 包
         * 【内存布局】magic(4) + block_num(4) + data_len(2) + block_crc(2) + data(N)
         *   = 12 字节头 + N 字节数据，与下位机 Bootloader 的结构体定义必须严格一致。 */
        uint8_t pkt_buf[12 + 512];
        OTAData *pkt = (OTAData *)pkt_buf;   /* 把字节数组按结构体解释，方便逐字段赋值 */
        pkt->magic     = HTONL(OTA_DATA_MAGIC);
        pkt->block_num = HTONL(i);           /* 块序号从 0 开始 */
        pkt->data_len  = HTONS((uint16_t)nread);
        pkt->block_crc = HTONS(crc);
        memcpy(pkt->data, buf, nread);
        int pkt_len = 12 + nread;            /* 实际要发送的总长度 */

        int retry_ok = 0;                    /* 本块是否最终成功 */
        for (int retry = 0; retry < 3; retry++) {
            /* ---- 取消检查：每轮重试前都看一眼有没有被取消 ---- */
            if (g_ota_ctx.status == OTA_STATUS_CANCELLED) {
                close(sock); close(fw_fd);
                g_ota_ctx.status = OTA_STATUS_IDLE;  /* 取消后回归空闲, 允许再次升级 */
                return NULL;
            }

            if (send_all(sock, pkt_buf, pkt_len, 3000) != 0) continue;   /* 发送失败 → 消耗一次重试 */

            uint8_t ack;
            if (recv_byte(sock, &ack, 3000) != 0) continue;              /* 接收超时 → 消耗一次重试 */

            if (ack == RSP_ACK) {          /* 正常确认 → 本块完成 */
                busy_total_ms = 0;         /* 成功就清零 BUSY 累计 */
                retry_ok = 1;
                break;
            }

            if (ack == RSP_BUSY) {
                /* 设备正忙（擦除中）：等 200ms 再试，且【不消耗重试次数】 */
                usleep(200000);            /* 200ms = 200000 微秒 */
                busy_total_ms += 200;
                if (busy_total_ms >= OTA_BUSY_TOTAL_MS) {   /* 累计超过 30s → 判定失败 */
                    snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                             "BUSY timeout (30s) at block %d", i);
                    goto FAIL;
                }
                retry--;  /* 不计重试 */   /* 【关键技巧】先减一次，抵消 for 的 retry++ */
                continue;
            }

            if (ack == RSP_ERR_ERASE || ack == RSP_ERR_WRITE || ack == RSP_ERR_OOB) {
                /* 设备端硬件级错误（擦除失败/写入失败/越界），重试也没用 → 直接失败 */
                snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                         "Device error 0x%02X at block %d", ack, i);
                goto FAIL;
            }

            /* NACK 或其他 → 消耗重试 */
        }

        if (!retry_ok) {   /* 3 次重试用尽仍未成功 */
            snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                     "Block %d: retry exhausted", i);
            goto FAIL;
        }

        /* 更新进度（供 HTTP 接口轮询显示进度条） */
        g_ota_ctx.block_sent = i + 1;
        g_ota_ctx.progress_pct = (i + 1) * 100 / g_ota_ctx.block_total;
    }

    close(fw_fd);   /* 数据发完，固件文件不再需要 */

    /* 5. 发 FINISH (3 次重试, 6s 超时)
     * FINISH 是"收尾握手"：告诉设备"数据发完了，请校验整体 CRC32 并写正式区"。
     * 因为是最后的确认，等待时间给到 6s（设备要做整体校验）。 */
    printf("[OTA] Sending FINISH...\n");
    OTACmd finish;
    finish.magic = HTONL(OTA_CMD_MAGIC);
    finish.cmd = CMD_FINISH;
    finish.reserved = 0;
    finish.fw_version = 0;                     /* FINISH 不带版本 */
    finish.total_size = 0;                     /* FINISH 不带大小 */
    finish.total_crc = HTONL(g_ota_ctx.fw_crc32);   /* 带上整体 CRC32，供设备比对 */

    int finish_ok = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (send_all(sock, &finish, sizeof(finish), 3000) != 0) continue;
        if (recv_byte(sock, &ack, 6000) == 0 && ack == RSP_ACK) {
            finish_ok = 1;
            break;
        }
    }

    close(sock);   /* 传输完成，连接可以关了 */

    if (!finish_ok) {
        snprintf(g_ota_ctx.err_msg, sizeof(g_ota_ctx.err_msg),
                 "FINISH failed after 3 retries");
        g_ota_ctx.status = OTA_STATUS_FAILED;
        return NULL;
    }

    /* 6. 轮询网关重启上线
     * 收到 FINISH ACK 后，设备会重启并跳转到新固件。
     * 这里通过"能否连上它的 502（Modbus 服务端口）"来判断它是否已经正常工作。
     * 最长等 15×2s = 30s。 */
    printf("[OTA] FINISH ACK received, waiting for reboot...\n");
    int online = 0;
    for (int i = 0; i < 15; i++) {
        sleep(2);                                       /* 每次间隔 2s */
        int test = tcp_connect_timeout(g_ota_ctx.host, 502, 1000);
        if (test >= 0) {                                /* 连上即认为启动成功 */
            close(test);                                /* 探测连接要马上关掉，别占着 */
            online = 1;
            break;
        }
    }

    if (online) {
        g_ota_ctx.status = OTA_STATUS_SUCCESS;          /* 完整成功 */
        printf("[OTA] Gateway online, upgrade SUCCESS\n");
    } else {
        /* 【设计考虑】"升级完成但没确认上线"≠失败：新固件可能启动稍慢，
         *   或网络暂时不通。所以单独给一个 PENDING_VERIFY 状态，
         *   让运维后续人工确认，而不是简单判失败。 */
        g_ota_ctx.status = OTA_STATUS_PENDING_VERIFY;
        printf("[OTA] Upgrade done, but gateway not confirmed online\n");
    }

    return NULL;

/* ---- 统一失败清理路径 ---- */
FAIL:
    close(sock);
    close(fw_fd);
    g_ota_ctx.status = OTA_STATUS_FAILED;
    printf("[OTA] FAILED: %s\n", g_ota_ctx.err_msg);
    return NULL;
}

/* ======================== TCP 辅助 ======================== */

/* 带超时的 TCP 连接。
 * 【要点】用 setsockopt 设置 SO_RCVTIMEO/SO_SNDTIMEO 给后续读写加超时；
 *   连接本身用阻塞式 connect（所以超时靠系统默认，这里主要保证读写不无限等）。 */
static int tcp_connect_timeout(const char *host, int port, int timeout_ms)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    /* 把毫秒拆成 timeval 的"秒 + 微秒"两部分 */
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000   /* 余数毫秒 → 微秒 */
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));   /* 接收超时 */
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));   /* 发送超时 */

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);   /* 端口要转网络序 */

    /* 把点分十进制的 IP 字符串转成二进制（inet_pton 是"新"接口，可重入；
     * 旧的 inet_addr 用静态缓冲区，多线程不安全） */
    if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        close(fd);
        return -1;
    }

    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);   /* 连不上要关 fd，否则每次都泄漏一个 */
        return -1;
    }

    return fd;
}

/* 可靠发送：循环写直到发完。
 * 【易错点】write 可能只写一部分（TCP 是流、发送缓冲满时会短写），
 *   必须循环推进指针，直到 remain==0。一次性 write 后就认为发完是常见 bug。 */
static int send_all(int fd, const void *buf, int len, int timeout_ms)
{
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));   /* 本次发送的超时 */

    const uint8_t *p = (const uint8_t *)buf;
    int remain = len;
    while (remain > 0) {
        int n = write(fd, p, remain);
        if (n <= 0) return -1;   /* 出错（含超时）直接返回失败 */
        p += n;                  /* 指针后移已发送部分 */
        remain -= n;             /* 剩余待发字节数减少 */
    }
    return 0;
}

/* 收 1 字节（OTA 协议的响应都是单字节 ACK/BUSY/ERR） */
static int recv_byte(int fd, uint8_t *byte, int timeout_ms)
{
    struct timeval tv = {
        .tv_sec = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));   /* 本次接收的超时 */

    return (read(fd, byte, 1) == 1) ? 0 : -1;   /* 必须真读到 1 字节才算成功 */
}
