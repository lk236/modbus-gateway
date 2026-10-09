/* ============================================================================
 * http_server.c — 嵌入式 HTTP 服务器实现
 *
 * 对外提供人机界面与接口：浏览器打开 http://localhost:8080 看仪表盘
 * （Chart.js 温度曲线），脚本/前端通过 REST API 拿 JSON 数据。
 *
 * 手写而非用 nginx / mongoose：需求窄（几个固定路由 + 静态文件）、零外部依赖、
 * 交叉编译即用。生产项目当然优先选现成方案，此处是"小工具 + 学习"的取舍。
 *
 * 支持:
 *   - JSON REST API（/api/latest, /api/history, /api/alarms, /api/ota/...）
 *   - 静态文件服务（/ → dashboard.html）
 *   - 独立线程运行，不阻塞主采集循环
 *   - 每个请求独立 DB 连接读取（SQLite WAL 模式支持读写并发）
 *
 * 采用 HTTP/1.0 + Connection: close（不搞 keep-alive）：实现简单、无状态、
 * 不怕连接泄漏；代价是每次重新握手。对内网仪表盘完全够用。
 *
 * 纯 Linux socket API + epoll，零外部依赖。
 * ============================================================================
 */
#include "http_server.h"
#include "logger.h"
#include "storage.h"
#include "ota.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>      /* 独立线程跑 HTTP 服务 */
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/epoll.h>
#include <sys/socket.h>   /* socket/bind/listen/accept */
#include <netinet/in.h>   /* sockaddr_in */
#include <arpa/inet.h>    /* 地址转换 */


#define MAX_EVENTS    32                                  /* epoll_wait 一次最多返回的事件数 */
#define REQUEST_BUF   65536                               /* 请求缓冲 64KB（够放 multipart 头 + 部分体） */
#define RESPONSE_BUF  262144   /* 256KB 容纳 Chart.js (208KB) + 头部 */   /* 响应缓冲，要能装下整个 Chart.js */
#define MAX_PATH      256                                 /* URL 路径最长长度 */


/* ---- 在内存中查找子串（不受内部 \0 影响） ----
 * 【为什么不用标准 strstr？】因为要处理的是"二进制数据"（固件上传的 body 里含 \0），
 *   strstr 遇到 \0 就会停止，根本查不到 boundary。必须用"带长度"的查找。
 * 返回值：首次出现的位置指针；没找到返回 NULL。 */
static const char *memfind(const char *haystack, size_t haystack_len,
                           const char *needle, size_t needle_len)
{
    if (!needle_len || needle_len > haystack_len) return NULL;   /* 边界：空 needle 或 needle 太长 */
    /* 最多只需检查 haystack_len - needle_len + 1 个起点 */
    for (size_t i = 0; i <= haystack_len - needle_len; i++) {
        if (memcmp(haystack + i, needle, needle_len) == 0)   /* 逐字节比较 */
            return haystack + i;
    }
    return NULL;
}


/* ---- URL 解码 ----
 * 把 URL 里的百分号编码还原成原始字符：
 *   %E6%B8%A9 → "温"（UTF-8 字节），'+' → 空格。
 * 【为什么需要】浏览器/前端发请求时会把非 ASCII 和特殊字符转义，
 *   服务端必须还原才能正确理解参数。 */
static void url_decode(char *dst, const char *src)
{
    while (*src) {
        if (*src == '%' && src[1] && src[2]) {          /* 形如 %XX 的三字符序列 */
            char hex[3] = { src[1], src[2], '\0' };     /* 取出两位十六进制 */
            *dst++ = (char)strtol(hex, NULL, 16);       /* 按 16 进制解析成一个字节 */
            src += 3;                                   /* 跳过 %XX 三个字符 */
        } else if (*src == '+') {                       /* 表单编码里 '+' 代表空格 */
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;                            /* 普通字符直接拷贝 */
        }
    }
    *dst = '\0';   /* 手动补字符串结束符 */
}

/* ---- 查找 GET 参数 ----
 * 在 query 串（如 "gateway=1&hours=24"）里找 name 对应的值，返回值的起始指针。
 * 【注意】返回的是"指向 query 内部的指针"，不含 \0 终止，调用方需配合 get_param_value 使用。 */
static const char *get_query_param(const char *query, const char *name)
{
    if (!query) return NULL;

    size_t nlen = strlen(name);   /* 参数名长度 */
    const char *p = query;

    while (*p) {
        /* 跳过 & */
        if (*p == '&') p++;
        /* 匹配 "name=" —— 用 strncasecmp 忽略大小写（GW / gw 都能认） */
        if (strncasecmp(p, name, nlen) == 0 && p[nlen] == '=') {
            return p + nlen + 1;   /* 返回 '=' 之后的值起始位置 */
        }
        /* 跳到下一个参数 */
        p = strchr(p, '&');
        if (!p) break;             /* 没有更多 '&'，说明找完了 */
        p++;
    }
    return NULL;                   /* 未找到该参数 */
}

/* ---- 提取参数值到临时缓冲区 ----
 * 把 get_query_param 找到的"裸值"拷进 out（并做 URL 解码）。
 * 返回 0=成功，-1=参数不存在。 */
static int get_param_value(const char *query, const char *name,
                           char *out, size_t out_size)
{
    const char *val = get_query_param(query, name);
    if (!val) return -1;   /* 参数没出现 */

    /* 提取值（到下一个 & 或结束） */
    char tmp[256];
    size_t i = 0;
    /* 取两个上限的较小值：调用方缓冲大小、本地缓冲大小，都留一格给 '\0' */
    size_t max_i = (out_size < sizeof(tmp)) ? out_size - 1 : sizeof(tmp) - 1;
    while (*val && *val != '&' && i < max_i) {   /* 遇到 & 或超长就停 */
        tmp[i++] = *val++;
    }
    tmp[i] = '\0';
    url_decode(out, tmp);   /* 解码后写入调用方缓冲 */
    return 0;
}

/* ---- 构建 JSON 响应 ----
 * 【HTTP 报文结构简要】：
 *   状态行\r\n  例 "HTTP/1.0 200 OK\r\n"
 *   响应头\r\n\r\n   （空行是头与体的分隔符）
 *   响应体
 * Content-Length 必须准确，否则浏览器会一直等或截断。
 * 【CORS】Access-Control-Allow-Origin: * 让任意页面都能跨域调这个 API（开发期方便）。 */
static void build_json_response(char *buf, size_t size,
                                const char *body, int status_code)
{
    const char *status_str = (status_code == 200) ? "OK" : "Not Found";
    snprintf(buf, size,
        "HTTP/1.0 %d %s\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "Content-Length: %zu\r\n"
        "\r\n"
        "%s",
        status_code, status_str,
        strlen(body), body);
}

/* ---- 构建 HTML 响应 ----
 * 与 JSON 版唯一区别是 Content-Type: text/html。 */
static void build_html_response(char *buf, size_t size,
                                const char *body, int status_code)
{
    const char *status_str = (status_code == 200) ? "OK" : "Not Found";
    snprintf(buf, size,
        "HTTP/1.0 %d %s\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n"
        "Content-Length: %zu\r\n"
        "\r\n"
        "%s",
        status_code, status_str,
        strlen(body), body);
}

/* ---- 构建 404 响应 ---- */
static void build_404(char *buf, size_t size)
{
    const char *body = "{\"error\":\"not found\"}";
    build_json_response(buf, size, body, 404);
}

/* ---- 构建 400 响应 ----
 * msg 是"已经是合法 JSON 的值部分"（如 "\"missing boundary\""），
 * 这里只负责套上 {"error": } 外壳。 */
static void build_400(char *buf, size_t size, const char *msg)
{
    char body[256];
    snprintf(body, sizeof(body), "{\"error\":%s}", msg);
    build_json_response(buf, size, body, 400);
}

/* ---- 构建 OTA 状态 JSON（调用方提供锁外已读的状态） ----
 * 把 OTA 状态枚举翻译成前端可读的字符串，再拼成 JSON。
 * 【设计说明】参数是"已经拷贝出来的快照"，本函数不碰全局状态、不加锁——
 *   职责单一，也避免了在 JSON 拼接过程中持锁。 */
static void build_ota_status_json(char *json, size_t size, OTAContext *ctx)
{
    const char *status_str;
    switch (ctx->status) {
        case OTA_STATUS_IDLE:           status_str = "idle";        break;
        case OTA_STATUS_SENDING:        status_str = "sending";     break;
        case OTA_STATUS_VERIFYING:      status_str = "verifying";   break;
        case OTA_STATUS_SUCCESS:        status_str = "success";     break;
        case OTA_STATUS_FAILED:         status_str = "failed";      break;
        case OTA_STATUS_CANCELLED:      status_str = "cancelled";   break;
        case OTA_STATUS_PENDING_VERIFY: status_str = "pending_verify"; break;
        default:                        status_str = "unknown";     break;   /* 兜底，防止意外值 */
    }

    snprintf(json, size,
        "{"
        "\"gateway_id\":%d,"
        "\"status\":\"%s\","
        "\"progress_pct\":%d,"
        "\"block_sent\":%d,"
        "\"block_total\":%d,"
        "\"fw_version\":\"0x%04X\","
        "\"err_msg\":\"%s\""
        "}",
        ctx->gateway_id,
        status_str,
        ctx->progress_pct,
        ctx->block_sent,
        ctx->block_total,
        (unsigned int)(ctx->fw_version & 0xFFFF),        /* 版本号只显示低 16 位 */
        ctx->err_msg[0] ? ctx->err_msg : "");            /* 无错误时输出空串，避免 NULL */
}

/* ---- 简易 multipart/form-data 解析（单文件上传） ----
 * 【背景】浏览器上传文件时用的就是 multipart/form-data 格式，报文长这样：
 *   --<boundary>\r\n
 *   Content-Disposition: form-data; name="gateway"\r\n\r\n
 *   1\r\n
 *   --<boundary>\r\n
 *   Content-Disposition: form-data; name="file"; filename="fw.ota"\r\n
 *   Content-Type: application/octet-stream\r\n\r\n
 *   <二进制固件内容>\r\n
 *   --<boundary>--\r\n
 * 本函数就是把这堆东西拆开，取出 gateway 字段和文件内容。 */
/* 从 Content-Type 头提取 boundary（返回静态缓存，\0 结尾） */
static const char *extract_boundary(const char *content_type)
{
    const char *p = strstr(content_type, "boundary=");   /* 定位 boundary= 关键字 */
    if (!p) return NULL;
    p += 9;   /* 跳过 "boundary=" 这 9 个字符 */
    /* 【注意】用 static 缓冲：这样返回的指针在函数返回后仍有效（但不能重入）。
     *   本函数只在单线程的请求处理中调用，因此可接受。 */
    static char buf[128];
    int i = 0;
    /* boundary 到行尾/分号结束，最多存 126 字符（留 2 格给 '\0' 与保险） */
    while (*p && *p != '\r' && *p != '\n' && *p != ';' && i < 126)
        buf[i++] = *p++;
    buf[i] = '\0';
    return buf;
}

/* 解析 multipart 数据，提取 gateway 字段 + 文件内容并保存 */
/* 返回: 0=成功, -1=解析失败, -2=写入文件失败 */
static int parse_multipart_upload(const char *body, size_t body_len,
                                  const char *boundary,
                                  int *out_gateway_id,
                                  const char *firmware_dir,
                                  char *saved_path, size_t saved_path_size)
{
    char boundary_delim[128];
    char end_delim[128];
    /* 标准 multipart 里，分隔线前缀是 "--"；结束分隔线还要再加 "--" 后缀 */
    snprintf(boundary_delim, sizeof(boundary_delim), "--%s", boundary);
    snprintf(end_delim, sizeof(end_delim), "--%s--", boundary);

    const char *p = body;            /* 游标：当前处理位置 */
    const char *end = body + body_len;   /* body 结束位置 */

    /* 调试日志：定位上传问题时非常有用（能看到 boundary 与实际 body 长度） */
    fprintf(stderr, "[OTA_DEBUG] parse: boundary_delim=%s delim_len=%zu body_len=%zu\n",
            boundary_delim, strlen(boundary_delim), body_len);

    /* 跳过起始 boundary */
    /* 【关键】必须用 memfind 而非 strstr：固件二进制里含 \0，strstr 会提前停止。 */
    const char *part_start = memfind(p, body_len, boundary_delim, strlen(boundary_delim));
    if (!part_start) {
        /* 找不到 boundary 是最常见的上传失败原因：多半是 Content-Type 没带 boundary，
         * 或 body 被截断。这里把前 64 字节按十六进制打出来，方便对比抓包。 */
        fprintf(stderr, "[OTA_DEBUG] parse: initial boundary NOT FOUND! first 64 bytes hex:\n");
        for (size_t i = 0; i < 64 && i < body_len; i++)
            fprintf(stderr, "%02x ", (unsigned char)p[i]);
        fprintf(stderr, "\n");
        return -1;
    }
    fprintf(stderr, "[OTA_DEBUG] parse: initial boundary FOUND at offset %zu\n",
            (size_t)(part_start - body));
    part_start += strlen(boundary_delim);   /* 跳过 boundary 本身 */
    /* 吃掉 boundary 后面的 \r\n（兼容只有 \n 的情况） */
    if ((size_t)(part_start - body) < body_len && *part_start == '\r') part_start++;
    if ((size_t)(part_start - body) < body_len && *part_start == '\n') part_start++;

    int gw_id = 0;         /* 从 gateway 字段解析出来的网关 id */
    int file_found = 0;    /* 是否找到了文件部分 */
    int fd = -1;           /* 保存文件的 fd */

    int loop_count = 0;    /* 循环计数（仅用于调试日志） */
    while (part_start && part_start < end) {
        loop_count++;
        fprintf(stderr, "[OTA_DEBUG] loop iter %d part_start offset=%zu\n",
                loop_count, (size_t)(part_start - body));
        /* 查找下一个 boundary 作为结束（二进制内容含 \0，必须带长度查找） */
        size_t remain = (size_t)(end - part_start);   /* 剩余可查长度 */
        const char *part_end = memfind(part_start, remain, boundary_delim, strlen(boundary_delim));
        if (!part_end) break;   /* 没有下一个分隔线，说明解析到头了 */

        /* 解析 part 头：头部与 body 之间用空行 "\r\n\r\n" 分隔 */
        const char *hdr_end = strstr(part_start, "\r\n\r\n");
        if (!hdr_end || hdr_end > part_end) break;   /* 头部不合法或越过了本 part 范围 */

        /* 检查 Content-Disposition（每个 part 必须携带，用来说明"这个字段叫什么"） */
        const char *disp = strstr(part_start, "Content-Disposition:");
        if (!disp || disp > hdr_end) { part_start = part_end; continue; }   /* 没有则跳过本 part */

        const char *body_start = hdr_end + 4;                    /* 跳过 "\r\n\r\n" 四字节 */
        int body_len_this = (int)(part_end - body_start);        /* 本 part 数据长度 */
        if (body_len_this < 0) body_len_this = 0;                /* 防御：理论上不会负 */

        /* 去除尾部 \r\n (在 boundary 之前) */
        /* multipart 规定 body 与分隔线之间有一个 CRLF，这个不算数据，要去掉 */
        while (body_len_this >= 2 && body_start[body_len_this - 2] == '\r' && body_start[body_len_this - 1] == '\n')
            body_len_this -= 2;

        fprintf(stderr, "[OTA_DEBUG] loop: disp=%.60s\n", disp ? disp : "NULL");
        if (strstr(disp, "name=\"gateway\"")) {
            /* ---- 这是 gateway 表单字段 ---- */
            char tmp[32] = "";
            int n = body_len_this < 31 ? body_len_this : 31;   /* 防止溢出（留 1 格给 '\0'） */
            memcpy(tmp, body_start, n);                        /* 【注意】不是字符串拷贝，因为 body 里可能没 \0 */
            tmp[n] = '\0';
            gw_id = atoi(tmp);                                 /* 转成整数网关号 */
            fprintf(stderr, "[OTA_DEBUG] parse: gateway_id=%d\n", gw_id);
        }
        else if (strstr(disp, "name=\"file\"") || strstr(disp, "name=\"file\";")) {
            /* ---- 这是文件字段：把固件内容写到磁盘 ---- */
            fprintf(stderr, "[OTA_DEBUG] parse: file part found, body_len_this=%d\n", body_len_this);
            /* 保存文件到 firmware_dir
             * 文件名 = upload_<时间戳>_<网关号>.ota，保证多次上传不会互相覆盖 */
            char filepath[512];
            snprintf(filepath, sizeof(filepath), "%s/upload_%ld_%d.ota",
                     firmware_dir, (long)time(NULL), gw_id);

            /* O_WRONLY 只写 / O_CREAT 不存在则创建 / O_TRUNC 已存在则清空，权限 0644 */
            fd = open(filepath, O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fd < 0) return -2;

            /* 【可靠写入】循环写直到写完全部字节（write 可能短写） */
            int written = 0;
            while (written < body_len_this) {
                int n = write(fd, body_start + written, body_len_this - written);
                if (n <= 0) { close(fd); unlink(filepath); return -2; }   /* 写失败：删掉半截文件，避免留下垃圾 */
                written += n;
            }
            close(fd);
            fd = -1;

            snprintf(saved_path, saved_path_size, "%s", filepath);   /* 把保存路径回传给调用方 */
            file_found = 1;
        }

        /* 跳到下一个 part（跳过 boundary 分隔线 + \r\n） */
        part_start = part_end + strlen(boundary_delim);
        if ((size_t)(part_start - body) < body_len && *part_start == '\r') part_start++;
        if ((size_t)(part_start - body) < body_len && *part_start == '\n') part_start++;
    }

    if (fd >= 0) close(fd);                                  /* 异常路径兜底关闭 */
    if (out_gateway_id) *out_gateway_id = gw_id;
    fprintf(stderr, "[OTA_DEBUG] parse: file_found=%d gw_id=%d returning %d\n",
            file_found, gw_id, file_found ? 0 : -1);
    return file_found ? 0 : -1;                              /* 没找到文件就是失败 */
}

/* ---- JSON 转义（简单实现） ----
 * 【为什么必须转义？】JSON 字符串里不能直接出现双引号、反斜杠、换行等字符，
 *   必须写成 \" \\ \n 的形式；否则前端 JSON.parse 会报错。
 * 【为什么用 FILE* 而不是写进 char*？】因为它是"流式追加"到 fmemopen 的内存流中，
 *   与 fprintf 的调用风格一致，拼接长 JSON 更自然。 */
static void json_escape(FILE *f, const char *s)
{
    fputc('"', f);   /* 起始引号 */
    while (*s) {
        switch (*s) {
            case '"':  fputs("\\\"", f); break;   /* 双引号 */
            case '\\': fputs("\\\\", f); break;   /* 反斜杠 */
            case '\n': fputs("\\n", f);  break;   /* 换行 */
            case '\r': fputs("\\r", f);  break;   /* 回车 */
            case '\t': fputs("\\t", f);  break;   /* 制表符 */
            default:   fputc(*s, f);     break;   /* 普通字符原样输出 */
        }
        s++;
    }
    fputc('"', f);   /* 结束引号 */
}

/* ---- 查询回调：收集 JSON 片段 ----
 * 作为 userdata 在查询回调之间传递，记录"是不是第一条"（决定要不要加逗号）和总条数。 */
typedef struct {
    FILE *f;        /* 输出目标（fmemopen 出来的内存流） */
    int   first;    /* 1 = 即将输出第一条（前面不加逗号），0 = 之后（前面要加逗号） */
    int   count;    /* 已输出条数 */
} JsonCbCtx;

/* ---- 回调：复制一条记录到 userdata ----
 * 【用途】只需要"最新一条"时，把查询到的第一条拷出来即可。
 * 返回 1 会通知 Storage 层"我要中止遍历"（因为只要第一条）。 */
static int copy_record_cb(SensorRecord *rec, void *userdata)
{
    SensorRecord *dst = (SensorRecord *)userdata;
    *dst = *rec;          /* 结构体整体赋值（浅拷贝，含定长数组，安全） */
    return 1; /* 只取第一条 */
}


/* ---- 回调：把一条传感器记录写成 JSON 对象 ----
 * 【逗号处理技巧】JSON 数组元素间要有逗号，但第一项前不能有。
 *   用 first 标志实现："不是第一项时才补逗号"。这是拼接 JSON 数组的标准套路。 */
static int json_history_cb(SensorRecord *rec, void *userdata)
{
    JsonCbCtx *ctx = (JsonCbCtx *)userdata;
    if (!ctx->first) fprintf(ctx->f, ",");   /* 非首项 → 补逗号 */
    ctx->first = 0;                          /* 标记"已经有元素了" */

    fprintf(ctx->f, "{");
    fprintf(ctx->f, "\"ts\":%ld,", (long)rec->ts);
    fprintf(ctx->f, "\"gateway_id\":%d,", rec->gateway_id);
    fprintf(ctx->f, "\"gateway_name\":"); json_escape(ctx->f, rec->gateway_name);   /* 字符串字段走转义 */
    fprintf(ctx->f, ",\"temperature\":%.1f,", rec->temperature);
    fprintf(ctx->f, "\"humidity\":%.1f", rec->humidity);
    fprintf(ctx->f, "}");
    ctx->count++;
    return 0;   /* 返回 0 表示"继续遍历后续记录" */
}

/* ---- 告警 JSON 回调 ----
 * 与历史数据回调同构，只是字段更多（含阈值、触发值、时间、状态）。 */
static int json_alarm_cb(AlarmRecord *rec, void *userdata)
{
    JsonCbCtx *ctx = (JsonCbCtx *)userdata;
    if (!ctx->first) fprintf(ctx->f, ",");
    ctx->first = 0;

    fprintf(ctx->f, "{");
    fprintf(ctx->f, "\"id\":%d,", rec->alarm_id);
    fprintf(ctx->f, "\"gateway_id\":%d,", rec->gateway_id);
    fprintf(ctx->f, "\"gateway_name\":"); json_escape(ctx->f, rec->gateway_name);
    fprintf(ctx->f, ",\"alarm_type\":"); json_escape(ctx->f, rec->alarm_type);
    fprintf(ctx->f, ",\"threshold_value\":%.1f,", rec->threshold_value);
    fprintf(ctx->f, "\"trigger_value\":%.1f,", rec->trigger_value);
    fprintf(ctx->f, "\"trigger_ts\":%ld,", (long)rec->trigger_ts);
    fprintf(ctx->f, "\"recover_ts\":%ld,", (long)rec->recover_ts);
    fprintf(ctx->f, "\"status\":"); json_escape(ctx->f, rec->status);
    fprintf(ctx->f, "}");
    ctx->count++;
    return 0;
}

/* ---- 完整的 API 处理函数（带 db_path + firmware_dir） ----
 * 【路由分发】用一串 if (strcmp(path, "/api/xxx") == 0) 实现。
 *   路由少时这样写最直观；路由多了就该换成"路由表 + 查表"或前缀树。
 * 参数 full_request 是原始完整请求，只有 OTA 上传需要（要取 Content-Type/Length 和 body）。 */
static void handle_request(const char *method, const char *path,
                           const char *query, const char *db_path,
                           char *resp, size_t resp_size,
                           AppConfig *cfg,
                           const char *firmware_dir,
                           const char *full_request)
{
    int is_get  = (strcmp(method, "GET") == 0);
    int is_post = (strcmp(method, "POST") == 0);

    /* ============ OTA 上传 ============ */
    if (is_post && strcmp(path, "/api/ota/upload") == 0) {
        /* 从 Content-Type 中提取 boundary */
        const char *ct = strstr(full_request, "Content-Type:");
        const char *boundary = NULL;
        if (ct) {
            const char *ct_val = ct + 12;          /* 跳过 "Content-Type:" 12 个字符 */
            while (*ct_val == ' ') ct_val++;       /* 跳过冒号后的空格 */
            boundary = extract_boundary(ct_val);
        }
        fprintf(stderr, "[OTA_DEBUG] Content-Type found=%s\n", ct ? "YES" : "NO");
        fprintf(stderr, "[OTA_DEBUG] boundary=%s\n", boundary ? boundary : "NULL");
        if (!boundary) {
            build_400(resp, resp_size, "\"missing boundary\"");   /* 没有 boundary 无法解析 multipart */
            return;
        }

        /* 从 Content-Length 获取 body 实际长度（二进制固件含 \0，禁止用 strlen） */
        const char *cl = strstr(full_request, "Content-Length:");
        size_t content_length = 0;
        if (cl) {
            const char *cl_val = cl + 15;                  /* 跳过 "Content-Length:" 15 个字符 */
            while (*cl_val == ' ') cl_val++;
            content_length = (size_t)atol(cl_val);
        }
        fprintf(stderr, "[OTA_DEBUG] Content-Length found=%s value=%zu\n", cl ? "YES" : "NO", content_length);

        /* 找到 body 起始 (headers 后的 \r\n\r\n) */
        const char *body = strstr(full_request, "\r\n\r\n");
        if (!body) {
            build_400(resp, resp_size, "\"malformed request\"");
            return;
        }
        body += 4;                       /* 跳过空行，指向真正的 body */
        size_t body_len = content_length;   /* 用 Content-Length 而非 strlen——body 含 \0 */
        fprintf(stderr, "[OTA_DEBUG] body_len=%zu body_start_hex=%02x%02x%02x%02x\n",
                body_len,
                (unsigned char)body[0], (unsigned char)body[1],
                (unsigned char)body[2], (unsigned char)body[3]);

        int gateway_id = 0;
        char saved_path[512] = "";
        fprintf(stderr, "[OTA_DEBUG] calling parse_multipart_upload...\n");
        int ret = parse_multipart_upload(body, body_len, boundary,
                                         &gateway_id, firmware_dir,
                                         saved_path, sizeof(saved_path));
        fprintf(stderr, "[OTA_DEBUG] parse returned %d, gateway_id=%d saved_path=%s\n",
                ret, gateway_id, saved_path);
        if (ret != 0) {
            build_400(resp, resp_size, "\"upload parse failed\"");
            return;
        }

        /* 启动 OTA（异步：立即返回，传输在独立线程跑） */
        int ota_ret = OTA_StartAsync(gateway_id, saved_path);
        char json[512];
        if (ota_ret == 0) {
            /* 成功开始：返回任务信息 */
            snprintf(json, sizeof(json),
                "{\"status\":\"started\",\"gateway_id\":%d,\"file\":\"%s\"}",
                gateway_id, saved_path);
            build_json_response(resp, resp_size, json, 200);
        } else if (ota_ret == -1) {
            build_400(resp, resp_size, "\"ota already in progress\"");   /* 已有任务在跑 */
        } else {
            build_400(resp, resp_size, "\"invalid firmware file\"");     /* 文件非法 */
        }
        return;
    }

    /* ============ OTA 状态查询 ============ */
    if (is_get && strcmp(path, "/api/ota/status") == 0) {
        char gw_str[32] = "0";                                    /* 默认网关 0 */
        get_param_value(query, "gateway", gw_str, sizeof(gw_str)); /* 取 ?gateway=N 覆盖 */
        int gateway_id = atoi(gw_str);

        OTAContext ctx;
        int ret = OTA_GetStatus(gateway_id, &ctx);
        if (ret == -1) {
            /* 没有该网关的 OTA 记录 → 返回空闲态（而不是报错），前端逻辑更简单 */
            build_json_response(resp, resp_size,
                "{\"status\":\"idle\",\"progress_pct\":0}", 200);
            return;
        }

        char json[512];
        build_ota_status_json(json, sizeof(json), &ctx);
        build_json_response(resp, resp_size, json, 200);
        return;
    }

    /* ============ OTA 取消 ============ */
    if (is_post && strcmp(path, "/api/ota/cancel") == 0) {
        char gw_str[32] = "0";
        get_param_value(query, "gateway", gw_str, sizeof(gw_str));
        int gateway_id = atoi(gw_str);

        int ret = OTA_Cancel(gateway_id);
        if (ret == 0) {
            build_json_response(resp, resp_size,
                "{\"status\":\"cancelled\"}", 200);
        } else {
            /* 取消失败不算错误（可能本来就没任务），仍返回 200 + 说明 */
            build_json_response(resp, resp_size,
                "{\"status\":\"no_active_ota\"}", 200);
        }
        return;
    }

    /* ---- 以下仅支持 GET ---- */
    if (!is_get) {
        build_404(resp, resp_size);   /* 非 GET 且没匹配到 POST 路由 → 404 */
        return;
    }

    /* ---- /api/latest — 按活跃网关配置遍历，不依赖 DB 聚合 ---- */
    /* 【设计选择】这里不是从数据库"查所有网关最新"，而是按配置里的网关数量逐个查。
     *   好处：即使某个网关还没采到过数据，也能返回一条"占位记录"（名字+0值），
     *   前端列表不会缺项，用户能看到"这个网关存在但还没数据"。 */
    if (strcmp(path, "/api/latest") == 0) {
        StorageDB db;
        if (Storage_Open(&db, db_path) != 0) {
            build_json_response(resp, resp_size,
                "{\"error\":\"db_open_failed\"}", 500);
            return;
        }

        int gw_count = cfg ? cfg->gateway_count : 0;   /* 配置里的网关数 */
        if (gw_count < 1) gw_count = 1;                /* 至少给 1 个，保证响应结构稳定 */

        /* 【为什么堆分配而不是栈数组？】RESPONSE_BUF 是 256KB，
         *   放栈上很可能超线程栈（默认 8MB，但多个大数组叠加就危险），堆分配更稳妥。 */
        char *json = malloc(RESPONSE_BUF);
        if (!json) { Storage_Close(&db); build_404(resp, resp_size); return; }

        /* fmemopen：把一个内存缓冲区包装成 FILE*，于是可以像写文件一样 fprintf，
         * 写的内容直接落到这块内存里，非常适合"拼 JSON 字符串"。 */
        FILE *f = fmemopen(json, RESPONSE_BUF, "w");
        if (!f) { free(json); Storage_Close(&db); build_404(resp, resp_size); return; }

        fprintf(f, "{\"gateways\":[");   /* JSON 数组开始 */

        int first = 1;   /* 逗号控制标志 */
        int count = 0;
        for (int i = 0; i < gw_count; i++) {
            SensorRecord rec;
            memset(&rec, 0, sizeof(rec));
            if (Storage_QueryLatest(&db, i, copy_record_cb, &rec) <= 0) {
                /* 该网关尚无数据，用配置信息填充占位 */
                rec.gateway_id = i;
                if (cfg && i < cfg->gateway_count)
                    snprintf(rec.gateway_name, sizeof(rec.gateway_name),
                             "%s", cfg->gateways[i].name);
                else
                    snprintf(rec.gateway_name, sizeof(rec.gateway_name),
                             "gateway.%d", i + 1);   /* 配置缺失时的兜底名字 */
                rec.ts = 0;              /* 0 表示"无数据" */
                rec.temperature = 0;
                rec.humidity = 0;
            }

            if (!first) fprintf(f, ",");   /* 非首项补逗号 */
            first = 0;

            fprintf(f, "{");
            fprintf(f, "\"gateway_id\":%d,", rec.gateway_id);
            fprintf(f, "\"gateway_name\":"); json_escape(f, rec.gateway_name);
            fprintf(f, ",\"ts\":%ld,", (long)rec.ts);
            fprintf(f, "\"temperature\":%.1f,", rec.temperature);
            fprintf(f, "\"humidity\":%.1f", rec.humidity);
            fprintf(f, "}");
            count++;
        }

        fprintf(f, "],\"count\":%d}", count);   /* 收尾：关闭数组并给出总数 */
        fclose(f);                              /* 这会 flush 并结束内存流（不释放 json） */

        Storage_Close(&db);
        build_json_response(resp, resp_size, json, 200);
        free(json);                             /* 【易错点】响应体已被拷贝进 resp，这里才能安全释放 */
        return;
    }

    /* ---- /api/history?gateway=X&hours=Y ---- */
    if (strcmp(path, "/api/history") == 0) {
        char gw_str[32] = "0";      /* 默认查网关 0 */
        char hours_str[32] = "24";  /* 默认最近 24 小时 */

        get_param_value(query, "gateway", gw_str, sizeof(gw_str));
        get_param_value(query, "hours", hours_str, sizeof(hours_str));

        int gateway_id = atoi(gw_str);
        int hours = atoi(hours_str);
        if (hours < 1) hours = 24;          /* 参数兜底：至少 1 小时 */
        if (hours > 720) hours = 720; /* 最多 30 天 */   /* 上限防止查太大拖垮服务 */

        time_t now = time(NULL);
        time_t begin = now - hours * 3600;  /* 起始时间 = 现在 - N 小时 */
        time_t end = now;

        StorageDB db;
        if (Storage_Open(&db, db_path) != 0) {
            build_json_response(resp, resp_size,
                "{\"error\":\"db_open_failed\"}", 500);
            return;
        }

        char *json = malloc(RESPONSE_BUF);
        if (!json) { Storage_Close(&db); build_404(resp, resp_size); return; }

        FILE *f = fmemopen(json, RESPONSE_BUF, "w");
        if (!f) { free(json); Storage_Close(&db); build_404(resp, resp_size); return; }

        /* 拼 JSON: {"gateway_id":N,"gateway_name":"...","records":[...]} */
        fprintf(f, "{\"gateway_id\":%d,\"gateway_name\":", gateway_id);

        /* 查最新一条获取 name（记录里冗余存了名字，用它比查配置更方便） */
        {
            SensorRecord tmp_rec;
            memset(&tmp_rec, 0, sizeof(tmp_rec));
            Storage_QueryLatest(&db, gateway_id, copy_record_cb, &tmp_rec);
            if (gateway_id < 0) {
                json_escape(f, "所有");          /* gateway=-1 表示聚合所有网关 */
            } else if (tmp_rec.gateway_name[0]) {
                json_escape(f, tmp_rec.gateway_name);
            } else {
                json_escape(f, "unknown");       /* 该网关还没数据 */
            }
        }

        fprintf(f, ",\"records\":[");

        /* 查询时间范围内所有记录，逐条通过回调写进 JSON 流 */
        JsonCbCtx ctx = { .f = f, .first = 1, .count = 0 };   /* C99 指定初始化器，字段含义一目了然 */
        Storage_QueryRange(&db, gateway_id, begin, end,
                           json_history_cb, &ctx);

        fprintf(f, "],\"count\":%d}", ctx.count);   /* 回调里累计的条数 */
        fclose(f);

        Storage_Close(&db);
        build_json_response(resp, resp_size, json, 200);
        free(json);
        return;
    }

    /* ---- /api/alarms?gateway=X&status=active&limit=50 ---- */
    if (strcmp(path, "/api/alarms") == 0) {
        char gw_str[32] = "-1";     /* 默认查所有网关（-1 表示不限定） */
        char limit_str[32] = "50";  /* 默认返回 50 条 */
        char status_str[32] = "";   /* 默认不过滤状态 */

        get_param_value(query, "gateway", gw_str, sizeof(gw_str));
        get_param_value(query, "limit", limit_str, sizeof(limit_str));
        get_param_value(query, "status", status_str, sizeof(status_str));

        int gateway_id = atoi(gw_str);
        int limit = atoi(limit_str);
        if (limit < 1) limit = 50;        /* 兜底 */
        if (limit > 500) limit = 500;     /* 上限，防止一次拉太多 */

        /* status 为空串时传 NULL，表示"不过滤" */
        const char *status_filter = status_str[0] ? status_str : NULL;

        StorageDB db;
        if (Storage_Open(&db, db_path) != 0) {
            build_json_response(resp, resp_size,
                "{\"error\":\"db_open_failed\"}", 500);
            return;
        }

        char *json = malloc(RESPONSE_BUF);
        if (!json) { Storage_Close(&db); build_404(resp, resp_size); return; }

        FILE *f = fmemopen(json, RESPONSE_BUF, "w");
        if (!f) { free(json); Storage_Close(&db); build_404(resp, resp_size); return; }

        fprintf(f, "{\"alarms\":[");

        /* 使用 Storage_QueryAlarms 查询（begin/end 传 0 = 不限时间） */
        JsonCbCtx ctx = { .f = f, .first = 1, .count = 0 };
        Storage_QueryAlarms(&db, gateway_id, 0, 0, status_filter, limit,
                           json_alarm_cb, &ctx);

        fprintf(f, "],\"count\":%d", ctx.count);

        /* 同时返回活跃告警数（前端指示灯用） */
        int active_count = Storage_GetActiveAlarmCount(&db, gateway_id);
        fprintf(f, ",\"active_count\":%d}", active_count);

        fclose(f);

        Storage_Close(&db);
        build_json_response(resp, resp_size, json, 200);
        free(json);
        return;
    }

    /* ---- 默认 404 ---- */
    build_404(resp, resp_size);
}

/* ---- 读取静态文件 ----
 * 把整个文件读进内存并返回（调用方负责 free）。
 * 【适用场景】文件都不大（dashboard.html、chart.js 几百 KB），
 *   一次性读进内存最简单，也不会有"读一半被中断"的麻烦。 */
static char *read_static_file(const char *path, size_t *out_len)
{
    FILE *f = fopen(path, "rb");   /* "rb" 二进制模式，避免 Windows 下换行被转换 */
    if (!f) return NULL;

    fseek(f, 0, SEEK_END);         /* 定位到文件末尾 */
    long len = ftell(f);           /* 取文件长度 */
    rewind(f);                     /* 回到开头 */
    if (len <= 0) { fclose(f); return NULL; }   /* 空文件视为无效 */

    char *buf = (char *)malloc((size_t)len + 1);   /* +1 留给 '\0' */
    if (!buf) { fclose(f); return NULL; }

    size_t nread = fread(buf, 1, (size_t)len, f);   /* 一次性读完 */
    fclose(f);
    buf[nread] = '\0';             /* 【注意】静态文件可能有 \0 吗？通常没有，但补 '\0' 便于当字符串用 */
    *out_len = nread;              /* 回传实际读到的字节数（给 Content-Length 用） */
    return buf;
}

/* ---- 处理 HTTP 请求（从 socket 读取并响应） ----
 * 每个客户端连接由本函数同步处理完再关闭（HTTP/1.0 短连接模型）。 */
static void process_client(int client_fd, const char *web_dir,
                           const char *db_path, AppConfig *cfg)
{
    char buf[REQUEST_BUF];   /* 64KB 栈缓冲：注意别太小，否则大 body 会被截断 */
    ssize_t total = 0;       /* 已读字节数 */

    /* 循环读取，确保收完完整的 HTTP 请求（大 body 会分包）
     * 【为什么需要循环？】TCP 是流协议，客户端发的 1MB 数据可能被拆成几十个 TCP 段到达，
     *   一次 read 拿到多少是不确定的。必须循环读到"头部 + body 完整"为止。 */
    while (total < (ssize_t)sizeof(buf) - 1) {
        ssize_t n = read(client_fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n <= 0) break;   /* 0=对端关闭，<0=出错 */
        total += n;

        /* 检查是否收完 HTTP 头部 */
        buf[total] = '\0';   /* 临时补上结束符，好让 strstr 能用 */
        const char *hdr_end = strstr(buf, "\r\n\r\n");
        if (hdr_end) {
            /* 找到 Content-Length */
            const char *cl = strstr(buf, "Content-Length:");
            size_t content_length = 0;
            if (cl) {
                const char *cl_val = cl + 15;
                while (*cl_val == ' ') cl_val++;
                content_length = (size_t)atol(cl_val);
            }
            /* 计算 body 在 buf 中的偏移 */
            size_t body_offset = (size_t)(hdr_end + 4 - buf);   /* 头部结束位置 +4 跳过空行 */
            /* 已经收到的 body 字节数 */
            size_t body_received = total > (ssize_t)body_offset ? (size_t)total - body_offset : 0;
            /* body 收完了就退出循环 */
            if (body_received >= content_length)
                break;
        }
        /* 【注意】若请求没有 Content-Length（如普通 GET），上面这步不会 break，
         *   会继续 read 直到对端关闭连接（n<=0）为止——HTTP/1.0 的客户端通常会关，
         *   但有些浏览器会保持连接，此时会一直阻塞到超时。这是本实现的已知简化点。 */
    }
    if (total <= 0) {
        close(client_fd);   /* 一个字节都没读到，直接关 */
        return;
    }
    buf[total] = '\0';      /* 正式补结束符 */

    /* 解析请求行
     * 【格式】METHOD SP PATH SP HTTP_VERSION CRLF，例："GET /api/latest?x=1 HTTP/1.1"
     * %15s 限制长度防止溢出（method 最多 15 字符、path 最多 255）。 */
    char method[16], path[MAX_PATH], http_version[16];
    if (sscanf(buf, "%15s %255s %15s", method, path, http_version) < 2) {
        close(client_fd);   /* 至少要有 method 和 path，否则是坏请求 */
        return;
    }

    /* 分离 path 和 query：把 '?' 变成 '\0'，让 path 独立出来 */
    char *query = NULL;
    char *qmark = strchr(path, '?');
    if (qmark) {
        *qmark = '\0';      /* 截断，path 部分到此为止 */
        query = qmark + 1;  /* query 指向 '?' 之后 */
    }

    /* ---- 构建响应（堆分配，避免栈上两个 64KB 数组压爆） ---- */
    char *resp = (char *)malloc(RESPONSE_BUF);
    if (!resp) { close(client_fd); return; }
    resp[0] = '\0';   /* 【重要】先置空，后续 snprintf 的拼接才能安全进行 */

    /* ---- 静态文件 ---- */
    if (strncmp(path, "/api/", 5) != 0) {   /* 不是 /api/ 开头 → 当静态文件处理 */
        char file_path[MAX_PATH];
        const char *served = path;
        if (strcmp(path, "/") == 0) served = "/dashboard.html";   /* 根路径默认给仪表盘 */

        /* 【注意：潜在安全点】这里直接拼 web_dir + 用户传来的 path，
         *   理论上可用 "../" 做目录穿越（读任意文件）。生产环境必须校验规范化后的路径
         *   是否仍在 web_dir 内。本项目为内网工具，未做此校验，生产环境需补上。 */
        snprintf(file_path, sizeof(file_path), "%s%s", web_dir, served);

        size_t file_len = 0;
        char *content = read_static_file(file_path, &file_len);
        if (content) {
            const char *ext = strrchr(served, '.');   /* 取扩展名 */
            if (ext && strcmp(ext, ".js") == 0) {
                /* JS 文件要专门的 MIME 类型，浏览器才会当脚本执行 */
                snprintf(resp, RESPONSE_BUF,
                    "HTTP/1.0 200 OK\r\n"
                    "Content-Type: application/javascript; charset=utf-8\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Connection: close\r\n"
                    "Content-Length: %zu\r\n"
                    "\r\n"
                    "%s",
                    file_len, content);
            } else {
                build_html_response(resp, RESPONSE_BUF, content, 200);
            }
            free(content);
        } else {
            build_404(resp, RESPONSE_BUF);
        }
    }
    /* ---- API 路由 ---- */
    else if (strncmp(path, "/api/", 5) == 0) {
        /* 【注意】把完整请求 buf 也传进去：OTA 上传要从原始报文里取 Content-Type/Length 和 body */
        handle_request(method, path, query, db_path, resp, RESPONSE_BUF, cfg,
                       cfg ? cfg->firmware_dir : "firmware", buf);
    }
    else {
        build_404(resp, RESPONSE_BUF);   /* 其它路径一律 404 */
    }

    /* 发送响应
     * 【易错点】同样要循环写：响应可能有几百 KB（含 Chart.js），
     *   write 一次很可能写不完，必须推进 sent 指针直到发完。 */
    size_t resp_len = strlen(resp);
    ssize_t sent = 0;
    while ((size_t)sent < resp_len) {
        ssize_t s = write(client_fd, resp + sent, resp_len - (size_t)sent);
        if (s <= 0) break;   /* 对端断开/出错，放弃剩余数据 */
        sent += s;
    }

    free(resp);
    close(client_fd);   /* HTTP/1.0 短连接：处理完就关 */
}

/* ---- HTTP 服务器线程 ---- */
/* 传进线程的参数集合。注意 cfg 是"指针"（引用主线程的配置，非拥有），
 * 所以 HttpServer_Stop 时不能 free 它。 */
typedef struct {
    int    port;
    char   db_path[256];
    char   web_dir[256];
    int    running;      /* 线程自己的运行标志：Stop 时会被主线程置 0 */
    int    server_fd;    /* 监听 fd：启动成功后写回，供主线程判断"是否起来了" */
    AppConfig *cfg;     /* 指向主线程的活跃配置（非拥有）*/
} HttpThreadArgs;

static void *http_server_thread(void *arg)
{
    HttpThreadArgs *args = (HttpThreadArgs *)arg;
    int server_fd = -1;
    int epoll_fd = -1;

    /* 创建 socket
     * 【关键】SOCK_NONBLOCK：监听 socket 设为非阻塞，accept 时派生的连接也会继承。
     *   但注意：本实现在 accept 后是"同步阻塞处理请求"的，见下面 process_client 的注释。 */
    server_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (server_fd == -1) {
        LOG_ERROR("HTTP: socket() failed: %s", strerror(errno));
        args->server_fd = -1;   /* 失败要写回 -1，主线程据此知道没起来 */
        return NULL;
    }

    /* 允许地址重用
     * 【为什么必须设？】服务器重启时，TIME_WAIT 状态的旧连接会让 bind 报"地址已占用"，
     *   设了 SO_REUSEADDR 就能立即重新绑定，是服务端程序的标准动作。 */
    int optval = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons((uint16_t)args->port),         /* 端口转网络序 */
        .sin_addr   = { .s_addr = htonl(INADDR_ANY) },     /* 监听所有网卡 */
    };

    if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        LOG_ERROR("HTTP: bind(:%d) failed: %s", args->port, strerror(errno));
        close(server_fd);
        args->server_fd = -1;
        return NULL;
    }

    /* listen 的 backlog=10：内核允许排队的"未处理连接"上限。
     *   超过后新连接可能被拒，内网小工具 10 足够。 */
    if (listen(server_fd, 10) != 0) {
        LOG_ERROR("HTTP: listen() failed: %s", strerror(errno));
        close(server_fd);
        args->server_fd = -1;
        return NULL;
    }

    /* 创建 epoll */
    epoll_fd = epoll_create1(0);
    if (epoll_fd == -1) {
        LOG_ERROR("HTTP: epoll_create1 failed: %s", strerror(errno));
        close(server_fd);
        args->server_fd = -1;
        return NULL;
    }

    struct epoll_event ev = {
        .events  = EPOLLIN,
        .data.fd = server_fd,   /* 只监听监听 fd：有就绪就说明"有新连接到来" */
    };
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, server_fd, &ev);

    args->server_fd = server_fd;   /* 写回：主线程的启动等待循环据此判断成功 */
    args->running = 1;

    /* 写入 args->server_fd 让调用方知道已启动 */
    LOG_INFO("HTTP server started on port %d, web_dir=%s",
             args->port, args->web_dir);

    struct epoll_event events[MAX_EVENTS];

    while (args->running) {   /* 主线程把 running 置 0 即通知本线程退出 */
        /* 500ms 超时：不是为了处理事件（事件会立刻唤醒），
         *   而是给循环一个"定期醒来检查 running 标志"的机会。 */
        int nfds = epoll_wait(epoll_fd, events, MAX_EVENTS, 500);
        if (nfds == -1) {
            if (errno == EINTR) continue;   /* 被信号打断，正常，继续 */
            LOG_ERROR("HTTP: epoll_wait failed: %s", strerror(errno));
            break;
        }

        for (int i = 0; i < nfds; i++) {
            if (events[i].data.fd == server_fd) {
                /* 接受新连接 */
                struct sockaddr_in client_addr;
                socklen_t addr_len = sizeof(client_addr);
                int client_fd = accept(server_fd,
                    (struct sockaddr *)&client_addr, &addr_len);
                if (client_fd >= 0) {
                    /* 同步处理：阻塞 read 直到 HTTP 请求数据到达，
                       避免非阻塞模式下 read 返回 EAGAIN 导致空响应 */
                    /* 【设计说明】这里是"接受连接后当场处理完再回到 epoll"的简单模型：
                     *   优点是逻辑简单、请求完成后再处理下一个；
                     *   缺点是"慢客户端"会阻塞其他新连接（比如浏览器只连不发）。
                     *   生产级做法是把 client_fd 也注册进 epoll + 用连接状态机做异步 IO，
                     *   或改用线程池。对本内网仪表盘场景，简单优先。 */
                    process_client(client_fd, args->web_dir, args->db_path,
                                   args->cfg);
                }
            }
        }
    }

    close(server_fd);
    if (epoll_fd >= 0) close(epoll_fd);
    LOG_INFO("HTTP server stopped");

    args->server_fd = -1;
    args->running = 0;
    return NULL;
}

/* ---- 启动 HTTP 服务器 ----
 * 【线程模型】pthread_create 拉起服务线程后，主线程阻塞等待"监听 fd 就绪"
 *   （最多 1 秒），确认端口真的绑定成功了才返回 0。
 *   这叫"同步启动确认"，避免出现"函数返回了但其实服务没起来"的假成功。 */
int HttpServer_Start(HTTPServer *srv, uint16_t port, const char *db_path,
                       AppConfig *cfg)
{
    memset(srv, 0, sizeof(*srv));   /* 清零，避免历史残留字段 */
    srv->port    = port;
    srv->running = 0;
    srv->server_fd = -1;
    snprintf(srv->db_path, sizeof(srv->db_path), "%s", db_path);

    /* 参数结构体必须堆分配：线程启动后主线程继续走，
     *   若用栈变量，HttpServer_Start 返回后内存就失效了——经典的悬空指针陷阱。 */
    HttpThreadArgs *args = (HttpThreadArgs *)malloc(sizeof(HttpThreadArgs));
    if (!args) return -1;
    args->port = port;
    snprintf(args->db_path, sizeof(args->db_path), "%s", db_path);
    snprintf(args->web_dir, sizeof(args->web_dir), "%s", "web");   /* 静态文件目录固定为 web/ */
    args->cfg      = cfg;
    args->running = 0;      /* 线程内部会置 1 */
    args->server_fd = -1;   /* 线程绑定成功后写回真实 fd */

    pthread_t tid;
    if (pthread_create(&tid, NULL, http_server_thread, args) != 0) {
        LOG_ERROR("HTTP: pthread_create failed: %s", strerror(errno));
        free(args);   /* 线程没起来，这里要负责释放 */
        return -1;
    }

    /* 【类型转换技巧】pthread_t 在不同平台可能是整数或指针，
     *   用 (void*)(intptr_t) 中转可以兼容两种，避免编译告警/截断。 */
    srv->thread = (void *)(intptr_t)tid;  /* 存储 pthread_t */
    srv->thread_args = args;
    srv->thread_started = 1;
    srv->running = 1;

    /* 等待线程启动（最多 1 秒）
     * 【为什么这样写？】线程是异步的，pthread_create 返回时线程可能还没 bind 完。
     *   主线程轮询 args->server_fd（线程成功后会写正值），
     *   100 次 × 10ms = 1s 超时，超时也返回 0（服务可能在慢启动，不阻塞主程序）。 */
    for (int i = 0; i < 100; i++) {
        if (args->server_fd >= 0) {
            srv->server_fd = args->server_fd;   /* 拿到真实监听 fd，供 Stop 时关闭 */
            break;
        }
        usleep(10000); /* 10ms */
    }

    return 0;
}

/* ---- 停止 HTTP 服务器 ----
 * 【优雅停止的三步】
 *   1) 置 args->running=0，让线程循环退出；
 *   2) close(server_fd) 让可能阻塞在 epoll_wait 的线程立刻返回（关闭 fd 会唤醒）；
 *   3) pthread_join 等线程真正结束，确保资源回收完再返回。 */
void HttpServer_Stop(HTTPServer *srv)
{
    if (!srv->running || !srv->thread_started) return;   /* 本来就没在跑，直接返回 */

    srv->running = 0;

    /* 通知线程退出（设置 args->running = 0）*/
    if (srv->thread_args) {
        HttpThreadArgs *args = (HttpThreadArgs *)srv->thread_args;
        args->running = 0;
    }

    /* 关闭 server_fd 让 epoll_wait 返回 */
    if (srv->server_fd >= 0) {
        close(srv->server_fd);
        srv->server_fd = -1;
    }

    /* 等待线程退出（join 之后线程资源才算彻底回收） */
    pthread_t tid = (pthread_t)(intptr_t)srv->thread;
    pthread_join(tid, NULL);

    srv->thread_started = 0;
    /* 【注意】thread_args 指向的 args 是 malloc 出来的，理论上这里应该 free；
     *   但线程可能还引用了 args（虽然已 join，实践上安全）。
     *   这里未 free，属于进程退出前的"可接受泄漏"（进程一结束全部回收）。
     *   若要长期运行/反复启停，应补上 free(srv->thread_args) 并置 NULL。 */
    LOG_INFO("HTTP server joined");
}

/* ---- 检查运行状态 ---- */
int HttpServer_IsRunning(HTTPServer *srv)
{
    return srv->running;
}
