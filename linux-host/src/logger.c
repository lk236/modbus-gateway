/*
 * logger.c — 日志系统实现
 *
 * 自写而非 syslog/printf: 需要终端彩色 + 文件落盘同时输出、需要源码定位
 * (宏 + __FILE__/__LINE__/__func__ 自动带上)、需要运行时动态调级。
 *
 * 特性: 线程安全(pthread_mutex 保护); 文件日志自动创建目录; ANSI 彩色终端输出;
 * 每次写日志都 flush(防程序崩溃时丢掉最后一段关键日志)。
 */
#include "logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>      /* va_list/va_start/vsnprintf */
#include <time.h>        /* localtime */
#include <pthread.h>     /* pthread_mutex */
#include <unistd.h>
#include <sys/time.h>    /* gettimeofday: 取毫秒级时间 */
#include <sys/stat.h>    /* mkdir/stat/S_ISDIR */
#include <sys/types.h>
#include <errno.h>

/* ---- 内部状态(模块级静态变量, 只在本文件可见, 相当于单例日志器) ---- */
static FILE     *g_log_fp      = NULL;   /* 日志文件指针(NULL = 不写文件) */
static LogLevel  g_log_level   = LOG_LEVEL_INFO;   /* 当前最低输出级别 */
static bool      g_color       = true;             /* 终端是否加 ANSI 颜色 */
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;  /* 静态初始化, 无需手动 init */

/* ---- 级别名称和颜色 ----
 * 用"指定初始化器"([下标] = 值) 把枚举值直接当数组下标, 查表不用 switch,
 * 也不怕枚举顺序与数组顺序不一致写错(C99 特性)。 */
static const char *s_level_str[] = {
    [LOG_LEVEL_DEBUG] = "DEBUG",
    [LOG_LEVEL_INFO]  = "INFO",
    [LOG_LEVEL_WARN]  = "WARN",
    [LOG_LEVEL_ERROR] = "ERROR",
};

static const char *s_color_code[] = {
    [LOG_LEVEL_DEBUG] = "\033[37m",    /* 白(\033 是 ESC, [37m 是白字 ANSI 序列) */
    [LOG_LEVEL_INFO]  = "\033[32m",    /* 绿 */
    [LOG_LEVEL_WARN]  = "\033[33m",    /* 黄 */
    [LOG_LEVEL_ERROR] = "\033[31m",    /* 红 */
};

#define COLOR_RESET "\033[0m"   /* 颜色复位, 否则后续终端输出都会被染色 */

/* 取当前时间字符串。用 gettimeofday 而非 time() 是为了拿到毫秒 */
static void get_timestamp(char *buf, size_t len)
{
    struct timeval tv;
    struct tm *tm;

    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);   /* 非线程安全, 但本函数在锁内调用 */

    /* tm_year 从 1900 起算, tm_mon 是 0~11, 都要修正 */
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
             tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec,
             tv.tv_usec / 1000);   /* 微秒 → 毫秒 */
}

/* 确保目录存在(写 "data/xxx.log" 时 data/ 不存在就得先建)。
 * 实现为递归创建多级目录。 */
static int ensure_dir(const char *path)
{
    char dir[512];
    const char *p = strrchr(path, '/');   /* 最后一个 '/' 左边就是目录部分 */
    if (p == NULL) return 0;  /* 无目录部分(如 "app.log") */

    size_t len = p - path;
    if (len >= sizeof(dir)) return -1;   /* 防 memcpy 溢出 */

    memcpy(dir, path, len);
    dir[len] = '\0';

    /* 目录已存在且是目录 → 成功; 存在但是普通文件 → 报错 */
    struct stat st;
    if (stat(dir, &st) == 0) {
        if (S_ISDIR(st.st_mode)) return 0;
        errno = ENOTDIR;
        return -1;
    }

    /* 递归创建: 从左到右每遇到 '/' 就截断并建一级, 如 "a/b/c" 依次 mkdir("a")、
     * mkdir("a/b")、mkdir("a/b/c")。已存在的那几级会返回 EEXIST, 故意忽略。 */
    char *q;
    for (q = dir + 1; *q; q++) {   /* 从 dir+1 开始, 跳过首字符(可能是 '/') */
        if (*q == '/') {
            *q = '\0';             /* 临时截断 */
            mkdir(dir, 0755);      /* 0755 = rwxr-xr-x */
            *q = '/';              /* 还原继续扫描 */
        }
    }
    return mkdir(dir, 0755);       /* 最后一级也要建 */
}

/* 初始化。filepath 为 NULL 或空串: 只输出到终端;
 * 否则同时写文件(追加模式 "a", 重启不覆盖历史日志)。 */
int LOG_Init(const char *filepath, LogLevel level, bool color)
{
    g_log_level = level;
    g_color     = color;

    if (filepath && filepath[0] != '\0') {
        /* 先建目录; EEXIST 是正常的(目录本来就在), 不算错误 */
        if (ensure_dir(filepath) != 0 && errno != EEXIST) {
            fprintf(stderr, "[LOG] Failed to create log dir: %s\n", strerror(errno));
        }

        g_log_fp = fopen(filepath, "a");   /* "a" = append */
        if (g_log_fp == NULL) {
            fprintf(stderr, "[LOG] Failed to open log file '%s': %s\n",
                    filepath, strerror(errno));
            return -1;
        }
        setbuf(g_log_fp, NULL);  /* 关用户态缓冲, 每条直接 write 到内核 → 崩溃不丢日志 */
    }

    /* 打印"日志系统就绪"作自检: 能出来说明级别过滤和格式化链路都通了。
     * 此时 g_log_fp 可能为 NULL, LOG_Write 内部已判空。 */
    LOG_INFO("=== Log system initialized, level=%s file=%s color=%d ===",
             s_level_str[level], filepath ? filepath : "(console)", color);
    return 0;
}

/* 运行时调整级别: 热加载配置时会调用 */
void LOG_SetLevel(LogLevel level)
{
    g_log_level = level;
}

/* 写日志核心。可变参数函数: fmt 之后跟任意多个参数, 由 vsnprintf 按格式串解释;
 * 所有 LOG_XXX 宏最终都走到这里。 */
void LOG_Write(LogLevel level, const char *file, int line,
               const char *func, const char *fmt, ...)
{
    /* 级别不够直接返回。放最前面避免无谓的时间戳/格式化, 级别过滤是最频繁的判断 */
    if (level < g_log_level) return;

    char timestamp[64];
    get_timestamp(timestamp, sizeof(timestamp));

    /* 构造消息体（单条最大 2KB, 超出被 vsnprintf 安全截断） */
    char msg[2048];
    va_list args;
    va_start(args, fmt);                          /* 从 fmt 之后开始取可变参数 */
    vsnprintf(msg, sizeof(msg), fmt, args);       /* "v" 前缀版本才能接受 va_list */
    va_end(args);                                 /* 与 va_start 配对, 必须调用 */

    /* 文件名简写(去掉路径), 只保留 xxx.c, 日志更紧凑 */
    const char *short_file = strrchr(file, '/');
    short_file = short_file ? short_file + 1 : file;

    /* 加锁: 采集线程和 HTTP 线程都会写日志, 不加锁会出现两行日志交错的乱码 */
    pthread_mutex_lock(&g_lock);

    /* 终端输出(带颜色)。"%-5s" 左对齐占 5 格, 让级别名对齐 */
    if (g_color) {
        fprintf(stdout, "%s %s%-5s%s [%s:%d %s] %s\n",
                timestamp,
                s_color_code[level], s_level_str[level], COLOR_RESET,
                short_file, line, func, msg);
    } else {
        fprintf(stdout, "%s %-5s [%s:%d %s] %s\n",
                timestamp, s_level_str[level],
                short_file, line, func, msg);
    }
    fflush(stdout);   /* 立即刷出, 保证日志顺序与真实发生顺序一致 */

    /* 文件输出(无颜色): 文件里不能带 ANSI 转义序列, 否则 cat/less 看全是乱码 */
    if (g_log_fp) {
        fprintf(g_log_fp, "%s %-5s [%s:%d %s] %s\n",
                timestamp, s_level_str[level],
                short_file, line, func, msg);
        fflush(g_log_fp);   /* 配合 setbuf(NULL), 确保每条都落盘 */
    }

    pthread_mutex_unlock(&g_lock);
}

void LOG_Close(void)
{
    if (g_log_fp) {
        LOG_INFO("=== Log system shutting down ===");   /* 关之前留一条"正常关闭" */
        fclose(g_log_fp);
        g_log_fp = NULL;   /* 置空, 防后续误写已关闭的 FILE* */
    }
}
