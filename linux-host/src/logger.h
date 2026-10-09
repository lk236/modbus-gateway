#ifndef __LOGGER_H
#define __LOGGER_H

/*
 * logger.h — 日志系统接口
 *
 * 提供分级日志输出，支持：
 *   - 终端 ANSI 彩色输出
 *   - 文件日志（轮转由外部 logrotate 完成, 见 deploy/modbus-backend.logrotate）
 *   - 时间戳 + 源码定位
 *
 * 使用示例:
 *   LOG_INIT("app.log", LOG_LEVEL_INFO, true);
 *   LOG_INFO("Gateway connected: %s:%d", host, port);
 *   LOG_ERROR("Read failed: %s", strerror(errno));
 *   LOG_CLOSE();
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __GNUC__
#define LOG_ATTR_PRINTF(fmt, arg) __attribute__((format(printf, fmt, arg)))
#else
#define LOG_ATTR_PRINTF(fmt, arg)
#endif

/* ======================== 日志级别 ======================== */
typedef enum {
    LOG_LEVEL_DEBUG = 0,  /* 调试信息 */
    LOG_LEVEL_INFO  = 1,  /* 常规信息 */
    LOG_LEVEL_WARN  = 2,  /* 警告 */
    LOG_LEVEL_ERROR = 3,  /* 错误 */
    LOG_LEVEL_NONE  = 4,  /* 不输出 */
} LogLevel;

/* ======================== API ======================== */

/**
 * @brief 初始化日志系统
 * @param filepath  日志文件路径（NULL 或 "" 表示仅终端输出）
 * @param level     最低输出级别
 * @param color     终端是否使用 ANSI 颜色
 * @return 0=成功, -1=失败
 */
int  LOG_Init(const char *filepath, LogLevel level, bool color);

/**
 * @brief 设置日志级别（运行时动态调整）
 */
void LOG_SetLevel(LogLevel level);

/**
 * @brief 输出日志（通常用宏调用，不用直接调）
 */
void LOG_Write(LogLevel level, const char *file, int line,
               const char *func, const char *fmt, ...) LOG_ATTR_PRINTF(5, 6);

/**
 * @brief 关闭日志系统
 */
void LOG_Close(void);

/* ======================== 便捷宏 ======================== */
#define LOG_DEBUG(fmt, ...) \
    LOG_Write(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...) \
    LOG_Write(LOG_LEVEL_INFO,  __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...) \
    LOG_Write(LOG_LEVEL_WARN,  __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...) \
    LOG_Write(LOG_LEVEL_ERROR, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

/* 初始化快捷宏 */
#define LOG_INIT(fp, lv, cl) LOG_Init((fp), (lv), (cl))
#define LOG_CLOSE()          LOG_Close()

#endif /* __LOGGER_H */
