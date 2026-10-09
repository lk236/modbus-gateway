#ifndef __CONFIG_H
#define __CONFIG_H

/*
 * config.h — INI 配置文件解析接口
 *
 * 支持的格式:
 *   [section]
 *   key = value    # 注释
 *   [gateway.N]    # 多网关（N 从 1 开始）
 *
 * v2: 支持多网关 + HTTP 端口配置
 */

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_HOST_MAX       64
#define CONFIG_NAME_MAX       64
#define CONFIG_DB_PATH_MAX    256
#define CONFIG_LOG_PATH_MAX   256
#define CONFIG_MAX_GATEWAYS   16

/* 单个网关的配置 */
typedef struct {
    char     host[CONFIG_HOST_MAX];
    uint16_t port;
    char     name[CONFIG_NAME_MAX];
    uint32_t poll_interval_sec;
    uint8_t  slave_id;
    uint16_t reg_start;
    uint16_t reg_count;
    uint8_t  retry_count;
    uint32_t retry_interval_ms;

    /* ---- 报警阈值（0 = 不启用该通道） ---- */
    double   temp_high;         /* 温度上限 (℃) */
    double   temp_low;          /* 温度下限 (℃) */
    double   hum_high;          /* 湿度上限 (%RH) */
    double   hum_low;           /* 湿度下限 (%RH) */
    uint8_t  alarm_debounce;    /* 去抖次数（连续 N 次超限才触发） */
} GatewayConfig;

/* 应用配置结构体 */
typedef struct {
    GatewayConfig gateways[CONFIG_MAX_GATEWAYS];
    int           gateway_count;

    /* [storage] */
    char     db_path[CONFIG_DB_PATH_MAX];
    uint32_t retain_days;

    /* [log] */
    char     log_level_str[16];
    char     log_file[CONFIG_LOG_PATH_MAX];
    bool     log_color;

    /* [http] */
    uint16_t http_port;

    /* [ota] */
    char     firmware_dir[256];
    uint16_t ota_port;
} AppConfig;

/**
 * @brief 加载配置文件，失败返回 -1
 */
int Config_Load(const char *filepath, AppConfig *cfg);

/**
 * @brief 设置为默认值
 */
void Config_Default(AppConfig *cfg);

/**
 * @brief 打印当前配置
 */
void Config_Print(const AppConfig *cfg);

/**
 * @brief 将日志级别字符串转为枚举
 */
int Config_ParseLogLevel(const char *str);

#endif /* __CONFIG_H */
