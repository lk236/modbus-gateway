/* ============================================================================
 * config.c — INI 配置文件解析实现
 *
 * 手写流式解析（fgets 逐行），不引入 libinih 之类的库：零外部依赖、完全可控。
 *
 * 支持的格式:
 *   [section]        段落
 *   key = value      键值对（等号两边空白会被去掉）
 *   # 注释 / ; 注释   整行注释
 *   空行             忽略
 *   [gateway.N]      N 从 1 开始的多网关段（后端专有扩展）
 *
 * 解析策略：逐行读，维护"当前段名"变量 section，遇 [xxx] 更新，遇 key=value
 * 按当前段分发到对应结构体。O(文件行数)，内存占用恒定。
 * ============================================================================
 */
#include "config.h"
#include <stdio.h>    /* fopen/fgets/snprintf/fprintf */
#include <stdlib.h>   /* strtol/atoi/atof */
#include <string.h>   /* strlen/strchr/strcasecmp */
#include <ctype.h>    /* isspace */
#include <errno.h>    /* errno：fopen 失败时取错误描述 */

/* ---- 从 [gateway.N] 中提取索引 N，返回 0 表示 [gateway] 无序号 ---- */
/* 返回值含义：>=0 表示"确实是网关段"（返回数组下标，0-based）；-1 表示不是网关段。
 * 注意 [gateway]（不带序号）被当作旧格式，映射到 index 0。 */
static int parse_gateway_section(const char *section)
{
    if (strcasecmp(section, "gateway") == 0)
        return 0; /* 旧格式无序号，映射到 index 0 */

    const char *prefix = "gateway.";           /* 形如 gateway.3 的前缀 */
    size_t plen = strlen(prefix);
    if (strncasecmp(section, prefix, plen) == 0) {  /* 前缀匹配（忽略大小写） */
        char *end = NULL;
        long n = strtol(section + plen, &end, 10);  /* 取"点"后面的十进制数字 */
        /* 校验：必须整串都是数字（*end=='\0'，防止 "gateway.3abc" 被接受），
         * 且序号在 1..MAX 之间（配置文件从 1 开始编号，对使用者更直观） */
        if (*end == '\0' && n >= 1 && n <= CONFIG_MAX_GATEWAYS)
            return (int)n - 1; /* 配置文件从 1 开始，数组从 0 */
    }
    return -1; /* 不匹配任何网关段格式 */
}

/* ---- 去除字符串首尾空白 ----
 * 【为什么是原地修改？】直接改缓冲区并返回新起点，不额外分配内存。
 * 返回的指针可能不再是原指针（因为跳过了前导空白），调用者必须用返回值。 */
static char *trim(char *s)
{
    while (isspace((unsigned char)*s)) s++;     /* 跳过前导空白；转 unsigned char 避免负值传入 UB */
    if (*s == '\0') return s;                   /* 整行都是空白 → 返回指向结尾 */

    char *end = s + strlen(s) - 1;              /* 指向最后一个字符 */
    while (end > s && isspace((unsigned char)*end)) end--;  /* 从后往前找非空白 */
    *(end + 1) = '\0';                          /* 把尾部空白第一个字节截断 */
    return s;
}

/* ---- 去除引号 ----
 * 允许 value = "hello world" 这种带引号的写法。
 * 只在"首尾配对"时才去引号，避免把 value = 5" 这种误处理。 */
static char *unquote(char *s)
{
    size_t len = strlen(s);
    if (len >= 2 && ((s[0] == '"' && s[len-1] == '"') ||
                     (s[0] == '\'' && s[len-1] == '\''))) {
        s[len-1] = '\0';   /* 先把结尾引号变成字符串结束符 */
        return s + 1;      /* 再跳过开头引号 */
    }
    return s;              /* 没有引号，原样返回 */
}

/* ---- 设置默认配置 ----
 * 【为什么先 memset 再逐项赋值？】保证任何"没在配置文件里出现的字段"都有确定值，
 * 而不是栈上的随机垃圾。这是 C 里极其重要的习惯。 */
void Config_Default(AppConfig *cfg)
{
    memset(cfg, 0, sizeof(*cfg));   /* 全零：字符串为空、数字为 0 */

    /* 默认 1 个网关（地址用内网常见网段，端口用 Modbus 标准 502） */
    cfg->gateway_count = 1;
    strcpy(cfg->gateways[0].host, "192.168.1.100");
    cfg->gateways[0].port               = 502;
    strcpy(cfg->gateways[0].name,       "Default Gateway");
    cfg->gateways[0].poll_interval_sec  = 5;
    cfg->gateways[0].slave_id           = 1;
    cfg->gateways[0].reg_start          = 0x0300;  /* 与下位机 modbus_map.h 中的映射一致 */
    cfg->gateways[0].reg_count          = 2;       /* 温度 + 湿度，两个保持寄存器 */
    cfg->gateways[0].retry_count        = 3;
    cfg->gateways[0].retry_interval_ms  = 1000;

    strcpy(cfg->db_path,        "data/modbus_data.db");
    cfg->retain_days            = 90;              /* 数据保留 90 天 */

    strcpy(cfg->log_level_str,  "INFO");
    strcpy(cfg->log_file,       "data/modbus-backend.log");
    cfg->log_color              = true;

    cfg->http_port              = 8080;

    strcpy(cfg->firmware_dir,   "firmware");
    cfg->ota_port               = 506;             /* 与下位机 Bootloader 监听端口一致 */
}

/* ---- 值解析辅助函数 ----
 * 一个"宽松"的布尔解析：true/1/yes/on 都算真，其它一律算假。 */
static int parse_bool(const char *s)
{
    if (strcasecmp(s, "true") == 0 || strcasecmp(s, "1") == 0 ||
        strcasecmp(s, "yes") == 0 || strcasecmp(s, "on") == 0)
        return 1;
    return 0;
}

/* ---- 给指定网关索引设置值 ----
 * 这里是典型的"字符串 key 分发"。用 strcasecmp 而非 strcmp，
 * 是为了让配置文件大小写不敏感（HOST / host 都能认），对使用者更友好。
 * 数值用 atoi / strtol / atof 转换；reg_start 用 strtol(value,NULL,0)，
 * 因为 base=0 会自动识别 "0x0300" 这样的十六进制写法。 */
static void set_gateway_value(GatewayConfig *gw, const char *key, const char *value)
{
    if (strcasecmp(key, "host") == 0)
        snprintf(gw->host, sizeof(gw->host), "%s", value);   /* 用 snprintf 而非 strcpy，防溢出 */
    else if (strcasecmp(key, "port") == 0)
        gw->port = (uint16_t)atoi(value);
    else if (strcasecmp(key, "name") == 0)
        snprintf(gw->name, sizeof(gw->name), "%s", value);
    else if (strcasecmp(key, "interval_sec") == 0)
        gw->poll_interval_sec = (uint32_t)atoi(value);
    else if (strcasecmp(key, "slave_id") == 0)
        gw->slave_id = (uint8_t)atoi(value);
    else if (strcasecmp(key, "reg_start") == 0)
        gw->reg_start = (uint16_t)strtol(value, NULL, 0);    /* base=0 → 自动识别 0x 前缀 */
    else if (strcasecmp(key, "reg_count") == 0)
        gw->reg_count = (uint16_t)atoi(value);
    else if (strcasecmp(key, "retry_count") == 0)
        gw->retry_count = (uint8_t)atoi(value);
    else if (strcasecmp(key, "retry_interval_ms") == 0)
        gw->retry_interval_ms = (uint32_t)atoi(value);
    /* ---- 下面 5 项是报警阈值，0 表示该通道不启用 ---- */
    else if (strcasecmp(key, "temp_high") == 0)
        gw->temp_high = atof(value);          /* atof 转浮点，支持 "35.0" */
    else if (strcasecmp(key, "temp_low") == 0)
        gw->temp_low = atof(value);
    else if (strcasecmp(key, "hum_high") == 0)
        gw->hum_high = atof(value);
    else if (strcasecmp(key, "hum_low") == 0)
        gw->hum_low = atof(value);
    else if (strcasecmp(key, "alarm_debounce") == 0)
        gw->alarm_debounce = (uint8_t)atoi(value);
    /* 未知 key 静默忽略：这样配置文件里写错字段名不会导致解析失败，
     * 只是不生效——由 Config_Print 打印出来的最终值可以发现。 */
}

/* ---- 查找INI键值对并赋值 ---- */
static void set_config_value(AppConfig *cfg, const char *section,
                              const char *key, const char *value)
{
    /* 处理 [gateway] 或 [gateway.N] */
    int gw_idx = parse_gateway_section(section);
    if (gw_idx >= 0) {
        /* 【关键】动态扩展网关数量：只要出现更大的序号，就把 count 抬上去。
         * 这样配置文件里不需要显式写 "gateway_count=3"，
         * 写几个 [gateway.N] 段就有几个网关（N 可以跳跃，如 1 和 5 → count=5）。 */
        if (gw_idx + 1 > cfg->gateway_count)
            cfg->gateway_count = gw_idx + 1;
        set_gateway_value(&cfg->gateways[gw_idx], key, value);
        return;   /* 网关段处理完直接返回，不再走下面的全局段判断 */
    }

    /* ---- 下面都是全局段：[storage] / [log] / [http] / [ota] ---- */
    if (strcasecmp(section, "storage") == 0) {
        if (strcasecmp(key, "db_path") == 0)
            snprintf(cfg->db_path, sizeof(cfg->db_path), "%s", value);
        else if (strcasecmp(key, "retain_days") == 0)
            cfg->retain_days = (uint32_t)atoi(value);
    }
    else if (strcasecmp(section, "log") == 0) {
        if (strcasecmp(key, "level") == 0)
            snprintf(cfg->log_level_str, sizeof(cfg->log_level_str), "%s", value);
        else if (strcasecmp(key, "file") == 0)
            snprintf(cfg->log_file, sizeof(cfg->log_file), "%s", value);
        else if (strcasecmp(key, "color") == 0)
            cfg->log_color = parse_bool(value) ? true : false;
    }
    else if (strcasecmp(section, "http") == 0) {
        if (strcasecmp(key, "port") == 0)
            cfg->http_port = (uint16_t)atoi(value);
    }
    else if (strcasecmp(section, "ota") == 0) {
        if (strcasecmp(key, "firmware_dir") == 0)
            snprintf(cfg->firmware_dir, sizeof(cfg->firmware_dir), "%s", value);
        else if (strcasecmp(key, "ota_port") == 0)
            cfg->ota_port = (uint16_t)atoi(value);
    }
}

/* ---- 加载 INI 文件 ---- */
int Config_Load(const char *filepath, AppConfig *cfg)
{
    FILE *fp = fopen(filepath, "r");   /* "r" = 只读文本模式 */
    if (fp == NULL) {
        /* 【易错点】fopen 失败后用 errno 取具体原因（不存在/没权限），
         * 而不是只报"打开失败"——运维排查时这一步能省很多时间。 */
        fprintf(stderr, "[CONFIG] Cannot open '%s': %s\n",
                filepath, strerror(errno));
        return -1;
    }

    Config_Default(cfg);   /* 【顺序关键】先铺默认值，再用文件内容逐项覆盖 */

    char line[512];                 /* 单行缓冲，超长行会被 fgets 截断（可接受） */
    char section[64] = "";          /* 当前段名，初始为空（文件开头若无 [xxx]，键值对会被忽略） */
    int  line_num = 0;              /* 行号，报错时定位用 */

    while (fgets(line, sizeof(line), fp)) {   /* 每次读一行（含换行符），EOF 返回 NULL */
        line_num++;

        /* 去掉末尾换行 —— 兼容 Unix(\n) 与 Windows(\r\n) 两种行尾 */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r'))
            line[--len] = '\0';

        char *p = trim(line);   /* p 指向本行第一个有效字符 */

        /* 空行 / 注释：直接跳过 */
        if (*p == '\0' || *p == '#' || *p == ';') continue;

        /* [section] —— 段头解析 */
        if (*p == '[') {
            char *end = strchr(p, ']');   /* 找右方括号 */
            if (end == NULL) {
                fprintf(stderr, "[CONFIG] %s:%d: missing ']'\n", filepath, line_num);
                continue;   /* 只是忽略这一行，不终止整个解析（容错优先） */
            }
            *end = '\0';                                  /* 把 ']' 处截断 */
            snprintf(section, sizeof(section), "%s", trim(p + 1));  /* 记录段名 */
            continue;                                     /* 段头行不产生键值对 */
        }

        /* key = value —— 键值对解析 */
        char *eq = strchr(p, '=');   /* 找等号 */
        if (eq == NULL) {
            fprintf(stderr, "[CONFIG] %s:%d: missing '='\n", filepath, line_num);
            continue;   /* 同样容错：忽略这一行 */
        }

        *eq = '\0';                       /* 把 '=' 变成结束符，左半是 key */
        char *key   = trim(p);            /* 去空白后的键名 */
        char *value = trim(eq + 1);       /* 等号右边去空白 */
        value = unquote(value);           /* 若带引号则剥掉 */

        if (*key == '\0') continue;       /* "  = 3" 这种空键名，忽略 */

        set_config_value(cfg, section, key, value);   /* 分发到对应结构体字段 */
    }

    fclose(fp);
    return 0;
}

/* ---- 打印配置 ----
 * 启动时调用。作用：把"程序实际用的值"全部暴露出来，
 * 用来验证配置文件是否被正确解析（尤其是有默认值掺入时）。 */
void Config_Print(const AppConfig *cfg)
{
    printf("========== Config (gateways=%d) ==========\n", cfg->gateway_count);
    for (int i = 0; i < cfg->gateway_count; i++) {
        const GatewayConfig *gw = &cfg->gateways[i];
        printf("[gateway.%d] %s host=%s:%u name=%s interval=%us reg=0x%04X/%u\n",
               i + 1, gw->name, gw->host, gw->port,
               gw->name, gw->poll_interval_sec,
               gw->reg_start, gw->reg_count);
    }
    printf("[storage]   db=%s retain=%udays\n", cfg->db_path, cfg->retain_days);
    printf("[log]       level=%s file=%s color=%d\n",
           cfg->log_level_str,
           cfg->log_file[0] ? cfg->log_file : "(console)",   /* 空路径显示成 (console) 更直观 */
           cfg->log_color);
    printf("[http]      port=%u\n", cfg->http_port);
    printf("[ota]       firmware_dir=%s ota_port=%u\n",
           cfg->firmware_dir, cfg->ota_port);
    printf("=========================================\n");
}

/* ---- 日志级别字符串 → 枚举 ----
 * 返回值要与 logger.h 中 LogLevel 的数值对应（DEBUG=0 ... NONE=4）。
 * 不认识的一律兜底为 INFO，避免把日志级别设成空导致什么都不打印。 */
int Config_ParseLogLevel(const char *str)
{
    if (strcasecmp(str, "DEBUG") == 0) return 0;
    if (strcasecmp(str, "INFO")  == 0) return 1;
    if (strcasecmp(str, "WARN")  == 0) return 2;
    if (strcasecmp(str, "ERROR") == 0) return 3;
    if (strcasecmp(str, "NONE")  == 0) return 4;
    return 1; /* 默认 INFO */
}
