/* ============================================================================
 * alarm.c — 阈值报警引擎实现
 *
 * 每次采集后调用 Alarm_Check()，对 4 个报警通道逐一判断是否超限，并用
 * "连续 N 次"计数做去抖（消除总线偶发错误、传感器噪声导致的误报与告警风暴）:
 *   1) 用阈值判断当前值是否超限
 *   2) 超限: over_count++, normal_count 清零; 正常: 反之
 *   3) over_count 达到 debounce → 触发告警（写 DB + 日志）
 *   4) normal_count 达到 debounce 且之前活跃 → 标记恢复（更新 DB + 日志）
 *
 * 用计数而非定时: 采集周期本身是固定节拍，连续 N 次天然等价于 N×周期的时间窗，
 * 且与采集频率自动解耦。
 * ============================================================================
 */
#include "alarm.h"
#include "storage.h"
#include "logger.h"
#include <string.h>
#include <stdio.h>
#include <time.h>

/* ---- 阈值获取 ----
 * 按报警类型取出对应阈值。用 switch 而非数组，是因为阈值散落在 cfg 的各个字段里。
 * 【易错点】default 返回 0.0，而 0 在本系统里表示"不启用"（见 is_over_limit），
 * 所以即使传入非法类型，行为也是安全的（不报错、不误报）。 */
static double get_threshold(const GatewayConfig *cfg, AlarmType type)
{
    switch (type) {
        case ALARM_TEMP_HIGH: return cfg->temp_high;   /* 温度上限 */
        case ALARM_TEMP_LOW:  return cfg->temp_low;    /* 温度下限 */
        case ALARM_HUM_HIGH:  return cfg->hum_high;    /* 湿度上限 */
        case ALARM_HUM_LOW:   return cfg->hum_low;     /* 湿度下限 */
        default:              return 0.0;
    }
}

/* ---- 判断是否超限 ---- */
/* value 由调用方在 check_channel 之前根据类型选好（温度或湿度）*/
static int is_over_limit(AlarmType type, double threshold, double value)
{
    if (threshold <= 0.0) return 0;  /* 【约定】阈值为 0 = 不启用该通道 */

    switch (type) {
        case ALARM_TEMP_HIGH: return (value > threshold);   /* 上限：大于才超 */
        case ALARM_TEMP_LOW:  return (value < threshold);   /* 下限：小于才超 */
        case ALARM_HUM_HIGH:  return (value > threshold);
        case ALARM_HUM_LOW:   return (value < threshold);
        default:              return 0;
    }
}

/* ---- 告警类型名 ----
 * 这两个表都用"指定初始化器"绑定枚举值，避免顺序错位。
 * type_names 是英文标识，会写进数据库（语义稳定、便于程序判断）；
 * type_desc 是中文描述，只用于日志（给人看）。 */
static const char *type_names[] = {
    [ALARM_TEMP_HIGH] = "temp_high",
    [ALARM_TEMP_LOW]  = "temp_low",
    [ALARM_HUM_HIGH]  = "hum_high",
    [ALARM_HUM_LOW]   = "hum_low",
};

/* ---- 告警类型中文描述 ---- */
static const char *type_desc[] = {
    [ALARM_TEMP_HIGH] = "温度过高",
    [ALARM_TEMP_LOW]  = "温度过低",
    [ALARM_HUM_HIGH]  = "湿度过高",
    [ALARM_HUM_LOW]   = "湿度过低",
};

/* 对外的类型名查询接口，带边界检查（防越界访问） */
const char *Alarm_TypeName(AlarmType type)
{
    if (type < 0 || type >= ALARM_TYPE_COUNT) return "unknown";
    return type_names[type];
}

double Alarm_GetThreshold(const GatewayConfig *cfg, AlarmType type)
{
    return get_threshold(cfg, type);
}

/* ---- 初始化 ----
 * 全部清零，等价于"4 个通道都处于正常、未告警、无 DB 记录"的初始状态。 */
void Alarm_Init(AlarmState *state)
{
    memset(state, 0, sizeof(*state));
}

/* ---- 触发告警：写 DB + 日志 ----
 * 只有"成功写进数据库拿到 id"才算真正触发成功——
 * 否则 ch->active 不置 1，下次还会再尝试（宁可重试，也不能漏报）。 */
static void trigger_alarm(AlarmChannel *ch, const GatewayConfig *cfg,
                          int gateway_id, AlarmType type,
                          double threshold, double value,
                          const char *db_path)
{
    time_t now = time(NULL);

    /* 打开 DB 写入告警事件（同样用短连接，见 gateway.c 的说明） */
    StorageDB db;
    if (Storage_Open(&db, db_path) != 0) {
        LOG_ERROR("[ALARM] Failed to open DB for alarm insert");
        return;   /* 直接返回：alarm_id 不设置，下轮会重试 */
    }

    /* 组装告警记录 */
    AlarmRecord rec;
    memset(&rec, 0, sizeof(rec));
    rec.gateway_id     = gateway_id;
    snprintf(rec.gateway_name, sizeof(rec.gateway_name), "%s", cfg->name);
    snprintf(rec.alarm_type, sizeof(rec.alarm_type), "%s", type_names[type]);
    rec.threshold_value = threshold;   /* 当时的阈值（阈值可能被改，记下来便于追溯） */
    rec.trigger_value   = value;       /* 触发时的实际值 */
    rec.trigger_ts      = now;
    rec.recover_ts      = 0;           /* 0 = 尚未恢复 */
    snprintf(rec.status, sizeof(rec.status), "active");

    int alarm_id = Storage_InsertAlarm(&db, &rec);   /* 返回新记录的 rowid */
    Storage_Close(&db);

    if (alarm_id > 0) {
        /* 【关键】记住 DB 里的 row id，将来"恢复"时要用它 UPDATE 那一行 */
        ch->alarm_id = alarm_id;
        ch->active   = 1;
        LOG_WARN("[ALARM] GW%d %s 触发告警: %s (阈值=%.1f 实际=%.1f) alarm_id=%d",
                 gateway_id, cfg->name,
                 type_desc[type], threshold, value, alarm_id);
    }
}

/* ---- 恢复告警：更新 DB + 日志 ----
 * 恢复的前提是"之前确实触发过"（alarm_id > 0），否则什么也不做。 */
static void recover_alarm(AlarmChannel *ch, const GatewayConfig *cfg,
                          int gateway_id, AlarmType type,
                          const char *db_path)
{
    if (ch->alarm_id <= 0) return;   /* 没有活跃告警记录，无需恢复 */

    time_t now = time(NULL);

    StorageDB db;
    if (Storage_Open(&db, db_path) != 0) {
        LOG_ERROR("[ALARM] Failed to open DB for alarm recover");
        return;   /* 失败则保持 active，下轮再试 */
    }

    Storage_UpdateAlarmRecover(&db, ch->alarm_id, now);   /* 把那一行改成 recovered */
    Storage_Close(&db);

    LOG_INFO("[ALARM] GW%d %s 告警恢复: %s alarm_id=%d",
             gateway_id, cfg->name, type_desc[type], ch->alarm_id);

    /* 清状态，回到"正常待命"，为下一次告警做准备 */
    ch->alarm_id = 0;
    ch->active   = 0;
}

/* ---- 单通道检查 ----
 * 这是整个去抖算法的核心。请重点理解 over_count / normal_count 这对计数器的配合。 */
static void check_channel(AlarmChannel *ch, const GatewayConfig *cfg,
                          int gateway_id, AlarmType type,
                          double threshold, double value,
                          const char *db_path)
{
    int debounce = cfg->alarm_debounce;
    if (debounce < 1) debounce = 3;   /* 配置缺省/写错时的兜底值，避免 debounce==0 导致一超就报 */

    int over = is_over_limit(type, threshold, value);

    if (over) {
        ch->over_count++;
        ch->normal_count = 0;   /* 【对称设计】超限就把"连续正常"计数清零 */

        /* 连续超限达到 debounce 次，且当前未活跃 → 触发告警 */
        if (ch->over_count >= debounce && !ch->active) {
            trigger_alarm(ch, cfg, gateway_id, type, threshold, value, db_path);
            ch->over_count = 0;  /* 重置计数器，避免重复触发（下次要再攒 debounce 次） */
        }
    } else {
        ch->normal_count++;
        ch->over_count = 0;     /* 同理：正常就把"连续超限"清零 */

        /* 连续正常达到 debounce 次，且当前活跃 → 标记恢复 */
        if (ch->normal_count >= debounce && ch->active) {
            recover_alarm(ch, cfg, gateway_id, type, db_path);
            ch->normal_count = 0;
        }
    }
}

/* ---- 对外接口：每次采集后调用 ----
 * 【注意】没有信号量/互斥锁保护。因为每个 GatewayCtx 只被主线程（epoll 循环）访问，
 *   报警状态天然无并发，不需要锁——这正是"单线程采集"架构带来的好处之一。 */
void Alarm_Check(AlarmState *state, const GatewayConfig *cfg,
                 int gateway_id, double temperature, double humidity,
                 const char *db_path)
{
    /* 检查 4 个通道 */
    for (int t = 0; t < ALARM_TYPE_COUNT; t++) {
        double threshold = get_threshold(cfg, (AlarmType)t);
        if (threshold <= 0.0) continue;  /* 该通道未启用（阈值为 0），直接跳过 */

        /* 根据通道类型选择"拿温度还是拿湿度"来比对 */
        double value = 0.0;
        switch (t) {
            case ALARM_TEMP_HIGH:
            case ALARM_TEMP_LOW:  value = temperature; break;
            case ALARM_HUM_HIGH:
            case ALARM_HUM_LOW:   value = humidity;    break;
        }

        check_channel(&state->channels[t], cfg, gateway_id,
                      (AlarmType)t, threshold, value, db_path);
    }
}
