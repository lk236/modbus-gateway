#ifndef __ALARM_H
#define __ALARM_H

/*
 * alarm.h — 阈值报警引擎接口
 *
 * 设计要点:
 *   1. 每网关 4 个独立报警通道：temp_high / temp_low / hum_high / hum_low
 *   2. 去抖策略：连续 N 次超限 → 触发告警；连续 N 次正常 → 标记恢复
 *   3. 告警事件持久化到 SQLite，支持历史查询
 *
 * 使用方式:
 *   AlarmState state;
 *   Alarm_Init(&state);
 *   Alarm_Check(&state, cfg, temperature, humidity, db_path);  // 每次成功采集后调用
 */

#include "config.h"
#include <stdint.h>

/* 报警类型枚举 */
typedef enum {
    ALARM_TEMP_HIGH = 0,
    ALARM_TEMP_LOW,
    ALARM_HUM_HIGH,
    ALARM_HUM_LOW,
    ALARM_TYPE_COUNT
} AlarmType;

/* 单个报警通道的去抖状态 */
typedef struct {
    int  over_count;      /* 连续超限次数 */
    int  normal_count;    /* 连续正常次数（用于恢复） */
    int  active;          /* 0 = 正常, 1 = 告警中 */
    int  alarm_id;        /* DB 中活跃告警的 row id，0 = 无 */
} AlarmChannel;

/* 单个网关的报警状态（4 个通道） */
typedef struct {
    AlarmChannel channels[ALARM_TYPE_COUNT];
} AlarmState;

/**
 * @brief 初始化报警状态（所有通道清零）
 */
void Alarm_Init(AlarmState *state);

/**
 * @brief 执行一次阈值检查和去抖判定
 * @param state       该网关的报警状态（会更新内部计数器）
 * @param cfg         该网关的配置（阈值来源）
 * @param gateway_id  网关编号
 * @param temperature 当前温度 (℃)
 * @param humidity    当前湿度 (%)
 * @param db_path     数据库路径（用于写入告警事件）
 *
 * 每次成功采集后调用一次。内部会:
 *   1. 用 cfg 中的阈值比对当前值
 *   2. 更新去抖计数器
 *   3. 超限达到 debounce 次 → 写入 DB + 写日志
 *   4. 正常达到 debounce 次 → 更新 DB 恢复时间 + 写日志
 */
void Alarm_Check(AlarmState *state, const GatewayConfig *cfg,
                 int gateway_id, double temperature, double humidity,
                 const char *db_path);

/**
 * @brief 获取告警类型名称
 */
const char *Alarm_TypeName(AlarmType type);

/**
 * @brief 获取告警类型的阈值
 */
double Alarm_GetThreshold(const GatewayConfig *cfg, AlarmType type);

#endif /* __ALARM_H */
