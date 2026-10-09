#ifndef __STORAGE_H
#define __STORAGE_H

/*
 * storage.h — SQLite 数据存储接口 v2
 *
 * v2 新增:
 *   - gateway_id / gateway_name 字段，支持多网关数据分离
 *   - 自动 schema 迁移（旧数据库自动加列）
 *   - 按网关查询最新 / 历史数据
 *
 * 表结构 v2:
 *   t_sensor_data (
 *       id          INTEGER PRIMARY KEY AUTOINCREMENT,
 *       gateway_id  INTEGER NOT NULL DEFAULT 0,  -- 网关编号
 *       gateway_name TEXT NOT NULL DEFAULT '',    -- 网关名称
 *       ts          INTEGER NOT NULL,             -- Unix 时间戳 (秒)
 *       temp        REAL,                         -- 温度 (℃)
 *       humidity    REAL                          -- 湿度 (%RH)
 *   );
 *
 * 使用示例:
 *   StorageDB db;
 *   Storage_Open(&db, "data/modbus_data.db");
 *   Storage_InsertV2(&db, &rec);  // 带 gateway 信息
 *   Storage_QueryLatest(&db, 0, &cb, NULL);  // 查询网关 0 最新
 */

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <sqlite3.h>

/* 告警事件记录 */
typedef struct {
    int    alarm_id;          /* DB row id (查询时填充) */
    int    gateway_id;        /* 网关编号 */
    char   gateway_name[64];  /* 网关名称 */
    char   alarm_type[32];    /* temp_high / temp_low / hum_high / hum_low */
    double threshold_value;   /* 触发阈值 */
    double trigger_value;     /* 触发时的实际值 */
    time_t trigger_ts;        /* 触发时间戳 */
    time_t recover_ts;        /* 恢复时间戳 (0 = 未恢复) */
    char   status[16];        /* active / recovered */
} AlarmRecord;

/* 传感器数据记录 v2 */
typedef struct {
    time_t ts;          /* Unix 时间戳 */
    double temperature; /* 温度 (℃) */
    double humidity;    /* 湿度 (%RH) */
    int    gateway_id;  /* 网关编号 */
    char   gateway_name[64]; /* 网关名称 */
} SensorRecord;

/* 数据库上下文 */
typedef struct {
    sqlite3 *db;       /* SQLite 数据库句柄 */
    char     path[256];/* 数据库文件路径 */
    int      opened;   /* 是否已打开 */
    uint32_t insert_count; /* 累计插入条数 */
    int      schema_v2;    /* 是否已完成 v2 迁移 */
} StorageDB;

/* 传感器查询回调 */
typedef int (*StorageQueryCB)(SensorRecord *rec, void *userdata);

/* 告警查询回调 */
typedef int (*AlarmQueryCB)(AlarmRecord *rec, void *userdata);

/**
 * @brief 打开（或创建）数据库，自动执行 schema 迁移
 */
int Storage_Open(StorageDB *db, const char *path);

/**
 * @brief 插入一条传感器数据（含 gateway 信息）
 */
int Storage_Insert(StorageDB *db, SensorRecord *rec);

/**
 * @brief 批量插入
 */
int Storage_InsertBatch(StorageDB *db, SensorRecord *recs, int count);

/**
 * @brief 查询某个网关的最新一条数据
 * @param gateway_id  网关编号（-1 表示所有网关）
 */
int Storage_QueryLatest(StorageDB *db, int gateway_id,
                        StorageQueryCB callback, void *userdata);

/**
 * @brief 查询某个网关的最近 N 条数据
 * @param gateway_id  网关编号（-1 表示所有网关）
 */
int Storage_QueryRecent(StorageDB *db, int gateway_id, int limit,
                        StorageQueryCB callback, void *userdata);

/**
 * @brief 查询某个网关的时间范围数据
 * @param gateway_id  网关编号（-1 表示所有网关）
 */
int Storage_QueryRange(StorageDB *db, int gateway_id, time_t begin, time_t end,
                       StorageQueryCB callback, void *userdata);

/**
 * @brief 查询所有网关的最新数据（用于 /api/latest）
 * @param max_gateways  最多返回的网关数
 * @return 实际查到的网关数，每行通过 callback 返回
 */
int Storage_QueryAllLatest(StorageDB *db, int max_gateways,
                           StorageQueryCB callback, void *userdata);

/**
 * @brief 获取总记录数
 */
int Storage_GetCount(StorageDB *db);

/**
 * @brief 清理超过保留天数的旧数据
 * @param days  保留天数（超过此天数的数据被删除）
 * @return 删除的行数
 */
int Storage_CleanOld(StorageDB *db, uint32_t days);

/**
 * @brief 关闭数据库
 */
void Storage_Close(StorageDB *db);

/* ================================================================
 * 告警事件接口
 * ================================================================ */

/**
 * @brief 插入一条告警事件
 * @return 新记录的 id，失败返回 -1
 */
int Storage_InsertAlarm(StorageDB *db, AlarmRecord *rec);

/**
 * @brief 更新告警恢复时间（标记 recovered）
 * @param alarm_id  告警事件 id
 * @param recover_ts 恢复时间戳
 * @return 0=成功, -1=失败
 */
int Storage_UpdateAlarmRecover(StorageDB *db, int alarm_id, time_t recover_ts);

/**
 * @brief 查询告警事件
 * @param gateway_id  网关编号（-1 = 所有网关）
 * @param begin       时间范围起始（0 = 不限）
 * @param end         时间范围结束（0 = 不限）
 * @param status_filter 状态过滤（NULL = 不限, "active" / "recovered"）
 * @param limit       最大返回条数
 * @param callback    每行回调
 * @param userdata    回调用户数据
 * @return 查询到的行数
 */
int Storage_QueryAlarms(StorageDB *db, int gateway_id,
                        time_t begin, time_t end,
                        const char *status_filter, int limit,
                        AlarmQueryCB callback, void *userdata);

/**
 * @brief 查询活跃告警数量（前端指示灯用）
 * @param gateway_id  网关编号（-1 = 所有）
 * @return 活跃告警数
 */
int Storage_GetActiveAlarmCount(StorageDB *db, int gateway_id);

#endif /* __STORAGE_H */
