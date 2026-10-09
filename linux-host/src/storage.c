/* ============================================================================
 * storage.c — SQLite 数据存储实现 v2
 *
 * 选 SQLite 而非 MySQL/Redis：单机、单进程、数据量不大、需历史查询。
 * SQLite 零部署、单文件、无服务进程、支持完整 SQL 与事务；MySQL 要单独装服务、
 * Redis 断电可能丢数据，都属杀鸡用牛刀。选型原则是让架构匹配真实规模。
 *
 * v2 变更:
 *   1. schema 升级（加 gateway_id / gateway_name 列，支持多网关数据分离）
 *   2. 自动迁移（检测旧表并 ALTER TABLE ADD COLUMN，老数据库平滑升级）
 *   3. 新增按网关查询的接口
 *
 * 核心工程习惯:
 *   - 所有 SQL 都用"预编译 + 参数绑定"（sqlite3_prepare_v2 + sqlite3_bind_*），
 *     不用 sprintf 拼字符串 —— 防 SQL 注入的标准做法。
 *   - 每条 prepare 必须配 finalize，否则会泄漏资源（stmt 不释放会累积）。
 * ============================================================================
 */
#include "storage.h"
#include "logger.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ---- v2 传感器数据表 SQL ----
 * 【字段说明】
 *   id          : 自增主键（SQLite 的 INTEGER PRIMARY KEY 自动成为 rowid 别名）
 *   gateway_id  : 网关编号，用于多网关数据分离（WHERE gateway_id=? 查某台设备）
 *   gateway_name: 网关名称（冗余存储，让查询结果自带名字，避免再 JOIN 配置表）
 *   ts          : Unix 时间戳（秒），存整数便于比较与索引
 *   temp/humidity: REAL 浮点，直接存摄氏度/百分比
 * 【索引】idx_ts 用于"按时间范围查询"；idx_gw_ts 用于"某网关按时间查询"，
 *   复合索引把 gateway_id 放前面是因为查询常带等值条件 gateway_id=?。 */
static const char *g_schema_v2_sql =
    "CREATE TABLE IF NOT EXISTS t_sensor_data ("
    "  id          INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  gateway_id  INTEGER NOT NULL DEFAULT 0,"
    "  gateway_name TEXT NOT NULL DEFAULT '',"
    "  ts          INTEGER NOT NULL,"
    "  temp        REAL,"
    "  humidity    REAL"
    ");"
    "CREATE INDEX IF NOT EXISTS idx_ts     ON t_sensor_data(ts);"
    "CREATE INDEX IF NOT EXISTS idx_gw_ts ON t_sensor_data(gateway_id, ts);"
    "";

/* ---- 告警事件表 SQL ----
 * 【设计要点】告警是"一条记录对应一次告警的生命周期"：
 *   active 时插入一行，恢复时 UPDATE 同一行的 status 和 recover_ts。
 *   这样"查询某次告警"只需一行，不用把触发/恢复拆成两条记录再关联。 */
static const char *g_schema_alarm_sql =
    "CREATE TABLE IF NOT EXISTS t_alarm_events ("
    "  id             INTEGER PRIMARY KEY AUTOINCREMENT,"
    "  gateway_id     INTEGER NOT NULL,"
    "  gateway_name   TEXT NOT NULL DEFAULT '',"
    "  alarm_type     TEXT NOT NULL,"
    "  threshold_value REAL,"
    "  trigger_value   REAL,"
    "  trigger_ts     INTEGER NOT NULL,"
    "  recover_ts     INTEGER DEFAULT 0,"
    "  status         TEXT NOT NULL DEFAULT 'active'"
    ");"
    "CREATE INDEX IF NOT EXISTS idx_alarm_gw   ON t_alarm_events(gateway_id);"
    "CREATE INDEX IF NOT EXISTS idx_alarm_ts   ON t_alarm_events(trigger_ts);"
    "CREATE INDEX IF NOT EXISTS idx_alarm_stat ON t_alarm_events(status);"
    "";

/* ---- 检查表中是否存在列 ----
 * 【为什么需要这个？】做 schema 迁移前，得先知道旧表有没有目标列，
 *   否则执行 ADD COLUMN 会报"duplicate column name"。
 * 【实现技巧】用 "PRAGMA table_info(表名)" 这条特殊语句，
 *   它会返回该表的所有列信息（每列一行），其中第 1 列(索引1)是列名。 */
static int column_exists(sqlite3 *db, const char *table, const char *col)
{
    char sql[256];
    /* 【注意】这里的表名是硬编码常量（非用户输入），所以拼接是安全的。
     *   但凡涉及用户输入，都绝不能用这种方式拼 SQL。 */
    snprintf(sql, sizeof(sql), "PRAGMA table_info(%s);", table);

    sqlite3_stmt *stmt = NULL;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, NULL) != SQLITE_OK)
        return 0;   /* 准备失败（如表不存在）当作"没有该列" */

    int found = 0;
    /* 逐行遍历：sqlite3_step 返回 SQLITE_ROW 表示还有一行，SQLITE_DONE 表示结束 */
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        /* 第 1 列是列名（0 是 cid，1 是 name，2 是 type...） */
        const char *name = (const char *)sqlite3_column_text(stmt, 1);
        if (name && strcasecmp(name, col) == 0) {   /* 忽略大小写比较列名 */
            found = 1;
            break;
        }
    }
    sqlite3_finalize(stmt);   /* 【必须】释放语句对象 */
    return found;
}

/* ---- 执行 schema 迁移 ----
 * 目标：让"v1 时代建的旧数据库"自动升级到 v2（加上 gateway_id/gateway_name 列），
 * 用户不需要手工导出/重建数据库。这是"向前兼容"的工程实践。 */
static int migrate_schema(sqlite3 *db)
{
    /* 检查是否是空数据库 */
    sqlite3_stmt *stmt = NULL;
    /* 到 sqlite_master（SQLite 的系统表，记录所有表/索引的定义）里查表是否存在 */
    int rc = sqlite3_prepare_v2(db,
        "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='t_sensor_data';",
        -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        sqlite3_finalize(stmt);
        return 0; /* 全新数据库，建表时会自动用 v2 格式 */
    }
    int has_table = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        has_table = sqlite3_column_int(stmt, 0);   /* 计数：1=表存在，0=不存在 */
    }
    sqlite3_finalize(stmt);

    if (!has_table) {
        return 0; /* 全新数据库，无需迁移 */
    }

    /* 检查是否已有 gateway_id 列 */
    if (column_exists(db, "t_sensor_data", "gateway_id")) {
        return 0; /* 已经是 v2，无需再迁移（保证本函数可重复执行/幂等） */
    }

    /* 旧表迁移：添加列 */
    LOG_INFO("Storage: migrating schema from v1 to v2...");

    /* 【注意 err 的用法】sqlite3_exec 出错时会把错误信息 malloc 出来放进 err，
     *   调用方必须用 sqlite3_free 释放，否则内存泄漏。 */
    char *err = NULL;
    rc = sqlite3_exec(db,
        "ALTER TABLE t_sensor_data ADD COLUMN gateway_id INTEGER NOT NULL DEFAULT 0;",
        NULL, NULL, &err);
    if (rc != SQLITE_OK) {
        LOG_WARN("Migration ADD gateway_id failed: %s", err);
        sqlite3_free(err);
        return -1;
    }

    rc = sqlite3_exec(db,
        "ALTER TABLE t_sensor_data ADD COLUMN gateway_name TEXT NOT NULL DEFAULT '';",
        NULL, NULL, &err);
    if (rc != SQLITE_OK) {
        LOG_WARN("Migration ADD gateway_name failed: %s", err);
        sqlite3_free(err);
        return -1;
    }

    /* 创建新索引（配合新加的 gateway_id 列） */
    sqlite3_exec(db,
        "CREATE INDEX IF NOT EXISTS idx_gw_ts ON t_sensor_data(gateway_id, ts);",
        NULL, NULL, NULL);

    LOG_INFO("Storage: schema migration v1→v2 complete");
    return 0;
}

/* ---- 插入 SQL (预编译) ----
 * ?1..?5 是"参数占位符"（SQLite 支持带编号的 ?N，绑定更清晰）。
 * 预编译一次、多次复用的模式在 Storage_InsertBatch 里体现得最明显。 */
static const char *g_insert_sql =
    "INSERT INTO t_sensor_data (gateway_id, gateway_name, ts, temp, humidity) "
    "VALUES (?1, ?2, ?3, ?4, ?5);";

/* ---- 开启数据库 ----
 * 返回 0=成功，-1=失败。整个过程包含：打开 → 开 WAL → 迁移 → 建表。 */
int Storage_Open(StorageDB *db, const char *path)
{
    memset(db, 0, sizeof(*db));   /* 保证未赋值的字段是确定值 */
    snprintf(db->path, sizeof(db->path), "%s", path);

    int rc = sqlite3_open(path, &db->db);   /* 文件不存在会自动创建 */
    if (rc != SQLITE_OK) {
        /* 【易错点】sqlite3_open 失败时 db 句柄可能仍非空，标准做法是取 errmsg 后关闭。
         *   这里只打印不关闭（简化处理），但错误信息必须取到。 */
        LOG_ERROR("Failed to open DB '%s': %s", path, sqlite3_errmsg(db->db));
        return -1;
    }

    /* 启用 WAL 模式
     * 【WAL 是什么、为什么用它】
     *   传统的 rollback journal 模式下，写入时会阻塞其他读写（同一时刻只能一个人写）。
     *   WAL(Write-Ahead Logging) 把修改先写日志文件，读操作可以继续读旧快照，
     *   从而实现"读写并发"、大幅提升写入吞吐，非常适合"一边采集写、一边 Web 读"的场景。 */
    char *err = NULL;
    rc = sqlite3_exec(db->db, "PRAGMA journal_mode=WAL;", NULL, NULL, &err);
    if (rc != SQLITE_OK) {
        LOG_WARN("WAL mode failed: %s", err);   /* 非致命：退回普通模式也能跑 */
        sqlite3_free(err);
    }

    /* 执行 schema 迁移（旧库升级） */
    migrate_schema(db->db);

    /* v2 建表（如果表不存在会新建，已存在则跳过——靠 CREATE TABLE IF NOT EXISTS） */
    rc = sqlite3_exec(db->db, g_schema_v2_sql, NULL, NULL, &err);
    if (rc != SQLITE_OK) {
        LOG_ERROR("Schema v2 init failed: %s", err);
        sqlite3_free(err);
        sqlite3_close(db->db);   /* 【回滚】建表失败就关掉句柄，不让调用方拿到半残的 db */
        db->db = NULL;
        return -1;
    }

    /* 告警事件表 */
    rc = sqlite3_exec(db->db, g_schema_alarm_sql, NULL, NULL, &err);
    if (rc != SQLITE_OK) {
        LOG_WARN("Alarm table init failed (non-fatal): %s", err);
        sqlite3_free(err);
        /* 告警表非致命，继续运行（采集数据比告警更重要） */
    }

    db->opened    = 1;
    db->schema_v2 = 1;
    db->insert_count = 0;
    LOG_INFO("Storage opened: %s", path);
    return 0;
}

/* ---- 插入记录 ---- */
int Storage_Insert(StorageDB *db, SensorRecord *rec)
{
    if (!db->opened || !db->db) {   /* 【防御】双重判空：标志位 + 句柄都要检查 */
        LOG_ERROR("Insert: DB not opened");
        return -1;
    }

    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db->db, g_insert_sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        LOG_ERROR("Insert prepare failed: %s", sqlite3_errmsg(db->db));
        return -1;
    }

    /* ---- 参数绑定（从 1 开始计数，对应 SQL 里的 ?1..?5）----
     * SQLITE_STATIC 告诉 SQLite："这块内存的生命周期由我保证，你不用拷贝"。
     * 因为 gateway_name 在调用期间一直有效，所以用 STATIC 可以少一次内存拷贝。
     * （若传的是临时缓冲区，就必须用 SQLITE_TRANSIENT 让 SQLite 自行拷贝。） */
    sqlite3_bind_int(stmt, 1, rec->gateway_id);
    sqlite3_bind_text(stmt, 2, rec->gateway_name, -1, SQLITE_STATIC);   /* -1 = 按 strlen 自动算长度 */
    sqlite3_bind_int64(stmt, 3, (sqlite3_int64)rec->ts);               /* time_t 转 SQLite 的 64 位整数 */
    sqlite3_bind_double(stmt, 4, rec->temperature);
    sqlite3_bind_double(stmt, 5, rec->humidity);

    rc = sqlite3_step(stmt);   /* 执行。INSERT 成功返回 SQLITE_DONE */
    if (rc != SQLITE_DONE) {
        LOG_ERROR("Insert step failed: %s", sqlite3_errmsg(db->db));
        sqlite3_finalize(stmt);
        return -1;
    }

    sqlite3_finalize(stmt);
    db->insert_count++;   /* 累计计数，退出时用来核对写入量与采集量是否一致 */
    return 0;
}

/* ---- 批量插入 ----
 * 【为什么需要批量？】SQLite 默认每条 INSERT 都单独提交事务并 fsync 磁盘，
 *   写 1000 条就是 1000 次磁盘同步，慢得离谱。
 *   放进一个事务里（BEGIN...COMMIT）能把 1000 次同步压缩成 1 次，
 *   性能通常能提升几十倍——这是 SQLite 使用中最经典的一条优化。 */
int Storage_InsertBatch(StorageDB *db, SensorRecord *recs, int count)
{
    if (!db->opened || !db->db || count <= 0) return -1;

    /* 显式开启事务 */
    char *err = NULL;
    if (sqlite3_exec(db->db, "BEGIN TRANSACTION;", NULL, NULL, &err) != SQLITE_OK) {
        LOG_ERROR("Batch BEGIN failed: %s", err);
        sqlite3_free(err);
        return -1;
    }

    /* 只 prepare 一次，循环内反复 bind+step+reset —— "预编译复用"的典型写法 */
    sqlite3_stmt *stmt = NULL;
    sqlite3_prepare_v2(db->db, g_insert_sql, -1, &stmt, NULL);

    int ok = 0;
    for (int i = 0; i < count; i++) {
        sqlite3_bind_int(stmt, 1, recs[i].gateway_id);
        sqlite3_bind_text(stmt, 2, recs[i].gateway_name, -1, SQLITE_STATIC);
        sqlite3_bind_int64(stmt, 3, (sqlite3_int64)recs[i].ts);
        sqlite3_bind_double(stmt, 4, recs[i].temperature);
        sqlite3_bind_double(stmt, 5, recs[i].humidity);

        if (sqlite3_step(stmt) == SQLITE_DONE) ok++;
        /* 【关键】reset 把语句重置回"可再次绑定"状态，但保留预编译结果。
         * 注意是 reset（复位）而不是 finalize（销毁）——finalize 只能调用一次。 */
        sqlite3_reset(stmt);
    }

    sqlite3_finalize(stmt);

    /* 提交事务。若这里失败，前面插入的都会回滚，保证数据一致性 */
    if (sqlite3_exec(db->db, "COMMIT;", NULL, NULL, &err) != SQLITE_OK) {
        LOG_ERROR("Batch COMMIT failed: %s", err);
        sqlite3_free(err);
        return -1;
    }

    db->insert_count += ok;
    return ok;
}

/* ---- 读一行到 SensorRecord ----
 * 【按列数判断的写法】这种 "if (ncol >= N)" 是为了兼容不同查询语句
 * （有的 SELECT 5 列，有的可能只选 3 列），避免读不存在的列拿到 NULL 或垃圾。 */
static int read_row_to_record(sqlite3_stmt *stmt, SensorRecord *rec)
{
    int ncol = sqlite3_column_count(stmt);

    /* 先给默认值，防止某列缺失时结构体残留上次的旧数据 */
    rec->gateway_id   = 0;
    rec->gateway_name[0] = '\0';
    rec->ts           = 0;
    rec->temperature  = 0;
    rec->humidity     = 0;

    if (ncol >= 1) rec->gateway_id   = sqlite3_column_int(stmt, 0);
    if (ncol >= 2) {
        const char *name = (const char *)sqlite3_column_text(stmt, 1);
        if (name) snprintf(rec->gateway_name, sizeof(rec->gateway_name), "%s", name);
    }
    if (ncol >= 3) rec->ts           = (time_t)sqlite3_column_int64(stmt, 2);
    if (ncol >= 4) rec->temperature  = sqlite3_column_double(stmt, 3);
    if (ncol >= 5) rec->humidity     = sqlite3_column_double(stmt, 4);

    return 0;
}

/* ---- 查询某个网关的最新一条 ----
 * gateway_id < 0 表示"不限定网关"（跨所有网关取最新）。 */
int Storage_QueryLatest(StorageDB *db, int gateway_id,
                        StorageQueryCB callback, void *userdata)
{
    if (!db->opened || !db->db) return -1;

    const char *sql;
    sqlite3_stmt *stmt = NULL;
    int rc;

    if (gateway_id < 0) {
        /* 跨网关：按时间倒排取最新（ORDER BY ts DESC LIMIT 1） */
        sql = "SELECT gateway_id, gateway_name, ts, temp, humidity "
              "FROM t_sensor_data ORDER BY ts DESC LIMIT 1;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    } else {
        sql = "SELECT gateway_id, gateway_name, ts, temp, humidity "
              "FROM t_sensor_data WHERE gateway_id=?1 "
              "ORDER BY ts DESC LIMIT 1;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
        if (rc == SQLITE_OK) sqlite3_bind_int(stmt, 1, gateway_id);   /* 只有 prepare 成功才能 bind */
    }

    if (rc != SQLITE_OK) return -1;

    int count = 0;
    SensorRecord rec;
    if (sqlite3_step(stmt) == SQLITE_ROW) {   /* 最多就一行（LIMIT 1） */
        read_row_to_record(stmt, &rec);
        count++;
        if (callback) callback(&rec, userdata);   /* 回调交给调用方处理（如序列化成 JSON） */
    }

    sqlite3_finalize(stmt);
    return count;
}

/* ---- 查询所有网关的最新数据 ----
 * 【SQL 思路】用子查询做"分组取每组最大时间"：
 *   内部 SELECT gateway_id, MAX(ts) GROUP BY gateway_id 得到"每个网关的最新时间"，
 *   再和原表 JOIN 回去补齐 temp/humidity。
 *   这是 SQL 里非常经典的"取每组最新一条"写法（不用窗口函数的老派写法）。 */
int Storage_QueryAllLatest(StorageDB *db, int max_gateways,
                           StorageQueryCB callback, void *userdata)
{
    if (!db->opened || !db->db) return -1;

    /* 子查询：每个 gateway_id 分组取最新时间 */
    const char *sql =
        "SELECT a.gateway_id, a.gateway_name, a.ts, a.temp, a.humidity "
        "FROM t_sensor_data a "
        "INNER JOIN ("
        "  SELECT gateway_id, MAX(ts) AS max_ts "
        "  FROM t_sensor_data "
        "  GROUP BY gateway_id"
        ") b ON a.gateway_id = b.gateway_id AND a.ts = b.max_ts "
        "ORDER BY a.gateway_id ASC "
        "LIMIT ?1;";

    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        LOG_ERROR("QueryAllLatest prepare failed: %s", sqlite3_errmsg(db->db));
        return -1;
    }

    sqlite3_bind_int(stmt, 1, max_gateways);   /* 限制返回的网关数 */

    int count = 0;
    SensorRecord rec;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        read_row_to_record(stmt, &rec);
        count++;
        /* 【约定】回调返回非 0 表示"调用方想中止遍历"（比如接收缓冲区已满），
         *   此时提前 break，避免无谓的继续处理。 */
        if (callback && callback(&rec, userdata) != 0) break;
    }

    sqlite3_finalize(stmt);
    return count;
}

/* ---- 查询某个网关的最近 N 条 ---- */
int Storage_QueryRecent(StorageDB *db, int gateway_id, int limit,
                        StorageQueryCB callback, void *userdata)
{
    if (!db->opened || !db->db) return -1;

    const char *sql;
    sqlite3_stmt *stmt = NULL;
    int rc;

    if (gateway_id < 0) {
        sql = "SELECT gateway_id, gateway_name, ts, temp, humidity "
              "FROM t_sensor_data ORDER BY ts DESC LIMIT ?1;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
        if (rc == SQLITE_OK) sqlite3_bind_int(stmt, 1, limit);
    } else {
        /* 两个占位符：?1=网关过滤，?2=条数限制 */
        sql = "SELECT gateway_id, gateway_name, ts, temp, humidity "
              "FROM t_sensor_data WHERE gateway_id=?1 "
              "ORDER BY ts DESC LIMIT ?2;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
        if (rc == SQLITE_OK) {
            sqlite3_bind_int(stmt, 1, gateway_id);
            sqlite3_bind_int(stmt, 2, limit);
        }
    }

    if (rc != SQLITE_OK) return -1;

    int count = 0;
    SensorRecord rec;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        read_row_to_record(stmt, &rec);
        count++;
        if (callback && callback(&rec, userdata) != 0) break;
    }

    sqlite3_finalize(stmt);
    return count;
}

/* ---- 查询时间范围 ----
 * 用于仪表盘画曲线：给定 [begin, end] 区间，按时间升序返回全部点。
 * 升序(ASC)是刻意的——画图时需要时间从早到晚，直接拿来用。 */
int Storage_QueryRange(StorageDB *db, int gateway_id, time_t begin, time_t end,
                       StorageQueryCB callback, void *userdata)
{
    if (!db->opened || !db->db) return -1;

    const char *sql;
    sqlite3_stmt *stmt = NULL;
    int rc;

    if (gateway_id < 0) {
        sql = "SELECT gateway_id, gateway_name, ts, temp, humidity "
              "FROM t_sensor_data WHERE ts >= ?1 AND ts <= ?2 "
              "ORDER BY ts ASC;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
        if (rc == SQLITE_OK) {
            sqlite3_bind_int64(stmt, 1, (sqlite3_int64)begin);
            sqlite3_bind_int64(stmt, 2, (sqlite3_int64)end);
        }
    } else {
        sql = "SELECT gateway_id, gateway_name, ts, temp, humidity "
              "FROM t_sensor_data WHERE gateway_id=?1 AND ts >= ?2 AND ts <= ?3 "
              "ORDER BY ts ASC;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
        if (rc == SQLITE_OK) {
            sqlite3_bind_int(stmt, 1, gateway_id);
            sqlite3_bind_int64(stmt, 2, (sqlite3_int64)begin);
            sqlite3_bind_int64(stmt, 3, (sqlite3_int64)end);
        }
    }

    if (rc != SQLITE_OK) return -1;

    int count = 0;
    SensorRecord rec;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        read_row_to_record(stmt, &rec);
        count++;
        if (callback && callback(&rec, userdata) != 0) break;
    }

    sqlite3_finalize(stmt);
    return count;
}

/* ---- 总记录数 ---- */
int Storage_GetCount(StorageDB *db)
{
    if (!db->opened || !db->db) return -1;

    const char sql[] = "SELECT COUNT(*) FROM t_sensor_data;";
    sqlite3_stmt *stmt = NULL;

    if (sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL) != SQLITE_OK) {
        return -1;
    }

    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
        count = sqlite3_column_int(stmt, 0);   /* 聚合查询只有一列一行的结果 */
    }
    sqlite3_finalize(stmt);
    return count;
}

/* ---- 清理旧数据 ----
 * 按"保留天数"删除过期记录：cutoff = 当前时间 - days*86400，
 * 删除所有 ts < cutoff 的行。86400 = 一天的秒数。 */
int Storage_CleanOld(StorageDB *db, uint32_t days)
{
    if (!db->opened || !db->db || days == 0) return 0;   /* days=0 表示永久保留，不删 */

    time_t cutoff = time(NULL) - (time_t)days * 86400;

    const char sql[] = "DELETE FROM t_sensor_data WHERE ts < ?1;";

    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        LOG_ERROR("CleanOld prepare failed: %s", sqlite3_errmsg(db->db));
        return -1;
    }

    sqlite3_bind_int64(stmt, 1, (sqlite3_int64)cutoff);

    rc = sqlite3_step(stmt);
    /* 【重要】sqlite3_changes 返回"最近一条语句影响的行数"。
     *   必须在 finalize 之前取（finalize 后信息可能失效），这就是下面这句放在这里的原因。 */
    int deleted = sqlite3_changes(db->db);
    sqlite3_finalize(stmt);

    if (deleted > 0) {
        LOG_INFO("Cleaned %d old records (before %ld)", deleted, (long)cutoff);
        /* 【WAL 维护】删除大量数据后，做一次 checkpoint 把 WAL 文件收缩回去，
         *   TRUNCATE 模式会把 WAL 文件截断到 0 字节，释放磁盘空间。
         *   否则 WAL 文件会一直很大，看起来很吓人。 */
        sqlite3_exec(db->db, "PRAGMA wal_checkpoint(TRUNCATE);", NULL, NULL, NULL);
    }

    return deleted;
}

/* ================================================================
 * 告警事件函数实现
 * ================================================================ */

/* ---- 插入告警事件 ----
 * 返回新记录的 rowid（>0 成功），供后续"恢复"时 UPDATE 使用。 */
int Storage_InsertAlarm(StorageDB *db, AlarmRecord *rec)
{
    if (!db->opened || !db->db) return -1;

    const char *sql =
        "INSERT INTO t_alarm_events "
        "(gateway_id, gateway_name, alarm_type, threshold_value, "
        " trigger_value, trigger_ts, recover_ts, status) "
        "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8);";

    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        LOG_ERROR("InsertAlarm prepare: %s", sqlite3_errmsg(db->db));
        return -1;
    }

    /* 8 个参数依次绑定，注意顺序要和 SQL 里的列顺序完全对应 */
    sqlite3_bind_int(stmt, 1, rec->gateway_id);
    sqlite3_bind_text(stmt, 2, rec->gateway_name, -1, SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, rec->alarm_type, -1, SQLITE_STATIC);
    sqlite3_bind_double(stmt, 4, rec->threshold_value);
    sqlite3_bind_double(stmt, 5, rec->trigger_value);
    sqlite3_bind_int64(stmt, 6, (sqlite3_int64)rec->trigger_ts);
    sqlite3_bind_int64(stmt, 7, (sqlite3_int64)rec->recover_ts);
    sqlite3_bind_text(stmt, 8, rec->status, -1, SQLITE_STATIC);

    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        LOG_ERROR("InsertAlarm step: %s", sqlite3_errmsg(db->db));
        sqlite3_finalize(stmt);
        return -1;
    }

    /* 取刚插入那行的自增 id */
    int new_id = (int)sqlite3_last_insert_rowid(db->db);
    sqlite3_finalize(stmt);
    return new_id;
}

/* ---- 更新告警恢复 ----
 * 把指定告警从 active 改成 recovered，并记录恢复时间。 */
int Storage_UpdateAlarmRecover(StorageDB *db, int alarm_id, time_t recover_ts)
{
    if (!db->opened || !db->db || alarm_id <= 0) return -1;

    const char *sql =
        "UPDATE t_alarm_events "
        "SET status = 'recovered', recover_ts = ?1 "
        "WHERE id = ?2;";

    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        LOG_ERROR("UpdateAlarmRecover prepare: %s", sqlite3_errmsg(db->db));
        return -1;
    }

    sqlite3_bind_int64(stmt, 1, (sqlite3_int64)recover_ts);
    sqlite3_bind_int(stmt, 2, alarm_id);

    rc = sqlite3_step(stmt);
    sqlite3_finalize(stmt);
    return (rc == SQLITE_DONE) ? 0 : -1;   /* UPDATE 成功也是 SQLITE_DONE */
}

/* ---- 读一行到 AlarmRecord ----
 * 列顺序与 Storage_QueryAlarms 里的 SELECT 严格对应（共 9 列）。 */
static int read_alarm_row(sqlite3_stmt *stmt, AlarmRecord *rec)
{
    int ncol = sqlite3_column_count(stmt);

    memset(rec, 0, sizeof(*rec));   /* 整体清零，避免残留 */
    if (ncol >= 1) rec->alarm_id       = sqlite3_column_int(stmt, 0);     /* id */
    if (ncol >= 2) rec->gateway_id     = sqlite3_column_int(stmt, 1);     /* gateway_id */
    if (ncol >= 3) {
        const char *s = (const char *)sqlite3_column_text(stmt, 2);       /* gateway_name */
        if (s) snprintf(rec->gateway_name, sizeof(rec->gateway_name), "%s", s);
    }
    if (ncol >= 4) {
        const char *s = (const char *)sqlite3_column_text(stmt, 3);       /* alarm_type */
        if (s) snprintf(rec->alarm_type, sizeof(rec->alarm_type), "%s", s);
    }
    if (ncol >= 5) rec->threshold_value = sqlite3_column_double(stmt, 4); /* threshold_value */
    if (ncol >= 6) rec->trigger_value   = sqlite3_column_double(stmt, 5); /* trigger_value */
    if (ncol >= 7) rec->trigger_ts      = (time_t)sqlite3_column_int64(stmt, 6);  /* trigger_ts */
    if (ncol >= 8) rec->recover_ts      = (time_t)sqlite3_column_int64(stmt, 7);  /* recover_ts */
    if (ncol >= 9) {
        const char *s = (const char *)sqlite3_column_text(stmt, 8);       /* status */
        if (s) snprintf(rec->status, sizeof(rec->status), "%s", s);
    }
    return 0;
}

/* ---- 查询告警事件 ----
 * 【注意：这里是本文件唯一"动态拼 SQL"的地方】
 *   因为过滤条件是可选的（网关/时间/状态可以任意组合），无法用一条固定 SQL 表达。
 *   这里拼进去的都是**数值**（gateway_id / 时间戳 / limit 是整数），
 *   不存在字符串注入风险——这也是为什么 status_filter 要由调用方保证是白名单值。
 * 【改进方向】更严谨的写法是：不管有没有条件都拼出固定的占位符，
 *   用 bind 绑定值；或者对字符串参数做白名单校验。 */
int Storage_QueryAlarms(StorageDB *db, int gateway_id,
                        time_t begin, time_t end,
                        const char *status_filter, int limit,
                        AlarmQueryCB callback, void *userdata)
{
    if (!db->opened || !db->db) return -1;

    /* 动态拼 SQL */
    char sql[512];
    int  pos = 0;   /* 已写入的字符数，snprintf 的返回值就是"本该写入的长度" */
    /* WHERE 1=1 是个小技巧：让后面每个条件都能统一写成 " AND xxx"，
     * 不用判断"这是第一个条件吗"（第一个不加 AND）。 */
    pos += snprintf(sql + pos, sizeof(sql) - pos,
        "SELECT id, gateway_id, gateway_name, alarm_type, "
        "threshold_value, trigger_value, trigger_ts, recover_ts, status "
        "FROM t_alarm_events WHERE 1=1");

    if (gateway_id >= 0)
        pos += snprintf(sql + pos, sizeof(sql) - pos,
                        " AND gateway_id = %d", gateway_id);
    if (begin > 0)
        pos += snprintf(sql + pos, sizeof(sql) - pos,
                        " AND trigger_ts >= %ld", (long)begin);
    if (end > 0)
        pos += snprintf(sql + pos, sizeof(sql) - pos,
                        " AND trigger_ts <= %ld", (long)end);
    if (status_filter)
        pos += snprintf(sql + pos, sizeof(sql) - pos,
                        " AND status = '%s'", status_filter);

    pos += snprintf(sql + pos, sizeof(sql) - pos,
                    " ORDER BY trigger_ts DESC LIMIT %d", limit);

    sqlite3_stmt *stmt = NULL;
    int rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        LOG_ERROR("QueryAlarms prepare: %s", sqlite3_errmsg(db->db));
        return -1;
    }

    int count = 0;
    AlarmRecord rec;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        read_alarm_row(stmt, &rec);
        count++;
        if (callback && callback(&rec, userdata) != 0) break;
    }

    sqlite3_finalize(stmt);
    return count;
}

/* ---- 查询活跃告警数量 ----
 * 前端仪表盘用它做"告警指示灯"：>0 就亮红灯。
 * 失败时返回 0（而不是 -1），这样前端逻辑简单：直接把返回值当数量用。 */
int Storage_GetActiveAlarmCount(StorageDB *db, int gateway_id)
{
    if (!db->opened || !db->db) return 0;

    const char *sql;
    sqlite3_stmt *stmt = NULL;
    int rc;

    if (gateway_id < 0) {
        sql = "SELECT COUNT(*) FROM t_alarm_events WHERE status = 'active';";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
    } else {
        sql = "SELECT COUNT(*) FROM t_alarm_events "
              "WHERE status = 'active' AND gateway_id = ?1;";
        rc = sqlite3_prepare_v2(db->db, sql, -1, &stmt, NULL);
        if (rc == SQLITE_OK) sqlite3_bind_int(stmt, 1, gateway_id);
    }

    if (rc != SQLITE_OK) return 0;

    int count = 0;
    if (sqlite3_step(stmt) == SQLITE_ROW)
        count = sqlite3_column_int(stmt, 0);

    sqlite3_finalize(stmt);
    return count;
}

/* ---- 关闭 ---- */
void Storage_Close(StorageDB *db)
{
    if (db->db) {
        LOG_INFO("Storage closed: %s, total inserts=%u",
                 db->path, db->insert_count);
        sqlite3_close(db->db);
        db->db = NULL;   /* 置空防止二次 close（double free） */
    }
    db->opened = 0;
}
