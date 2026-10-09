# linux-host — Modbus 网关上位机

Linux 端的采集与运维程序：并发采集多个网关节点的温湿度数据，写入本地 SQLite，
提供 REST API 与 Web 仪表盘，并支持向网关远程下发固件升级。

## 特性

- **多网关并发采集**：1 个 epoll + N 个 timerfd + 1 个 eventfd，每个网关按自己的
  周期采集，互不阻塞（默认上限 16 个网关）。
- **断线重连 + 故障隔离**：单个网关掉线自动重试；一个网关断线不影响其他网关。
- **本地存储**：SQLite3（WAL 模式、预编译语句绑定、事务批量提交），按
  `retain_days` 自动清理。
- **HTTP 服务**：手写 HTTP/1.0 服务器（无第三方框架），8 条路由 + Chart.js 仪表盘。
- **报警去抖**：4 通道对称计数，连续 N 次超限才触发告警，避免抖动刷屏。
- **配置热加载**：`kill -HUP <pid>` 运行时重载配置，增删改网关即时生效。
- **OTA 下发**：TCP 506 推送固件，START→DATA×N→FINISH 两阶段 ACK。

## 目录结构

```
linux-host/
├── src/                          业务源码（共 5069 行）
│   ├── main.c               267  入口：解析参数、注册信号、启动 poller/http、热加载
│   ├── config.c/.h          383  配置解析（手写 INI 流式解析，零外部依赖）
│   ├── poller.c/.h          585  多网关并发采集（epoll + timerfd + eventfd）
│   ├── gateway.c/.h         304  单个网关的连接、采集、报警联动
│   ├── modbus_master.c/.h   348  Modbus-TCP 主站收发（libmodbus 封装）
│   ├── storage.c/.h         935  SQLite 存储（数据表 / 报警表 / 迁移 / 批量插入）
│   ├── alarm.c/.h           293  报警阈值判断与去抖（4 通道对称计数）
│   ├── http_server.c/.h     1200 HTTP 服务器（8 条路由 + 静态文件 + multipart 上传）
│   ├── ota.c/.h             483  OTA 下发状态机（异步 pthread）
│   └── logger.c/.h          271  日志（终端彩色 + 文件 + 多级 + 线程安全）
├── include/                       OTA 协议（与 STM32 侧共用同一定义）
│   ├── ota_protocol_def.h   177  命令/数据/状态帧结构、magic、错误码、Flash 地址
│   ├── ota_ctrl_block.h     175  控制块读写（static inline，平台适配宏）
│   └── ota_crc.c/.h         217  CRC16/Modbus + CRC32/ZIP 查表实现
├── tools/
│   ├── fake_gateway.py      274  测试用 Modbus-TCP 从站（仅供回归测试构造多节点）
│   ├── pack_firmware.py     159  把 .bin 打包成带头部与 CRC 的 .ota
│   └── test_all.sh          344  一条龙回归测试（7 个用例）
├── web/
│   ├── dashboard.html           单页仪表盘（Chart.js，无构建步骤）
│   └── vendor/chart.umd.min.js  本地 Chart.js（离线可用，不依赖 CDN）
├── deploy/
│   ├── modbus-backend.service   systemd 服务单元
│   └── modbus-backend.logrotate 日志轮转配置
├── firmware/                     固件存放目录（.ota）
├── data/                         运行时数据库与日志（不进版本库）
├── config.ini                    多网关配置
└── Makefile
```

## 架构与线程模型

程序有 **3 个执行单元**：

```
  主线程                       HTTP 线程                   OTA 线程（按需创建）
  ────────                     ──────────                  ────────────────────
  epoll_wait(1s 超时)          accept + process_client     连接网关:506
    ├─ timerfd 到期 ─────────► Gateway_PollOnce           发 START → 等 10s ACK
    │    ├─ Modbus_ReadHoldingRegs                         发 DATA × N（每块 512B）
    │    ├─ Storage_Open/Insert/Close                      发 FINISH → 等 6s ACK
    │    └─ Alarm_Check ─────► 写告警表                    轮询网关:502 确认上线
    ├─ stop_fd (eventfd) ────► 退出
    └─ SIGHUP 置 g_reload ───► Poller_Reload（增删网关定时器）

      ↓ 共享
   ┌─────────────────────────────────────────────────────────┐
   │ SQLite (WAL) ← 主线程写数据/告警，HTTP 线程只读查询      │
   │ logger 全局锁 ← 三个线程都会写日志                       │
   └─────────────────────────────────────────────────────────┘
```

- **主线程**跑 `Poller_RunOnce`；`epoll_wait` 给 1s 超时是为了给主循环"喘息点"
  检查 `g_running` / `g_reload` 标志，否则无事件时会无限阻塞，连 Ctrl+C 都不响应。
- **HTTP 线程**由 `HttpServer_Start` 创建（`SOCK_NONBLOCK` + `SO_REUSEADDR` +
  `INADDR_ANY` + backlog 10 + epoll 500ms 超时轮询 `running` 标志）。
- **OTA 线程**由 `OTA_StartAsync` 按需 `pthread_create`，同一时刻只支持一个
  OTA 任务（全局 `g_ota_mutex` 保护）；线程全程不持锁，靠"关键字段单次写入原子"。
- 退出顺序**先 `HttpServer_Stop` → 再 `Poller_Stop` → 最后 `LOG_Close`**，
  保证最后一条关闭日志能落盘。

## 核心模块

### `main.c` —— 入口与热加载

- `signal_handler` 只置标志（信号处理里只能做异步信号安全操作，不能 `malloc` /
  `printf` / 加锁）。
- `setup_signals` 用 `sigaction`（比老式 `signal()` 可控），**不加 `SA_RESTART`**，
  以便 `epoll_wait` 能被信号打断；注册 `SIGINT` / `SIGTERM` / `SIGHUP`，
  另外 `signal(SIGPIPE, SIG_IGN)`（HTTP 客户端提前断开是常态，不忽略会杀掉进程）。
- `do_reload()`：堆分配 `new_cfg`（Poller 保存的是指针，栈变量会悬空）→
  `Config_Load` 失败则 `free` 并保留旧配置 → `LOG_SetLevel` 运行时生效 →
  `Poller_Reload` → **确认无人再引用旧配置后才 `free(g_prev_cfg)`**。
- 启动横幅打印每个网关的 `name / host / port / interval` 与 HTTP 地址。

### `poller.c/.h` —— epoll 多网关调度

- 一个 epoll 管理 **N + 1 个 fd**：N 个网关各一个 timerfd，加 1 个 eventfd 作停止信号。
- `Poller_Init` 只做准备（注意 `Gateway_Init` 内部 `memset` 会清掉 `index`，
  所以 `index` 必须在它之后赋值）。
- `Poller_RunOnce`：
  - `EINTR` 返回 0 不算错误；
  - `stop_fd` 与 timerfd 都必须 `read` 掉，**否则 100% CPU busy-loop**；
  - fd → 网关用 O(N) 线性查找（上限 16，可接受；理论可用 `epoll_data.ptr` 优化到 O(1)）；
  - 定期清理用 `s_total_polls % 100 < handled` 防跨过倍数漏清。
- `Poller_Reload` 三段式：公共段用 `GatewayConfig_Equals` 比对，不变则只换 `cfg`
  指针并保持连接；减少的段先 `EPOLL_CTL_DEL` 再 `close`；新增段做初始化。
- `Poller_PrintStats`：`last_temp` 放大 10 倍存 `int16`，打印时 `/ 10.0`。

### `gateway.c/.h` —— 单网关采集

- `Gateway_Init`：`memset` → `Alarm_Init` → 持 cfg 指针 → `timer_fd = -1` →
  `poll_interval` 小于 1 时强制为 5 → `Modbus_Connect` 失败只 WARN 不阻塞启动。
- `Gateway_StartTimer`：`timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK)`，
  `it_value = 1 + ctx->index % 5` **错峰抖动**，避免所有网关同一瞬间同时采集。
- `Gateway_PollOnce`：未连接先 `Modbus_Reconnect` → `Modbus_ReadHoldingRegs` →
  失败累计 `err_count >= 3` 强制断开（下轮机会性重连）→ 原始值 int16 强转后
  `/10.0` 得物理量（**必须 int16 才能表达负温**）→ `Storage_Open` / `Insert` /
  `Close` 短连接 → `Alarm_Check`（**必须在入库之后**，告警记录要关联入库数据）→ `LOG_INFO`。

### `modbus_master.c/.h` —— libmodbus 封装

- `Modbus_Connect` 里**必须设 1s 响应/字节超时**，否则一个掉线网关就能卡死采集主循环。
- 连接失败用 `modbus_strerror`（不是 `strerror`）取错误串，并 `modbus_free` + 置 NULL。
- `Modbus_Reconnect` 幂等（已连接即返 0），且处理 **`EINPROGRESS`**：非阻塞 socket
  下 `connect` 立刻返 -1 不代表失败，用 `poll(POLLOUT, 1000ms)` 等可写，再
  `getsockopt(SO_ERROR)` 取延迟错误码，为 0 才算握手成功。
- `handle_error` **先 `int err = errno` 存下来**（errno 会被后续调用覆盖）——
  `ETIMEDOUT / ECONNRESET / EPIPE / ECONNREFUSED / ENOTCONN` 置 `connected = 0`，
  其它只记日志；统一 `err_count++`。
- `Modbus_ReadHoldingRegs` 必须用 **`rc != count`** 判断（成功返回实际读到个数），
  成功则 `err_count = 0` —— 这就是"连续 3 次失败"中"连续"二字的实现。

### `storage.c/.h` —— SQLite 存储

- 两张表：

  ```sql
  t_sensor_data(id, gateway_id, gateway_name, ts, temp, humidity)   -- idx_ts, idx_gw_ts
  t_alarm_events(id, gateway_id, gateway_name, alarm_type, threshold_value,
                 trigger_value, trigger_ts, recover_ts, status)      -- idx_alarm_gw/ts/stat
  ```

- 告警表**一条记录承载全生命周期**：触发时插入 `status='active'`，恢复时
  `UPDATE` 回填 `recover_ts` 并改 `status='recovered'`（而不是插入第二条）。
- `migrate_schema` 用 `sqlite_master` 查表 + `column_exists` 用 `PRAGMA table_info`，
  配合 `ALTER TABLE ADD COLUMN` 做幂等 v1→v2 迁移（新增 `gateway_id/gateway_name`）。
- `Storage_Open`：`PRAGMA journal_mode=WAL`（非致命）→ 迁移 → 建 v2 表 → 建告警表。
- 全程预编译 + `sqlite3_bind_*` 防注入，**每条 prepare 必配 finalize**。
- `Storage_InsertBatch`：`BEGIN` / `COMMIT` 事务 + 只 prepare 一次 + `sqlite3_reset`
  复用，把 N 次 fsync 压成 1 次。
- 查询：`QueryLatest`（`INNER JOIN (SELECT gateway_id, MAX(ts) GROUP BY gateway_id)`
  分组取每组最新）/ `QueryAllLatest` / `QueryRecent` / `QueryRange`（ASC 供画图）/
  `Storage_GetCount`。
- `Storage_CleanOld`：`cutoff = now - days*86400`；`sqlite3_changes` **必须在
  finalize 前取**；删完 `PRAGMA wal_checkpoint(TRUNCATE)` 收缩 WAL。
- `QueryAlarms` 是全文件**唯一动态拼 SQL** 的地方，拼进去的都是数值，且
  `status_filter` 白名单由调用方保证；用 `WHERE 1=1` 技巧统一拼 ` AND xxx`。
- `read_row_to_record` 按 `ncol >= N` 兼容不同 SELECT 列数。

### `alarm.c/.h` —— 报警去抖

- 4 个通道：`ALARM_TEMP_HIGH / TEMP_LOW / HUM_HIGH / HUM_LOW`（`ALARM_TYPE_COUNT = 4`）。
- **约定 `threshold <= 0.0` 表示该通道不启用**（`is_over_limit` 直接返 0）。
- `check_channel` 对称设计：超限则 `over_count++` 且 `normal_count = 0`，达到
  `debounce` 且 `!active` 才触发并把 `over_count` 清零；正常则反之。
  `debounce` 兜底为 3（防 `alarm_debounce == 0` 一超就报）。
- `trigger_alarm` 里 `Storage_Open` 失败**直接 return 不置 active**（下轮重试，
  宁可重试不可漏报）；成功拿到 `alarm_id > 0` 才置 `ch->alarm_id / ch->active`。
- 无锁 —— 每个 `GatewayCtx` 只被主线程访问，天然无并发。
- 写入 DB 用英文标识（`temp_high` 等，见 `type_names[]`），日志用中文（`type_desc[]`），
  两者都用**指定初始化器** `[ENUM] = "..."` 与枚举绑定。

### `http_server.c/.h` —— 手写 HTTP 服务

- HTTP/1.0 + `Connection: close`（不搞 keep-alive），单线程 accept 后同步处理。
- 缓冲：`MAX_EVENTS = 32`、`REQUEST_BUF = 65536`、`RESPONSE_BUF = 262144`
  （要装下 208KB 的 Chart.js）、`MAX_PATH = 256`。
- 用 `memfind`（**必须带长度**）而非 `strstr` 搜索——二进制 body 里含 `\0`。
- `url_decode`（`%XX` + `'+'`→空格）、`get_query_param`（`strncasecmp` 忽略大小写）。
- `process_client` **循环 `read` 直到 headers + `Content-Length` body 收全**
  （一次 read 拿不全），`sscanf("%15s %255s %15s")` 解析请求行，`?` 截断出 query。
- 路由表：

  | 方法 | 路径 | 说明 |
  |---|---|---|
  | GET | `/` | 仪表盘（内部映射到 `/dashboard.html`） |
  | GET | `/api/latest` | 所有网关最新数据；**无数据的网关填占位记录**，保证前端列表不缺项 |
  | GET | `/api/history?gateway=N&hours=H` | 历史数据，H 限 1~720（30 天） |
  | GET | `/api/alarms?gateway=N&limit=L&status=S` | 报警事件，L 限 1~500 |
  | POST | `/api/ota/upload` | multipart 上传 `.ota` 并异步启动升级 |
  | GET | `/api/ota/status` | 升级状态（枚举转 `idle/sending/verifying/success/failed/cancelled/pending_verify`） |
  | POST | `/api/ota/cancel` | 取消升级 |
  | GET | `/api/history` `/api/latest` 等 | 见上 |
  | 其它 | * | 404 |

- `build_ota_status_json` 把 OTA 状态枚举转成字符串给前端。
- `parse_multipart_upload`：`extract_boundary` 从 `Content-Type` 取 boundary，
  按 `--boundary` / `--boundary--` 切分，从 `Content-Disposition` 取 `name="gateway"`
  与 `name="file"`，落盘为 `upload_<ts>_<gw>.ota`（`O_WRONLY|O_CREAT|O_TRUNC` 0644，
  循环写防短写，失败 unlink）。
- JSON 输出用 `fmemopen` 内存流 + `json_escape`（FILE* 流式转义）+ `JsonCbCtx`
  控制逗号；`copy_record_cb` 返 1 可中止遍历。
- `GET /api/latest` 用 `malloc(RESPONSE_BUF)` 堆分配响应缓冲（262KB 直接放栈会爆栈）。

### `ota.c/.h` —— OTA 下发（上位机侧）

- `OTA_StartAsync`：锁内只拦 `SENDING / VERIFYING` 运行态，`FAILED / CANCELLED /
  PENDING_VERIFY` 等历史终态允许覆盖重启；`read` 固件头必须校验
  `== sizeof(hdr)`（短读检测）；检查 `ntohl(hdr.magic) == 0x504A3142`。
- 状态查询 `OTA_GetStatus` **在锁内 `memcpy` 一份快照**返回，避免调用方无锁读
  与写线程竞争。
- `OTA_Cancel` 只置 `OTA_STATUS_CANCELLED` 标志 —— 协作式取消比 `pthread_cancel` 安全。
- `ota_thread_func` 流程：
  1. `open` + `lseek` 跳过 16 字节头；
  2. `tcp_connect_timeout(host, port, 5000)`；
  3. 发 START（`total_size` / `total_crc` / `fw_version`），**等 ACK 给 10s**
     （设备端 START 会同步全量擦除暂存区，最坏 ~4.4s）；
  4. DATA 循环：每块 512B `read`，包布局 `magic(4)+block_num(4)+data_len(2)+
     block_crc(2)+data(N)`；`RSP_ACK` 则清空 `busy_total_ms` 计入成功；
     **`RSP_BUSY` 等 200ms 重发且不给 `retry` 计数**（累计 ≥ 30s 才判失败）；
     `ERR_ERASE / ERR_WRITE / ERR_OOB` 是硬件级错误直接 `goto FAIL`；
     每轮重试前检查 `CANCELLED`；
  5. FINISH 3 次重试（每次给 6s）；
  6. 轮询网关重启上线：`for (i < 15)` 每 2s 探一次 502 端口（最长 30s），
     连上则 `SUCCESS`，否则 `PENDING_VERIFY`（"升级完成但未确认上线"≠失败）；
  7. 统一 `FAIL:` 标签清理 `close(sock); close(fw_fd); status = FAILED`。
- `tcp_connect_timeout`：`setsockopt(SO_RCVTIMEO/SO_SNDTIMEO)`（毫秒拆成
  `tv_sec = ms/1000` + `tv_usec = (ms%1000)*1000`），`inet_pton` 而非 `inet_addr`
  （可重入），`connect` 失败要 `close(fd)` 防泄漏。
- `send_all` 循环写防短写；`recv_byte` 必须 `read(fd, byte, 1) == 1` 才算成功。
- `block_total = (fw_size + 511) / 512`（整数向上取整）。
- **已知限制**：目标 IP / 端口当前硬编码 `192.168.10.100:506`，未从 `config.ini`
  的 `[ota]` 段读取（代码中已标 TODO）。

### `config.c/.h` —— 配置解析

- 手写 `fgets` 流式 INI 解析，零外部依赖。
- 容量上限：`CONFIG_HOST_MAX = 64`、`CONFIG_NAME_MAX = 64`、
  `CONFIG_DB_PATH_MAX = 256`、`CONFIG_MAX_GATEWAYS = 16`。
- `GatewayConfig` 关键字段：`host / port / name / poll_interval_sec / slave_id /
  reg_start / reg_count / retry_count / retry_interval_ms / temp_high / temp_low /
  hum_high / hum_low / alarm_debounce`。
- `parse_gateway_section`：`[gateway]` 映射为 index 0（兼容旧格式）；
  `gateway.` 前缀用 `strtol` 且校验 `*end == '\0'`（防 `gateway.3abc`）；
  范围 1~16 返 `n-1`，不匹配返 -1。
- **动态扩展网关数**：`set_config_value` 遇到更大序号就把 `gateway_count` 抬上去
  （支持序号跳跃），因此配置文件里**无需写 `gateway_count`**。
- `set_gateway_value` 用 `strcasecmp` 大小写不敏感；`reg_start` 用
  `strtol(value, NULL, 0)` 自动识别 `0x` 前缀；**未知 key 静默忽略**（容错优先）。
- `Config_Load`：先 `Config_Default` 再逐项覆盖；`fopen` 失败用 `errno` +
  `strerror` 报具体原因；段头缺 `]`、`key=value` 缺 `=` 都只报错并 `continue`，
  不整体失败。
- 重要易错点（`config.ini` 头部也写了）：解析器**只识别整行注释，不剥离行内注释**。
  字符串键（`host / name / db_path / file`）的行尾注释会被当作值的一部分；
  数值行因 `atoi/atof` 遇非数字即停而"侥幸可用"——但建议统一把说明写在上一行。

### `logger.c/.h` —— 日志

- `LOG_Write(level, file, line, func, fmt, ...)` + 四个便捷宏自动带
  `__FILE__ / __LINE__ / __func__`；`LOG_ATTR_PRINTF` 用
  `__attribute__((format(printf, ...)))` 让编译器检查格式串。
- 静态单例状态 `g_log_fp / g_log_level / g_color / g_lock`
  （`PTHREAD_MUTEX_INITIALIZER` 静态初始化）。
- `ensure_dir` 递归创建多级目录（从 `dir+1` 起逐 `/` 截断 `mkdir(dir, 0755)`）。
- `LOG_Init` 用 `fopen(filepath, "a")` 追加模式 + **`setbuf(g_log_fp, NULL)`
  关闭用户态缓冲** → 进程崩溃也不丢日志。
- `LOG_Write`：级别过滤放最前（避免无谓的 `vsnprintf` 格式化）→
  `char msg[2048]` + `vsnprintf` → `strrchr(file, '/')` 取短文件名 →
  **加锁**（采集线程和 HTTP 线程都写日志，不加锁会交错乱码）→
  终端带 ANSI 颜色 `%-5s` 左对齐 + `fflush(stdout)`；文件不带颜色 + `fflush`。
- 级别：`DEBUG=0 / INFO=1 / WARN=2 / ERROR=3 / NONE=4`。

### `include/ota_protocol_def.h` —— 跨平台协议契约

- 字节序宏按平台分派：`STM32F103xB` 用 `__REV / __REV16`；
  `__linux__` / `_WIN32` 用 `htonl` / `htons` 系列；未知平台 `#error`。
- Magic：`OTA_CMD_MAGIC = 0x4F544143`（"OTAC"）、`OTA_DATA_MAGIC = 0x4F544150`
  （"OTAP"）、`OTA_STATUS_MAGIC = 0x4F544153`（"OTAS"）、
  `OTA_CTRL_MAGIC = 0xA5A5A5A5`、`FW_HEADER_MAGIC = 0x504A3142`（"PJ1B"）。
- 命令码 `CMD_START = 0x00 / FINISH = 0x01 / CANCEL = 0x02 / STATUS = 0x03`。
- 响应码 `RSP_ACK = 0x06 / RSP_NACK = 0x15 / RSP_ABORT = 0x18 / RSP_BUSY = 0x19 /
  RSP_ERR_ERASE = 0x1A / RSP_ERR_WRITE = 0x1B / RSP_ERR_OOB = 0x1C`。
- 错误码 `ERR_NONE / ERR_CTRL_ERASE / ERR_CTRL_WRITE / ERR_FW_CRC /
  ERR_FLASH_WRITE / ERR_STAGING_CRC / ERR_SIZE_INVALID / ERR_OOB`。
- Flash 布局常量：`BOOTLOADER_ADDR = 0x08000000`、`APP_ADDR = 0x08004000`、
  `APP_MAX_SIZE = 0xC000`（48KB）、`CTRL_BLOCK_ADDR = 0x007000`、
  `STAGING_ADDR = 0x010000`、`ROLLBACK_BACKUP_ADDR = 0x01C000`、
  `LOG_AREA_ADDR = 0x080000`。
- `#pragma pack(1)` 结构体：`OTACmd`（16B）、`OTAData`（12+N）、
  `OTAStatusResp`（12B）、`OTACtrlBlock`（19B）、`FirmwareHeader`（16B）。
- **`CTRL_CRC_LEN = 12`**：CRC16 只算前 12 字节，**不含 `fail_flag[3]` 和
  `last_error`** —— 否则每次 `MarkOneFail` 后 CRC 就不匹配，会破坏重试机制。

### `include/ota_crc.c/.h` —— 双 CRC

- `CRC32_ZIP(data, len)`：多项式 `0x04C11DB7`、初值 `0xFFFFFFFF`、出异或
  `0xFFFFFFFF`（等价于反射多项式 `0xEDB88320`）；标准测试向量 `"123456789"`
  → `0xCBF43926`。
- `CRC32_ZIP_Update(crc, data, len)`：**不做初值设定也不取反**，供流式分段累加
  （调用方首传 `0xFFFFFFFF`、末次手工取反）。
- `CRC16_Modbus(data, len)`：多项式 `0x8005`、初值 `0xFFFF`、**不取反**；
  测试向量 → `0x4B37`。
- 为什么两个 CRC：**分层校验**——块级用轻量 CRC16（每 512B 一次，快速丢坏包），
  整体用强 CRC32（FINISH 时一次性重算，抗误码能力强）。

### `include/ota_ctrl_block.h` —— 控制块读写

- 全 `static inline`，要求使用方在使用前 define **平台适配宏**
  `OTA_FLASH_READ / OTA_FLASH_WRITE / OTA_FLASH_SECTOR_ERASE /
  OTA_FLASH_IS_BUSY / OTA_FLASH_WAIT_BUSY`（指向 `BSP_W25Q64_*`）。
- `OTA_ReadCtrlBlock`：读 19B → 校验 magic（返 -1）→ 校验 CRC16（返 -2）→ 0。
- `OTA_EraseAndVerifyCtrlSector`：擦扇区 → 等 `IS_BUSY` 归零（`timeout = 2000`，
  每次迭代调 `WAIT_BUSY` 喂狗）→ 回读 19B 逐一校验全 `0xFF`。
- `OTA_WriteCtrlBlock`：**先写 `[4~18]` 共 15 字节（firmware_size ~ ctrl_crc16），
  最后写 `[0~3]` magic** —— 这样断电也不会出现"magic 有效但数据只写了一半"；
  写后回读 `memcmp`，不一致则写 0 清 magic 并返 -3。
- `GetFailCount` 数 `fail_flag[3]` 中 ≠0xFF 的个数；`MarkOneFail` 内存里把第一个
  `0xFF` 改 `0x00`。
- `OTA_MarkFail(err, fw_size, fw_crc)`：若 `OTA_ReadCtrlBlock` 失败则
  `memset(&ctrl, 0xFF)` 全新创建（含 `last_error` 与 CRC）再写；成功则仅
  `MarkOneFail` + 只改写 `CTRL_BLOCK_ADDR + 12` 起的 3 字节 fail_flag
  （1→0 无需擦除，也不更新 CRC）。

## 构建与运行

**依赖**（Ubuntu/Debian）：

```bash
sudo apt install gcc make libmodbus-dev libsqlite3-dev
```

**Makefile 要点**：`LDFLAGS := -lmodbus -lsqlite3 -lrt -lpthread`（顺序敏感，
GNU ld 单遍解析，被依赖的库放后面）；`-MMD -MP` 自动生成 `.d` 依赖文件，
改头文件会自动重编；`ota_crc.c` 位于 `include/` 而非 `src/`，通配符扫不到，
故单独加了一条编译规则。

```bash
make            # 增量编译 → bin/modbus-backend
make clean      # 清理 obj/ 与 bin/
make run        # 编译后启动（自动创建 data/）
make install    # 拷贝到 /usr/local/bin/（systemd 服务用此路径）
```

**启动**：

```bash
./bin/modbus-backend -c config.ini      # 指定配置
./bin/modbus-backend -h                 # 查看用法
```

**信号**：

| 信号 | 行为 |
|---|---|
| `SIGINT` / `SIGTERM` | 优雅退出（停 HTTP → 停采集 → 刷日志） |
| `SIGHUP` | 重载配置（增/删/改网关，无需重启进程） |
| `SIGPIPE` | 被忽略（HTTP 客户端提前断开是常态） |

## HTTP API 响应格式

```jsonc
// GET /api/latest
{ "gateways": [ { "gateway_id": 1, "gateway_name": "一楼机房",
                  "temperature": 26.3, "humidity": 55.0, "ts": 1735689600 } ] }

// GET /api/history?gateway=1&hours=24
{ "gateway_id": 1, "gateway_name": "一楼机房", "hours": 24,
  "records": [ { "gateway_id": 1, "gateway_name": "一楼机房",
                 "ts": 1735689600, "temperature": 26.3, "humidity": 55.0 } ] }

// GET /api/alarms?gateway=1&limit=20&status=active
{ "active_count": 1,
  "alarms": [ { "gateway_id": 1, "gateway_name": "一楼机房",
                "alarm_type": "temp_high", "threshold_value": 35.0,
                "trigger_value": 36.2, "trigger_ts": 1735689600,
                "recover_ts": 0, "status": "active" } ] }

// GET /api/ota/status
{ "gateway_id": 1, "status": "sending", "progress_pct": 42, ... }
```

前端 `dashboard.html` 每 10s 用 `Promise.all` 并发拉这三个接口（任一失败不影响
另外两个），渲染网关卡片 + 双 Y 轴折线图 + 告警面板；告警类型英文枚举在
`ALARM_LABELS` 里映射为带图标中文文案。

## 配置

`config.ini` 按段组织：

| 段 | 内容 |
|---|---|
| `[gateway.N]` | 网络（host/port）、名称、采集周期、从机地址、寄存器范围、重试策略、4 个报警阈值与去抖次数 |
| `[storage]` | `db_path` / `retain_days` |
| `[log]` | `level` / `file` / `color` |
| `[http]` | `port`（默认 8080） |
| `[ota]` | `firmware_dir` / `ota_port`（默认 506） |

> **重要**：解析器只识别**整行注释**，不剥离行内注释。字符串键
> （`host` / `name` / `db_path` / `file`）行尾写注释会被当成值的一部分，
> 说明请写在上一行；数值行虽"侥幸可用"，也建议统一放上一行。

## 部署

```bash
sudo apt install gcc make libmodbus-dev libsqlite3-dev

make && sudo make install
sudo mkdir -p /etc/modbus-backend /var/lib/modbus-backend
sudo cp config.ini /etc/modbus-backend/

sudo cp deploy/modbus-backend.service /etc/systemd/system/
sudo cp deploy/modbus-backend.logrotate /etc/logrotate.d/modbus-backend
sudo systemctl daemon-reload
sudo systemctl enable --now modbus-backend

systemctl status modbus-backend          # 查看状态
journalctl -u modbus-backend -f          # 实时日志
kill -HUP $(pidof modbus-backend)        # 热重载配置
```

`modbus-backend.service` 关键点：`Type=simple`（前台程序）、`Restart=always` +
`RestartSec=5`（崩溃 5 秒后自动拉起，无人值守）、`WorkingDirectory=/var/lib/modbus-backend`
（相对路径的 db / 日志都落在这里）、`NoNewPrivileges=yes` + `PrivateTmp=yes` 安全加固。

`modbus-backend.logrotate` 用 `copytruncate`：**先复制再截断**，进程手里的
文件描述符依然有效，无需通知程序重开文件。注意被管理的日志路径必须与
systemd 的 `WorkingDirectory` 拼出来的实际路径一致，否则 logrotate 会
"cannot stat" 什么都不做（最常见踩坑点）。

## 固件打包与升级流程

```bash
# 1. Keil 编译出 modbus-gateway.bin（半字对齐、字节序都是小端）
# 2. 打包成 .ota（加 16 字节头：magic/version/size/crc32）
python3 tools/pack_firmware.py <input.bin> firmware/modbus-gateway_v1.2.ota --version 0x0102
#    脚本内置回读自检：struct.unpack 逐字段 assert，输出 Self-verification: PASS
# 3. 浏览器打开 http://<host>:8080/ → 选择网关 → 上传 .ota → 观察进度
#    或直接 curl：
curl -F "gateway=1" -F "file=@firmware/modbus-gateway_v1.2.ota" \
     http://localhost:8080/api/ota/upload
curl http://localhost:8080/api/ota/status
```

`pack_firmware.py` 的关键点：

- 读 `.bin` **必须 `'rb'`**（Windows 下漏 `b` 会坏数据）。
- **半字对齐补 `0xFF`**：STM32 Flash 编程单位是 16 位，奇数长度要补齐；
  补 `0xFF` 而非 `0x00` 是因为 ① 擦除后的空白值就是 `0xFF`；
  ② 万一被当代码执行，`0xFFFF` 是 ARM 非法指令会立刻 HardFault，
  而 `0x0000` 是合法指令会导致静默跑飞。
- **CRC32 算的是补位后的数据**（下位机收到多少就校验多少，按补位前算必然对不上）。
- `struct.pack('>4sIII', b'PJ1B', version, len(fw_data), fw_crc32)` 用大端
  与下位机结构体一致。

## 离线联调工具（附带的测试辅助）

> 说明：本项目的功能与性能数据均在**真实硬件**上测得（STM32F103 网关 + KTH2-R30
> 温湿度变送器 + W5500），见顶层 README 的「实测数据」一节。下面这个脚本只是
> 用来在**手头没有硬件**时跑自动化回归测试的辅助工具（`tools/test_all.sh`
> 依赖它构造多网关场景），不参与实际部署。

`tools/fake_gateway.py` 是一个纯 Python 实现的 Modbus-TCP 从站（无第三方依赖），
用于构造多个虚拟节点、验证上位机的并发采集与断线重连逻辑：

```bash
python3 tools/fake_gateway.py 5021 26.3 55.0    # 端口 / 温度℃ / 湿度%
```

- 支持功能码 `0x03`（读保持寄存器）/ `0x04`（读输入寄存器）/ `0x06`（写单寄存器），
  固定寄存器 `0x0300` = 温度、`0x0301` = 湿度。
- 温度用 `int(round(TEMP_C * 10))` 而非 `int()` —— 避免 `26.3 * 10 = 262.9999…`
  被截断成 262。
- `SO_REUSEADDR` + `settimeout(1.0)`（让阻塞 `accept` 每秒醒一次以响应退出信号）
  + `signal_handler` 优雅退出，保证端口不被 `TIME_WAIT` 占住。
- 单线程模型（一次只服务一个连接），调试够用。
- 启动时会打印等价的 `mbpoll` 测试命令，可用标准 Modbus 工具交叉验证。

配套的 `tools/test_all.sh` 是一条龙回归脚本，用"测试从站 + 真进程 + 真 HTTP 请求"
端到端跑通整条链路，7 个用例：

```bash
cd linux-host && bash tools/test_all.sh
```

1. HTTP API 返回 ≥2 个网关数据
2. 日志中有成功采集记录（数据真的在流动）
3. 断线重连（停掉测试从站 → 检测断开 → 重启 → 检测重连）
4. 故障隔离（GW1 断线时 GW2 照常采集 ≥2 次）
5. 热加载新增网关（追加 `[gateway.3]` + SIGHUP → 返回 3 个）
6. 热加载删除网关（删段 + SIGHUP → 返回 2 个且都有名字）
7. 热加载改参数（改 `interval_sec` + SIGHUP → 采集频率变化）

脚本用 `trap cleanup EXIT` 保证无论成功失败还是 Ctrl+C，都会清理测试从站与后端、
还原 `config.ini` 备份。

## 已知限制

- HTTP 服务为自用级实现：HTTP/1.0 + `Connection: close`；**未做 `../` 目录穿越
  校验**；accept 后同步处理，慢客户端会阻塞其他新连接；无鉴权、无 HTTPS。
- OTA 目标 IP / 端口硬编码（`ota.c` 标 TODO），未从配置读取。
- `HttpServer_Stop` 后 `thread_args` 未 `free`（属可接受的一次性泄漏）。
- Poller 的 fd → 网关映射是 O(N) 线性查找（上限 16 可接受）。
- 同一时刻只支持一个 OTA 任务（全局单例）。
