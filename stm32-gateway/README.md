# stm32-gateway — Modbus 协议转换网关固件

STM32F103C8T6 + FreeRTOS + W5500 + W25Q64 的工业协议转换网关。下位通过 RS-485
采集 Modbus-RTU 从机（KTH2-R30 温湿度变送器），上位用 W5500 提供 4 路 Modbus-TCP 服务端
（502~505）加 1 路 OTA 升级通道（506）。

```
                   ┌──────────────── STM32F103C8T6 (72MHz / 64KB Flash / 20KB RAM) ────────────────┐
                   │                                                                              │
 RS-485            │   ┌──────────┐   ┌───────────────┐   ┌──────────────┐   ┌───────────────┐   │
 9600 8N1  ◄───────┼──►│ USART2   │──►│ modbus_rtu.c  │──►│ modbus_tcp.c │──►│  app_modbus.c │   │
 KTH2-R30 从机    │   │ +DMA+IDLE│   │ RTU 主站收发  │   │ MBAP↔RTU 转换│   │ TcpServerTask │   │
 DE/RE=PA1         │   └──────────┘   └───────────────┘   └──────────────┘   │  Socket 0..3  │   │
                   │                                                          └───────┬───────┘   │
 Ethernet   ◄──────┼───────────────────────── SPI2 (PB13/14/15, CS=PB12) ─────────────┘           │
 (Modbus-TCP)      │                          bsp_w5500.c                                          │
                   │                                                                              │
 Flash      ◄──────┼── SPI1 (PA5/6/7, CS=PA4) ── bsp_w25q64.c  配置 / 2048 条环形日志 / OTA 暂存 │
 RTC        ◄──────┼── I2C1 (PB6/PB7) ──────── bsp_ds3231.c    日志时间戳                        │
 UART 调试  ◄──────┼── USART1 (PA9/PA10) ───── bsp_printf_dma.c 非阻塞 printf                     │
                   └──────────────────────────────────────────────────────────────────────────────┘
```

## 目录结构

| 目录 | 内容 |
|---|---|
| `App/` | 应用任务层：TCP 服务、健康巡检、看门狗/日志任务、OTA 接收 |
| `BSP/` | 板级驱动：W5500 / W25Q64 / DS3231 / RS-485 / IWDG / DMA printf |
| `Protocol/` | Modbus 协议：RTU 主站、TCP↔RTU 转换、寄存器映射 |
| `Utils/` | 通用工具（CRC 查表） |
| `Bootloader/` | 独立 Bootloader 工程（双分区升级 + 回滚） |
| `Core/` | CubeMX 生成的外设初始化与 FreeRTOS 启动代码 |
| `Drivers/` / `Middlewares/` | ST 官方 HAL 库 / FreeRTOS 内核 |
| `NET/context.md` | 引脚分配、端口配置、W5500 寻址公式与 5 条历史调试记录 |
| `MDK-ARM/` | Keil MDK 工程（`modbus-gateway.uvprojx`） |

## 目录结构（文件级）

### App/ — 应用任务层

| 文件 | 行数 | 作用 |
|---|---|---|
| `app_modbus.c/.h` | 433 | `TcpServerTask`：4 个 TCP Socket 两阶段状态机 + Socket4 OTA 轮询 |
| `app_sensor.c/.h` | 96 | `HealthCheck` 任务：每 60s 巡检 RS-485 总线，连续 3 次失败判故障 |
| `app_task.c/.h` | 212 | `Wdg_Supervisor_Task` 软件看门狗 + `Log_Task` 异步日志 |
| `ota_receive.c/.h` | 704 | App 侧 OTA 固件接收：环形缓冲 + 粘包解帧 + 写 W25Q64 暂存区 |

### BSP/ — 板级驱动

| 文件 | 行数 | 作用 |
|---|---|---|
| `bsp_w5500.c/.h` | 509 | W5500 驱动：SPI 帧组装、Socket 收发、BSB 区块寻址 |
| `bsp_w25q64.c/.h` | 1060 | W25Q64 驱动：配置持久化、2048 条环形日志、OTA 暂存/备份 |
| `bsp_rs485.c/.h` | 306 | RS-485 收发：DE/RE 方向切换 + DMA 循环 + USART IDLE 帧捕获 |
| `bsp_ds3231.c/.h` | 448 | DS3231 RTC：BCD 编解码、Unix 时间戳转换 |
| `bsp_iwdg.c/.h` | 141 | 独立看门狗（LSI 40kHz / 预分频 64 / RLR 4000 → 6.4s） |
| `bsp_printf_dma.c/.h` | 188 | printf 重定向 USART1 + DMA 环形缓冲（非阻塞输出） |

### Protocol/ — Modbus 协议

| 文件 | 行数 | 作用 |
|---|---|---|
| `modbus_rtu.c/.h` | 439 | RTU 主站：CRC16 查表、请求组帧、响应校验 |
| `modbus_tcp.c/.h` | 208 | TCP↔RTU 转换：MBAP 头解析与封装、异常帧自造 |
| `modbus_map.h` | 42 | 寄存器映射（温度 `0x0300` / 湿度 `0x0301`） |

### Core/ — 启动与配置

| 文件 | 行数 | 作用 |
|---|---|---|
| `main.c` | 271 | 入口：VTOR 重定位、IWDG 立即喂狗、时钟与外设初始化、启动调度器 |
| `freertos.c` | 305 | `StartDefaultTask` 五阶段初始化 + 全局配置 `g_gateway_config` |
| `FreeRTOSConfig.h` | 154 | 内核裁剪：1kHz tick、56 优先级、9216B 堆、静态+动态混合分配 |

## 各模块实现细节

### 应用任务

| 文件 | 任务名 | 优先级 | 栈（字 / 字节） | 周期 |
|---|---|---|---|---|
| `app_modbus.c` | `TcpServer` | `osPriorityNormal` | 1024 / 4 KB | 10ms 轮询 |
| `app_sensor.c` | `HealthCheck` | `osPriorityLow` | 256 / 1 KB | 60 s |
| `app_task.c` | `Wdg_Supervisor` | `osPriorityLow1` | 192 / 768 B | 2 s（`osDelayUntil` 定周期） |
| `app_task.c` | `Log` | `osPriorityLow` | 256 / 1 KB | 队列驱动（1 s 超时） |
| `freertos.c` | `defaultTask` | `osPriorityNormal` | 128 / 512 B | 1 s 心跳 |

**`app_modbus.c` — TcpServerTask**

- 4 个 TCP Socket 复用同一组 2KB 收发缓冲，用两阶段状态机
  `SOCK_PHASE_IDLE → SOCK_PHASE_TX_WAIT` 代替"一发一收"阻塞写法，
  避免等待响应的 Socket 卡住其余 3 个。
- 三层并发保护：
  1. `g_buf_mutex` —— 缓冲互斥锁，整个事务期间持有；
  2. `g_rtu_busy` —— 快速门闩，保证同一时刻只有一个 Socket 在跑 RTU 事务；
  3. `g_modbus_mutex` —— RS-485 物理总线锁，防止与 `HealthCheck` 巡检任务撞车。
- 常量：`TCP_IDLE_TIMEOUT_MS = 30000`（空闲断连）、`TCP_SEND_FAIL_MAX = 3`
  （连续发送失败 3 次判连接死亡）。
- 初始化 7 步：W5500 初始化 → 构造上下文 → Socket0~3 各分配 2KB 收发缓冲 →
  Socket4 保留剩余缓冲 → 监听 502~505 → 6a 启动状态机 / 6b 启动 OTA /
  6c 启动 5s 统计打印。
- 栈 4KB 的原因：OTA 热路径调用链（`W5500 收发 → 解帧 → W25Q64 写页`）实测
  峰值约 2.6KB，留一倍余量。

**`app_task.c` — 双层看门狗**

- `WDG_TASK_COUNT = 4`，被监控的任务 ID：`WDG_TASK_ID_DEFAULT / TCP / SENSOR / LOG`。
- `WDG_ALIVE_TIMEOUT_MS = 15000`（任务判死门限）、`WDG_CHECK_INTERVAL_MS = 2000`
  （检查周期）。
- 关键设计：必须**连续两次全员存活**（`consecutive_alive >= 2`）才调用
  `BSP_IWDG_Reload()`，单次"假活"不喂狗。
- `Wdg_Heartbeat(id)` 无锁实现 —— 每个任务只写自己下标对应的 `uint32_t`，
  Cortex-M3 上 32 位对齐写入是原子的，无需临界区。
- `Log_Task` 从 `osMessageQueueGet(queue, &entry, 1000)` 超时取日志，
  取到就写 W25Q64 环形区。
- `Log_Push()` 非阻塞投递（超时 0，队列满直接丢）：日志是"可丢的辅助信息"，
  绝不能因为队列满把业务任务阻塞住。
- 队列容量 64 条，元素为 `W25Q64_LogEntry`（8 字节：`timestamp / type / reserved / data`）。

**`app_sensor.c` — HealthCheck 任务**

- `HEALTH_CHECK_INTERVAL_SEC = 60`、`HEALTH_FAIL_THRESHOLD = 3`。
- 每轮读一次传感器，连续 3 次失败才判定总线故障（避免偶发干扰误报）。
- 拆成 60 个 1s 小循环喂狗，保证一个 60s 长周期内软件看门狗也不会判死。

### BSP 驱动

**`bsp_w5500.c/.h`**

- SPI 帧格式：`[ADDR_H][ADDR_L][BSB<<3 | OM][DATA...]`，`OM` 的 bit2 = 1 表示写。
- **BSB 区块寻址**（本项目修正过一次的坑）：

  | 用途 | 公式 | 示例 n=1 |
  |---|---|---|
  | Socket 寄存器区 | `1 + 4n` | 5 |
  | TX 缓冲 | `2 + 4n` | 6 |
  | RX 缓冲 | `3 + 4n` | 7 |

  网上流传的 `2 + n*3` 是 W5100 时代的错误公式，照抄会导致连上 TCP 但数据全 0。

- `VERSIONR = 0x0039`，读回应为 `0x04`，是判断 SPI 通不通的首选探针。
- `PHYCFGR = 0x002E` 是 **8 位**寄存器，必须用单字节读，用 16 位读会读到
  相邻寄存器内容（排查表里专门记了这条）。
- 默认 `W5500_TCP_PORT = 502`，Socket 收发缓冲各 2048 字节。

**`bsp_w25q64.c/.h`**

- NOR Flash 四铁律：① 位只能 1→0；② 最小擦除单位是 4KB 扇区；
  ③ 写前必须先发 `0x06` 写使能；④ 擦/写后必须轮询 `SR1.BUSY` 直到清零。
- `g_w25q64_mutex` 串行化所有 Flash 访问；内部 `xxxNoLock()` 是无锁版本
  （普通互斥锁不可重入，加锁函数内部调用加锁函数会死锁）。
- CRC32 查表（多项式 `0xEDB88320`，初值 `0xFFFFFFFF`，末尾取反）。
- 分区布局：

  | 偏移 | 用途 |
  |---|---|
  | `0x000000` | 配置区（`W25Q64_Config`，CRC32 校验） |
  | `0x001000` | 日志元数据（`[2:3]count` / `[4:5]index`，大端） |
  | `0x002000` | 日志数据区（最多 `W25Q64_LOG_ENTRY_MAX = 2048` 条，每 4KB 扇区 512 条） |
  | `0x007000` | OTA 控制块（19 字节） |
  | `0x010000` | OTA 暂存区（48 KB） |
  | `0x01C000` | 回滚备份区（48 KB） |

- 上电"校准"日志条数：从写指针向前回扫，直到遇到第一条非法记录，避免掉电
  导致的元数据与实际数据不一致。
- `BSP_W25Q64_Write` 自动按 256 字节页拆分跨页写入。
- `SendEraseCmd` / `IsBusy` 组成非阻塞擦除对，供 OTA 在有限状态机里逐步推进。
- `ReadStatus2` 查 `P_FAIL`（bit5）辅助判断擦写是否物理失败。
- `LogClear` 只清元数据（重置 count/index），不擦 7.5MB 数据区 —— 秒级完成。

**`bsp_rs485.c/.h`**

- 收方向：DMA1_Channel6 循环模式 + USART2 IDLE 中断捕获整帧。帧长用
  `RS485_FRAME_BUF_SIZE - 剩余 DMA counter` 反推；`len > 0` 过滤掉
  "刚发完请求、从机还没答"的第一次空 IDLE。
- `StartRxCapture` 关键三步：Abort DMA → 读 SR+DR 清 IDLE/RXNE/ORE 标志 →
  置 `huart2.RxState = HAL_UART_STATE_READY`（HAL 库状态机没复位会导致下次收失败）。
- `BSP_RS485_Send` 十步：拿锁 → 停 DMA → 清标志 → 切 TX 模式 → 关 RE 与 RXNEIE
  （**杜绝自发自收回显**）→ `HAL_UART_Transmit(..., 200ms)` → **死等 TC 标志**
  （必须是 TC 不是 TXE，TXE 只代表数据进了移位寄存器）→ 切回 RX 模式 →
  空转 2000 次等收发器方向切换稳定 → 重新武装 DMA → 放锁。
- `HAL_UART_ErrorCallback` 只统计不恢复（自愈式）：错误计数
  `g_uart_error_count` / 最近错误码 `g_uart_last_error`，靠下一次完整帧自然恢复。

**`bsp_iwdg.c/.h`**

- LSI ≈ 40 kHz，预分频 64 → 每格 1.6 ms；`RLR = 4000` → 超时 6.4 s。
- 三个密钥：`0xCCCC` 使能（**一旦写下去软件无法关闭**）、`0x5555` 解锁 PR/RLR、
  `0xAAAA` 喂狗。
- `BSP_IWDG_Init` 五步：开 LSI 并轮询 `LSIRDY`（100000 次超时）→
  `__HAL_DBGMCU_FREEZE_IWDG`（调试断点时冻结，否则单步调试会被狗咬）→
  等 `PVU` 后写 PR → 等 `RVU` 后写 RLR → **先喂一次再使能**。
- `BSP_IWDG_IsResetByIWDG()` 查 `RCC_CSR.IWDGRSTF`；
  `BSP_IWDG_ClearResetFlag()` 清粘滞标志（不清的话下次启动永远看到"上次是看门狗复位"）。

**`bsp_ds3231.c/.h`**

- I2C 地址 `0xD0`；`DS3231_DateTime` 是解码后的 `year/month/date/weekday/hour/minute/second`。
- 提供 `BSP_DS3231_Init / GetTime / SetTime / GetUnixTimestamp / SetUnixTimestamp`。

**`bsp_printf_dma.c/.h`**

- printf 重定向到 `BSP_PrintfDma_PutChar`，走 USART1 + DMA1_Ch4 环形缓冲，
  不阻塞调用者；调试口 115200 8N1。

### Modbus 协议层

**`modbus_tcp.c`**

- MBAP 7 字节：`TransID(2) / ProtoID(2)=0 / Length(2)=1+PDU长度 / UnitID(1)`，全大端。
- `TCPtoRTU`：校验 PDU 长度 1~253，转成 RTU 帧并补 CRC16（低字节在前）。
- `RTUtoTCP`：PDU 长度 = `rtu_len - 3`，**TransID 原样回传**（客户端靠它配对请求）。
- 通信失败（从机不响应）时，网关自造 Modbus 异常帧：
  `[MBAP 7B][fc|0x80][0x0B]`，共 9 字节；`0x0B = 网关目标设备无响应`，
  让上位机明确知道"是网关后面的从机没答"，而不是网络问题。

**`modbus_rtu.c`**

- CRC16 查表实现，请求组帧 + 响应长度与 CRC 校验。

### Core/ — 启动流程

**`main.c`（11 步）**

1. `SCB->VTOR = 0x08004000`（App 起始地址，Bootloader 跳转后必须重定位向量表）
2. `IWDG->KR = 0xAAAA` —— **第一时间喂狗**（Bootloader 已使能 IWDG，超时很短，
   初始化耗时超过超时会立刻复位）
3. `HAL_Init()`
4. `SystemClock_Config()`：HSE 8MHz × PLL9 = 72MHz，APB1 = 36MHz / APB2 = 72MHz，
   `FLASH_LATENCY_2`
5. `MX_GPIO_Init / MX_I2C1_Init / MX_SPI1_Init / MX_USART2_Init / MX_SPI2_Init`
6. `MX_USART1_UART_Init` + `BSP_PrintfDma_Init()`
7. `osKernelInitialize()`
8. `MX_FREERTOS_Init()`
9. `osKernelStart()`

`Error_Handler()` 关中断死循环（等待看门狗复位）。

**`freertos.c` — `StartDefaultTask` 五阶段**

| 阶段 | 动作 |
|---|---|
| ① | `BSP_IWDG_IsResetByIWDG()` 判断上次复位原因，是看门狗复位则打印告警并 `ClearResetFlag()` |
| ② | `BSP_W25Q64_Init()` → `ConfigLoad(&g_gateway_config)` → 打印 IP / 网关 / 端口 |
| ③ | `BSP_DS3231_Init()`（年份 < 2024 用编译时间回写）→ `Log_TaskInit()` → 打印 Flash 中最近 10 条日志 → `Log_Push(W25Q64_LOG_SYSTEM_BOOT, 0)` |
| ④ | `Modbus_RTU_Init()` → `OTA_Init()` → `APP_Sensor_Init()` → `APP_Modbus_Init()` |
| ⑤ | `Wdg_TaskInit()` → `BSP_IWDG_Init()`（**顺序不能反**：先把监控者立起来，再开狗） |

之后 `for(;;) { Wdg_Heartbeat(WDG_TASK_ID_DEFAULT); osDelay(1000); }`。

日志类型字符串：`BOOT / TCP_ON / TCP_OFF / 485_ERR / 485_OK / CFG / SENSOR`。

**`FreeRTOSConfig.h`**（V10.3.1）

| 配置项 | 值 |
|---|---|
| `configTICK_RATE_HZ` | 1000（1ms 节拍） |
| `configMAX_PRIORITIES` | 56 |
| `configMINIMAL_STACK_SIZE` | 128 字 |
| `configTOTAL_HEAP_SIZE` | 9216 字节（`heap_4`） |
| `configMAX_TASK_NAME_LEN` | 16 |
| `configUSE_TRACE_FACILITY` | 1 |
| `configUSE_MUTEXES / RECURSIVE_MUTEXES / COUNTING_SEMAPHORES` | 1 |
| `configUSE_TIMERS` | 1（Timer 任务优先级 2、队列长度 10、栈 256） |
| `configLIBRARY_LOWEST_INTERRUPT_PRIORITY` | 15 |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 |
| `USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION` | 1 |

## OTA 升级（App 侧接收）

固件经 TCP 506 下发到 W5500 Socket4，`ota_receive.c` 负责接收、校验、写暂存区，
校验通过后写控制块并软复位，由 Bootloader 完成真正的烧写。

- 常量：`OTA_SOCKET = 4`、`OTA_RX_BUF_SIZE = 2048`（环形缓冲）、`OTA_TIMEOUT_MS = 5000`。
- 环形缓冲用 `head/tail`（`volatile`）配 `& (SIZE-1)` 位与代替取模（编译期常量，
  编译器会自动优化成单条 AND 指令）。
- 粘包解帧：命令帧 `OTA_CMD_MAGIC`（16B）、数据帧 `OTA_DATA_MAGIC`（12 + dlen），
  magic 不匹配时**滑窗前进 1 字节重同步**（而不是丢弃整个缓冲）。
- `OTA_OnData` 用 `__get_PRIMASK / __disable_irq / __set_PRIMASK` 临界区保护
  写入指针，保证与 `OTA_Tick` 的读取不打架。
- **START 时同步全量擦除暂存区**：实测发现"擦除窗口内页编程会被静默丢弃"，
  所以改成开始时一次性擦完，代价是 START 响应变慢（主机侧超时须 ≥ 5s）。
- **FINISH 时 256B 流式重算 CRC32** 比对：初值 `0xFFFFFFFF`、末尾 `^ 0xFFFFFFFF`，
  每 16 块喂一次狗（48KB 全部读回已超过一次喂狗间隔）。
- 成功则构造 `OTACtrlBlock`：`magic = 0xA5A5A5A5`、`fail_flag` 全 `0xFF`、
  `ctrl_crc16 = CRC16_Modbus(ctrl, CTRL_CRC_LEN=12)`；写控制块 → 等 W5500
  `TxFree ≥ 2048` → W5500 软复位（`W5500_MR_RST`）→ `NVIC_SystemReset()`。
- `OTA_HandleData` 10 步顺序：`erase_fatal` → 阶段检查 → 越界检查 → 取消检查 →
  扇区是否已擦 → 异步擦除门闸 → CRC16 校验 → 三态包序号检查 →
  写入 + 回读验证（重试 3 次）→ 回 ACK。
- `OTA_Tick` 推进异步擦除进度 + 5s 超时重建 Socket。
- `OTA_IsInProgress()` 供日志任务判断"升级期间暂停写 Flash"，避免 SPI 事务互相干扰。

## 引脚分配

```
SPI2 → W5500 (18MHz, Mode 0)        SPI1 → W25Q64 (18MHz, Mode 0)
  SCK  = PB13                        SCK  = PA5
  MISO = PB14                        MISO = PA6
  MOSI = PB15                        MOSI = PA7
  CS   = PB12 (软件片选)              CS   = PA4 (软件片选)
  RST  = PB1
  INT  = PB0 (查询方式，未用中断)

USART2 → RS-485 (半双工 9600 8N1)    USART1 → 调试串口 (115200 8N1)
  TX   = PA2                          TX = PA9
  RX   = PA3                          RX = PA10
  DE/RE= PA1 (高=发, 低=收)

I2C1 → DS3231 RTC (100kHz)
  SCL  = PB6
  SDA  = PB7
```

RS-485 在物理层还需要：SN65HVD230 / MAX485 收发器 + 总线两端 120Ω 终端电阻。

## 网络与端口

| 项 | 值 | 说明 |
|---|---|---|
| 默认 IP | 192.168.10.100 | 存 W25Q64 配置区，CRC 校验失败回退默认值 |
| 子网掩码 / 网关 | 255.255.255.0 / 192.168.10.1 | |
| Modbus-TCP | 502 ~ 505 | Socket0 ~ Socket3 |
| OTA | 506 | Socket4 |
| W5500 缓冲分配 | Socket0~3 各 2KB TX+RX；Socket4 预留其余 | 合计 20KB ≤ 32KB |

> Bootloader 故障态使用**另一套**网络参数（`Bootloader/User/app_config.h`：
> MAC = `00:08:DC:12:34:56`，IP = `192.168.1.100`），仅当 App 变砖、Bootloader
> 进入故障恢复模式时生效，与 App 正常运行的网段不同，联调时注意区分。

## 编译

用 Keil MDK5 打开 `MDK-ARM/modbus-gateway.uvprojx`：

- 分组：`Start` / `Library`(HAL) / `App` / `User` / `Bootloader`
- 产物：`modbus-gateway.axf` / `.hex`
- 头文件路径与源文件路径均已核对有效

Bootloader 单独打开 `Bootloader/Bootloader.uvprojx` 编译，烧写到 `0x08000000`
（链接上限 16KB，ARMCC V5 `-O1`）。

生成 `.ota` 升级包：

```bash
cd ../linux-host
python3 tools/pack_firmware.py ../stm32-gateway/MDK-ARM/modbus-gateway/modbus-gateway.bin \
                             firmware/modbus-gateway_v1.2.ota --version 0x0102
```

## 上电行为

1. `main()` 先设 `SCB->VTOR` 并立即喂狗（Bootloader 已使能 IWDG）。
2. 时钟与外设初始化 → 启动 FreeRTOS。
3. `defaultTask` 检查复位原因（若上次为看门狗复位则打印告警）并清除标志。
4. W25Q64 初始化 → 加载配置（CRC 校验，失败用默认值）。
5. DS3231 初始化 → 启动异步日志任务。
6. 打印 Flash 中最近 10 条历史日志，并记录本次启动事件。
7. 依次初始化：Modbus-RTU 互斥锁 → OTA 模块 → HealthCheck 任务 →
   TcpServerTask（4 Socket 监听 + OTA Socket4）。
8. 先启动软件看门狗任务，再使能硬件 IWDG（顺序不可颠倒）。
9. `defaultTask` 进入 1s 心跳循环。

## 排查表

| 现象 | 可能原因 | 检查点 |
|---|---|---|
| W5500 初始化失败，`VERSIONR ≠ 0x04` | SPI 通信异常 | CS 引脚定义、SPI 模式/时钟、是否被 NSS 复用占用 |
| TCP 连上但数据全 0 | TX/RX 区块寻址错 | `TX_BLOCK(n) = 2+4n`、`RX_BLOCK(n) = 3+4n` |
| `PHYCFGR` 读数异常 | 用了 16 位读函数 | PHYCFGR 是 8 位寄存器，须用单字节读 |
| RS-485 读写偶发失败 | 总线互斥 / 方向切换时序 | DE 引脚时序、任务锁超时设置、TC 是否死等 |
| 偶发 Flash 写入失败 | TcpServerTask 栈溢出 | 栈设为 1024 字 = 4KB |
| 系统频繁复位 | 喂狗时机 | 软件看门狗任务须覆盖全部 4 个任务心跳、需连续 2 次存活 |
| 从机不响应但网络正常 | 网关已回 0x0B 异常帧 | 查从机地址 / 波特率 / 接线 |

更多调试细节见 [`NET/context.md`](NET/context.md)。
