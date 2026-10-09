# STM32 Modbus 协议转换网关 — 调试上下文文档

> 本文档已按**当前代码**核对修正（GPIO/SPI/USART/I2C 初始化见 `Core/Src/*.c`）。
> 早期调试阶段的接线与现象（W5500 曾在 SPI1/CS=PA4、W25Q64 曾在 SPI2/CS=PB12、IP 192.168.1.100）
> 已整理到文末「七、历史调试记录」，**不代表当前接线**。

## 一、项目概述

STM32F103C8T6 + FreeRTOS + W5500 + W25Q64 工业级 Modbus 协议转换网关。

- **下位（采集）**：RS485（USART2，PA2=TX，PA3=RX，PA1=DE/RE）外接 Modbus-RTU 温湿度变送器（KTH2-R30，地址 1，9600 8N1，寄存器 0x0300 温度 / 0x0301 湿度）。
- **上位（上传）**：W5500（SPI2，CS=PB12，RST=PB1）提供 Modbus-TCP 服务端（502~505），Socket4 提供 OTA 服务（506）。
- **存储**：W25Q64（SPI1，CS=PA4）存配置参数、事件日志、OTA 暂存区/备份区/控制块。
- **RTC**：DS3231（I2C1，SCL=PB6，SDA=PB7）为日志提供时间戳。

## 二、硬件引脚分配（当前代码，已验证）

```
SPI2 → W5500 以太网（APB1 36MHz, 分频 2 → 18MHz, Mode 0）
  W5500_SCLK = PB13  (SPI2_SCK,  复用推挽)
  W5500_MISO = PB14  (SPI2_MISO, 浮空输入)
  W5500_MOSI = PB15  (SPI2_MOSI, 复用推挽)
  W5500_CS   = PB12  (GPIO 输出, 软件片选, 空闲高)      ← bsp_w5500.h
  W5500_RST  = PB1   (GPIO 输出, 空闲高)                 ← bsp_w5500.h
  W5500_INT  = PB0   (输入, 代码为轮询未用中断)          ← gpio.c

SPI1 → W25Q64 Flash（APB2 72MHz, 分频 4 → 18MHz, Mode 0）
  W25Q64_SCLK = PA5   (SPI1_SCK,  复用推挽)
  W25Q64_MISO = PA6   (SPI1_MISO, 浮空输入)
  W25Q64_MOSI = PA7   (SPI1_MOSI, 复用推挽)
  W25Q64_CS   = PA4   (GPIO 输出, 软件片选, 空闲高)      ← gpio.c

USART2 → RS485（半双工, 9600 8N1）
  RS485_DE/RE = PA1  (GPIO 输出, 高=发送, 低=接收)       ← bsp_rs485.h
  RS485_TX    = PA2  (USART2_TX, 复用推挽)
  RS485_RX    = PA3  (USART2_RX, 浮空输入)

USART1 → 调试串口（printf, 115200 8N1）
  TX = PA9, RX = PA10                                     ← usart.c

I2C1 → DS3231 RTC（100kHz）
  SCL = PB6, SDA = PB7                                    ← i2c.c
```

## 三、网络与端口配置

```
W5500 MAC       = 02:00:00:00:00:01      ← bsp_w5500.c: BSP_W5500_Init 内硬编码
默认 IP        = 192.168.10.100          ← 存 W25Q64 配置区, CRC 校验失败回退默认
子网掩码        = 255.255.255.0
默认网关        = 192.168.10.1
TCP 起始端口    = 502                      ← Socket0=502, Socket1=503, Socket2=504, Socket3=505
OTA 端口        = 506                      ← Socket4 (app_modbus.c / ota_protocol_def.h)

W5500 32KB 缓冲分配 (app_modbus.c):
  Socket0~3 (Modbus TCP) : TX 2KB + RX 2KB = 16KB
  Socket4  (OTA 506)     : TX 2KB + RX 2KB = 4KB
  Socket5~7              : 0 (不分配)
  合计 20KB ≤ 32KB, 未超限
```

> ⚠ Bootloader 故障态（Failsafe）用的是**另一套**网络参数（`Bootloader/User/app_config.h`：
> MAC=00:08:DC:12:34:56，IP=192.168.1.100，仅当 App 变砖进入故障态时生效），与 App 正常运行的
> 192.168.10.100 不同网段，联调时注意区分。

## 四、W5500 BSB 区块寻址（正确公式）

```c
// bsp_w5500.h — 与 WIZnet 官方 ioLibrary_Driver/Ethernet/W5500/w5500.h 一致
#define W5500_SOCK_REG(n)   (1 + (n) * 4)   // Socket n 寄存器区
#define W5500_TX_BLOCK(n)   (2 + (n) * 4)   // Socket n TX 缓冲区
#define W5500_RX_BLOCK(n)   (3 + (n) * 4)   // Socket n RX 缓冲区
```

每个 Socket 占 3 个区块，每 4 块一组（Block 4/8/12/… 为保留区）：

| Socket | 寄存器区 | TX 缓冲区 | RX 缓冲区 |
|--------|---------|-----------|-----------|
| 0 | Block 1  | Block 2   | Block 3   |
| 1 | Block 5  | Block 6   | Block 7   |
| 2 | Block 9  | Block 10  | Block 11  |
| 3 | Block 13 | Block 14  | Block 15  |
| 4 | Block 17 | Block 18  | Block 19  |
| … | 1+4n     | 2+4n      | 3+4n      |

> ⚠ 网上流传的 `2+n*3` / `3+n*3` 是**错误公式**（来自 W5100 时代每 Socket 只占 3 块的错误记忆）。
> 正确是 `2+4n` / `3+4n`。本项目早期踩过"TX/RX 区块写错导致读到错误区域、数据全 0"的坑（见文末历史记录）。

## 五、SPI 帧格式（读/写寄存器）

```
CS 拉低 → [地址高字节][地址低字节][控制字节][数据字节…] → CS 拉高

控制字节 = (BSB << 3) | (RWB << 2) | OM
  BSB[7:3] = 区块号（见上表）
  RWB[2]   = 1 写 / 0 读
  OM[1:0]  = 00 连续模式

读寄存器（4 字节）:  MOSI 发 [AH][AL][CB][0x00]，MISO 第 4 字节返回数据
写寄存器（4 字节）:  MOSI 发 [AH][AL][CB][DATA]
```

**调试要点**：VERSIONR（0x0039, Common Block）必须读回 **0x04** 才算 SPI 通信正常；
PHYCFGR（0x002E）是 **8 位**寄存器，要用 `BSP_W5500_ReadReg()`（单字节读），不能用 `ReadReg16()`。

## 六、关键文件索引

| 文件 | 作用 |
|------|------|
| `BSP/bsp_w5500.c/.h` | W5500 底层驱动（SPI 读写、Socket 管理、寄存器/区块宏定义） |
| `BSP/bsp_w25q64.c/.h` | W25Q64 驱动（配置持久化 + 环形日志 + OTA 暂存/备份） |
| `BSP/bsp_rs485.c/.h` | RS485 收发（DE/RE 方向切换 + DMA/IDLE 帧捕获） |
| `BSP/bsp_ds3231.c/.h` | DS3231 RTC（日志时间戳） |
| `BSP/bsp_iwdg.c/.h` | 独立看门狗（预分频 64 + 重装载 4000 → ≈6.4s） |
| `BSP/bsp_printf_dma.c/.h` | printf 重定向到 USART1 + DMA（非阻塞输出） |
| `Protocol/modbus_rtu.c` | Modbus RTU 主站（CRC16 查表 + 阻塞/非阻塞收发） |
| `Protocol/modbus_tcp.c` | Modbus TCP↔RTU 协议转换（MBAP 解析） |
| `Protocol/modbus_map.h` | 寄存器映射（温度 0x0300 / 湿度 0x0301） |
| `App/app_modbus.c` | TCP 服务任务（4 Socket 状态机 + OTA 收包） |
| `App/app_sensor.c` | 健康巡检任务（每 60s 读传感器） |
| `App/app_task.c` | 看门狗监控任务 + 异步日志任务 |
| `App/ota_receive.c` | OTA 固件接收（环形缓冲 + 帧解析 + 写暂存区） |
| `Core/Src/freertos.c` | FreeRTOS 初始化 + defaultTask |
| `Core/Inc/FreeRTOSConfig.h` | 堆 9216、优先级 56、tick 1000、heap_4 |
| `Bootloader/User/main.c` | Bootloader 主流程（读控制块 → 升级/回滚 → 跳 App） |
| `Bootloader/User/boot_jump.c` | 跳转 + 故障态（Failsafe） |
| `Bootloader/User/rollback.c` | 备份/回滚实现 |
| `OtaCommon/*.h` | OTA 控制块/协议/CRC（Bootloader 与 App 共用） |

## 七、历史调试记录（已修复）

> 以下都是**早期调试阶段**踩过的坑，现在代码已修复，保留作为排查参考。

### 1. W5500 SPI 通信失败（VERSIONR 读回 0x00）
- **现象**：W5500 初始化失败，VERSIONR=0x00（正常 0x04），Modbus Poll 连不上。
- **根因（三处叠加）**：① CS 引脚定义与硬件不一致；② PHYCFGR 是 8 位寄存器却用 16 位读函数；③ SPI 引脚被 NSS 硬件功能占用，与 GPIO 软件片选冲突。
- **修复**：核对硬件后统一 CS 引脚定义、PHYCFGR 改 8 位读、调整 GPIO 初始化顺序（先 GPIO 后 SPI）。
- **结果**：VERSIONR=0x04，PHYCFGR=0x07（LNK/SPD/DPX 全 1），ping 通、数据正常收发。
- **教训**：硬件调试以数据手册和实测回读为准，CS 引脚/寄存器位宽/复用功能逐一核对。

### 2. W5500 TX/RX 区块寻址宏写错（数据全 0 的根因）
- **现象**：TCP 能连上，但收发数据全 0。
- **根因**：早期把 TX/RX 区块号写成了错误偏移 `0x10+n`（甚至一度误记成 `2+n*3`）。
- **正解**：`TX_BLOCK(n)=2+4n`、`RX_BLOCK(n)=3+4n`（与 WIZnet 官方库一致）。
- **结果**：修正后 TCP 数据正常收发。

### 3. RS485 互斥锁死锁
- **现象**：健康巡检任务和 TCP 透传任务共享 RS485 总线，`osMutexAcquire(osWaitForever)` 导致任务永久阻塞。
- **修复**：阻塞路径（健康巡检）改 3 秒超时；非阻塞路径（TCP 网关）改 0 超时 + `g_rtu_busy` 门闩，拿不到锁跳过本轮。
- **教训**：RTOS 里等锁永远要设超时。

### 4. TCP 任务栈溢出
- **现象**：加入 OTA 后偶发闪存写入失败、行为诡异。
- **根因**：OTA 热路径栈占用 ≈2.6KB+，原 768×4=3KB 栈紧贴上限，溢栈静默踩坏相邻堆内存。
- **修复**：TcpServerTask 栈加到 1024×4=4KB。

### 5. FreeRTOS TCP 任务创建失败
- **根因**：早期栈设置过大（8KB）超出堆（3KB）。
- **修复**：`configTOTAL_HEAP_SIZE` 增大到 9216，并合理分配各任务栈。
