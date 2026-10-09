#ifndef __BSP_W25Q64_H
#define __BSP_W25Q64_H

/*
 * bsp_w25q64.h — W25Q64 SPI Flash 存储驱动接口
 *
 * 存储布局:
 *   0x000000 ~ 0x000FFF  配置扇区 (4KB)
 *   0x001000 ~ 0x001FFF  日志元数据 (写指针 + 条目数)
 *   0x002000 ~ 0x7FFFFF  日志数据环形缓冲区 (最多 2048 条)
 */

#include "main.h"
#include "spi.h"
#include "cmsis_os.h"

/* ======================== 容量参数 ======================== */
#define W25Q64_PAGE_SIZE            256U    /* 页大小 */
#define W25Q64_SECTOR_SIZE          4096U   /* 扇区大小 (最小擦除单位) */
#define W25Q64_BLOCK_SIZE           65536U  /* 块大小 */
#define W25Q64_CAPACITY             8388608U /* 8MB */
#define W25Q64_PAGE_COUNT           32768U
#define W25Q64_SECTOR_COUNT         2048U
#define W25Q64_BLOCK_COUNT          128U

/* ======================== SPI 指令 ======================== */
#define W25Q64_CMD_WRITE_ENABLE     0x06  /* 写使能 */
#define W25Q64_CMD_WRITE_DISABLE    0x04  /* 写禁止 */
#define W25Q64_CMD_READ_STATUS1     0x05  /* 读状态寄存器 1 */
#define W25Q64_CMD_READ_STATUS2     0x35
#define W25Q64_CMD_READ_STATUS3     0x15
#define W25Q64_CMD_WRITE_STATUS     0x01
#define W25Q64_CMD_PAGE_PROGRAM     0x02  /* 页编程 (一页最多 256 字节) */
#define W25Q64_CMD_READ_DATA        0x03  /* 读数据 */
#define W25Q64_CMD_FAST_READ        0x0B  /* 快速读 (带 dummy) */
#define W25Q64_CMD_SECTOR_ERASE     0x20  /* 扇区擦除 (4KB) */
#define W25Q64_CMD_BLOCK_ERASE_32K  0x52  /* 32KB 块擦除 */
#define W25Q64_CMD_BLOCK_ERASE_64K  0xD8  /* 64KB 块擦除 */
#define W25Q64_CMD_CHIP_ERASE       0xC7  /* 全片擦除 (~60s) */
#define W25Q64_CMD_POWER_DOWN       0xB9
#define W25Q64_CMD_RELEASE_POWERDOWN 0xAB
#define W25Q64_CMD_READ_JEDEC_ID    0x9F  /* 读 JEDEC ID */
#define W25Q64_CMD_READ_MAN_ID      0x90

/* 状态寄存器位定义 */
#define W25Q64_SR1_BUSY             0x01  /* BUSY: 1=忙, 0=空闲 */
#define W25Q64_SR1_WEL              0x02  /* WEL: 写使能锁存 */

/* ======================== 硬件接口 (SPI1, CS=PA4) ======================== */
#define W25Q64_CS_PORT              GPIOA
#define W25Q64_CS_PIN               GPIO_PIN_4
#define W25Q64_CS_LOW()             HAL_GPIO_WritePin(W25Q64_CS_PORT, W25Q64_CS_PIN, GPIO_PIN_RESET)
#define W25Q64_CS_HIGH()            HAL_GPIO_WritePin(W25Q64_CS_PORT, W25Q64_CS_PIN, GPIO_PIN_SET)

/* ======================== 存储分区 ======================== */
#define W25Q64_CONFIG_ADDR          0x00000000U   /* 配置存储地址 */
#define W25Q64_CONFIG_SIZE          W25Q64_SECTOR_SIZE  /* 4KB */
#define W25Q64_LOG_ADDR             (W25Q64_CONFIG_ADDR + W25Q64_SECTOR_SIZE)  /* 0x1000 */
#define W25Q64_LOG_SIZE             (W25Q64_CAPACITY - W25Q64_SECTOR_SIZE)
#define W25Q64_LOG_SECTOR_COUNT     (W25Q64_SECTOR_COUNT - 1)

#define W25Q64_LOG_META_ADDR        W25Q64_LOG_ADDR           /* 0x1000: 日志元数据 */
#define W25Q64_LOG_DATA_ADDR        (W25Q64_LOG_ADDR + W25Q64_SECTOR_SIZE)  /* 0x2000: 日志数据 */

/* 日志最大条目数 (剩余 7.5MB / 8 字节每条目) */
#define W25Q64_LOG_ENTRY_MAX        2048U

/* ======================== 日志类型枚举 ======================== */
typedef enum {
    W25Q64_LOG_SYSTEM_BOOT    = 0x01,  /* 系统启动 */
    W25Q64_LOG_TCP_CONNECT    = 0x02,  /* TCP 客户端连接 */
    W25Q64_LOG_TCP_DISCONNECT = 0x03,  /* TCP 客户端断开 */
    W25Q64_LOG_485_ERROR      = 0x04,  /* RS-485 总线故障 */
    W25Q64_LOG_485_OK         = 0x05,  /* RS-485 总线恢复 */
    W25Q64_LOG_CFG_UPDATED    = 0x06,  /* 配置更新 */
    W25Q64_LOG_SENSOR_DATA    = 0x07,  /* 传感器数据 */
} W25Q64_LogType;

/* 日志条目结构体 (8 字节) */
#pragma pack(1)
typedef struct {
    uint32_t timestamp;  /* Unix 时间戳 (从 DS3231 RTC 获取) */
    uint8_t  type;       /* 日志类型 (W25Q64_LogType) */
    uint8_t  reserved;   /* 保留 */
    uint16_t data;       /* 关联数据 (如端口号、计数) */
} W25Q64_LogEntry;
#pragma pack()

/* 系统配置结构体 (256 字节, 正好一页) */
#pragma pack(1)
typedef struct {
    uint32_t magic;              /* 魔数 0xA5C3F0E1, 标记配置有效性 */
    uint8_t  gateway_ip[4];      /* 网关 IP 地址 */
    uint8_t  subnet_mask[4];     /* 子网掩码 */
    uint8_t  default_gw[4];      /* 默认网关 */
    uint16_t poll_cycle_ms;      /* RS-485 轮询间隔 (毫秒) */
    uint32_t rs485_baudrate;     /* RS-485 波特率 */
    uint8_t  slave_id;           /* Modbus 从机地址 */
    uint8_t  dhcp_enable;        /* DHCP 使能 (0=关, 1=开) */
    uint16_t tcp_port;           /* Modbus-TCP 监听端口 (默认 502) */
    uint8_t  reserved[234];      /* 保留, 填满 256 字节 */
    uint32_t crc32;              /* CRC32 校验 (前面所有字段) */
} W25Q64_Config;
#pragma pack()

#define W25Q64_CONFIG_MAGIC     0xA5C3F0E1U  /* 配置魔数 */

/* 操作返回值 */
typedef enum {
    W25Q64_OK       = 0,
    W25Q64_ERROR    = 1,
    W25Q64_BUSY     = 2,
    W25Q64_TIMEOUT  = 3,
    W25Q64_INVALID  = 4,
} W25Q64_Status;

/* 基础 API */
W25Q64_Status BSP_W25Q64_Init(void);
W25Q64_Status BSP_W25Q64_ReadJEDEC_ID(uint8_t *manuf, uint16_t *dev_id);
W25Q64_Status BSP_W25Q64_SectorErase(uint32_t addr);
W25Q64_Status BSP_W25Q64_PageWrite(uint32_t addr, uint8_t *buf, uint16_t len);
W25Q64_Status BSP_W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len);
W25Q64_Status BSP_W25Q64_Write(uint32_t addr, uint8_t *buf, uint32_t len);
W25Q64_Status BSP_W25Q64_ChipErase(void);

/* 配置 API */
W25Q64_Status BSP_W25Q64_ConfigLoad(W25Q64_Config *cfg);
W25Q64_Status BSP_W25Q64_ConfigSave(W25Q64_Config *cfg);
W25Q64_Status BSP_W25Q64_ConfigDefault(W25Q64_Config *cfg);
uint8_t       BSP_W25Q64_ConfigIsValid(W25Q64_Config *cfg);

/* 日志 API */
W25Q64_Status BSP_W25Q64_LogWrite(W25Q64_LogEntry *entry);
W25Q64_Status BSP_W25Q64_LogRead(uint16_t index, W25Q64_LogEntry *entry);
uint16_t      BSP_W25Q64_LogGetCount(void);
W25Q64_Status BSP_W25Q64_LogClear(void);

/* OTA 异步擦除支持 */
void BSP_W25Q64_SendEraseCmd(uint32_t addr);
int  BSP_W25Q64_IsBusy(void);
void BSP_W25Q64_ReadStatus2(uint8_t *sr2);
W25Q64_Status BSP_W25Q64_WaitBusy(uint32_t timeout_ms);  
/* 测试 API */
void BSP_W25Q64_Test(void);

extern W25Q64_Config g_gateway_config;

#endif
