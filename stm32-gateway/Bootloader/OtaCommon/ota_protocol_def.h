/* ============================================================================
 * ota_protocol_def.h — OTA 协议公共定义
 *
 * 本文件为 Bootloader 与 App 共用，禁止复制粘贴式实现。
 * 所有多字节字段采用大端序（网络字节序）。
 * ============================================================================
 */
#ifndef __OTA_PROTOCOL_DEF_H
#define __OTA_PROTOCOL_DEF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 字节序转换宏 ======================== */
#if defined(STM32F103xB)
#include "stm32f1xx_hal.h"
#define HTONL(x)  __REV(x)
#define HTONS(x)  __REV16(x)
#define NTOHL(x)  __REV(x)
#define NTOHS(x)  __REV16(x)
#elif defined(__linux__)
#include <arpa/inet.h>
#define HTONL(x)  htonl(x)
#define HTONS(x)  htons(x)
#define NTOHL(x)  ntohl(x)
#define NTOHS(x)  ntohs(x)
#elif defined(_WIN32)
#include <winsock2.h>
#define HTONL(x)  htonl(x)
#define HTONS(x)  htons(x)
#define NTOHL(x)  ntohl(x)
#define NTOHS(x)  ntohs(x)
#else
#error "ota_protocol_def.h: Unknown platform, define STM32F103xB / __linux__ / _WIN32"
#endif

/* ======================== Magic 常量 ======================== */
#define OTA_CMD_MAGIC       0x4F544143UL    /* "OTAC" */
#define OTA_DATA_MAGIC      0x4F544150UL    /* "OTAP" */
#define OTA_STATUS_MAGIC    0x4F544153UL    /* "OTAS" */
#define OTA_CTRL_MAGIC      0xA5A5A5A5UL    /* 控制块有效标记 */
#define FW_HEADER_MAGIC     0x504A3142UL    /* "PJ1B" */

/* ======================== 命令码 ======================== */
#define CMD_START           0x00
#define CMD_FINISH          0x01
#define CMD_CANCEL          0x02
#define CMD_STATUS          0x03

/* ======================== 响应码 (1 字节) ======================== */
#define RSP_ACK             0x06
#define RSP_NACK            0x15
#define RSP_ABORT           0x18
#define RSP_BUSY            0x19
#define RSP_ERR_ERASE       0x1A
#define RSP_ERR_WRITE       0x1B
#define RSP_ERR_OOB         0x1C

/* ======================== 错误码 (uint8_t) ======================== */
#define ERR_NONE            0x00
#define ERR_CTRL_ERASE      0x01
#define ERR_CTRL_WRITE      0x02
#define ERR_FW_CRC          0x03
#define ERR_FLASH_WRITE     0x04
#define ERR_STAGING_CRC     0x05
#define ERR_SIZE_INVALID    0x06
#define ERR_OOB             0x07

/* ======================== 协议参数 ======================== */
#define OTA_PORT                506
#define OTA_PACKET_SIZE         512
#define OTA_TOTAL_MAX           0xC000      /* 48KB */
#define OTA_TOTAL_MIN           1024        /* 1KB */
#define OTA_TIMEOUT_MS          5000
#define OTA_BUSY_TOTAL_MS       30000

/* ======================== Flash 地址 ======================== */
#define BOOTLOADER_ADDR         0x08000000UL
#define APP_ADDR                0x08004000UL
#define APP_MAX_SIZE            0xC000UL
#define APP_MIN_SIZE            1024UL
#define APP_RAM_START           0x20000000UL
#define APP_RAM_END             0x20005000UL
#define CTRL_BLOCK_ADDR         0x007000UL  /* W25Q64 OTA 控制块 */
#define STAGING_ADDR            0x010000UL  /* W25Q64 固件暂存区 */
#define FW_HEADER_OFFSET        16UL        /* 暂存区中 Header 大小 */
#define LOG_AREA_ADDR           0x080000UL  /* 日志区起始 */

/* ======================== Rollback 回滚地址 ======================== */
#define ROLLBACK_BACKUP_ADDR    (STAGING_ADDR + APP_MAX_SIZE)  /* 0x01C000, 固件备份区 */
#define ROLLBACK_CTRL_ADDR      (CTRL_BLOCK_ADDR + 0x20)       /* 0x007020, Rollback 控制块 */
#define ROLLBACK_MAGIC          0x5242424BUL                   /* "RBBK" */

/* ======================== 数据结构 ======================== */

/* 命令帧 (16 字节) */
#pragma pack(1)
typedef struct {
    uint32_t magic;         /* "OTAC" */
    uint8_t  cmd;           /* CMD_START / FINISH / CANCEL / STATUS */
    uint8_t  reserved;      /* 保留 */
    uint16_t fw_version;    /* htons, CMD_START 时有效 */
    uint32_t total_size;    /* htonl */
    uint32_t total_crc;     /* htonl */
} OTACmd;

/* 数据帧 (12+N 字节) */
typedef struct {
    uint32_t magic;         /* "OTAP" */
    uint32_t block_num;     /* htonl, 0-based */
    uint16_t data_len;      /* htons, 1~512 */
    uint16_t block_crc;     /* htons, CRC16-Modbus */
    uint8_t  data[OTA_PACKET_SIZE];
} OTAData;

/* 状态响应 (12 字节) */
typedef struct {
    uint32_t magic;         /* "OTAS", htonl */
    uint8_t  stage;         /* 0=idle 1=receiving 2=verifying */
    uint8_t  progress_pct;  /* 0~100 */
    uint8_t  last_error;    /* uint8_t, 不转 */
    uint8_t  reserved;
    uint32_t fw_version;    /* htonl */
} OTAStatusResp;

/* OTA 控制块 (19 字节, W25Q64 0x007000) */
/*
 * fail_flag 规则:
 *   0xFF 0xFF 0xFF → 3 次机会
 *   0x00 0xFF 0xFF → 1 次失败
 *   0x00 0x00 0xFF → 2 次失败
 *   0x00 0x00 0x00 → 耗尽 → 故障态
 *   统计: ≠0xFF 即为失败次数
 *   写入: 仅 1→0, 无需擦扇区
 *
 * last_error 规则:
 *   仅首次失败时写入 (0xFF→error_code), 后续绝不覆盖
 *   成功升级时擦扇区后重置为 ERR_NONE
 */
typedef struct {
    uint32_t magic;          /* [0~3]   4B   0xA5A5A5A5 */
    uint32_t firmware_size;  /* [4~7]   4B   [1024,49152], 偶数 */
    uint32_t firmware_crc32; /* [8~11]  4B   CRC32-ZIP (纯固件) */
    uint8_t  fail_flag[3];   /* [12~14] 3B   0xFF=OK, 0x00=fail */
    uint8_t  last_error;     /* [15]    1B   仅首次写入 */
    uint8_t  reserved;       /* [16]    1B   0xFF */
    uint16_t ctrl_crc16;     /* [17~18] 2B   CRC16-Modbus(bytes[0~11]) */
} OTACtrlBlock;              /* total = 19 bytes */

/* 固件头部 (16 字节, 附在 .bin 文件前) */
typedef struct {
    uint32_t magic;     /* "PJ1B" = 0x504A3142 */
    uint32_t version;   /* 0x0100 = v1.0 */
    uint32_t size;      /* 纯固件大小 (已半字对齐) */
    uint32_t crc32;     /* 纯固件 CRC32-ZIP */
} FirmwareHeader;
#pragma pack()

/* ======================== 操作阶段枚举 ======================== */
typedef enum {
    OTA_STAGE_IDLE       = 0,
    OTA_STAGE_RECEIVING  = 1,
    OTA_STAGE_VERIFYING  = 2,
} OTAStage;

/* ======================== OTA 状态 (linux-host 使用) ======================== */
typedef enum {
    OTA_STATUS_IDLE           = 0,
    OTA_STATUS_SENDING        = 1,
    OTA_STATUS_VERIFYING      = 2,
    OTA_STATUS_SUCCESS        = 3,
    OTA_STATUS_FAILED         = -1,
    OTA_STATUS_CANCELLED      = -2,
    OTA_STATUS_PENDING_VERIFY = 4,
} OTAStatus;

/* ======================== CRC 常量 ======================== */
/* CTRL_CRC_LEN: 控制块 CRC16 计算范围 = bytes[0~11] (magic+size+crc32)
   不包含 fail_flag 和 last_error，因为 fail_flag 需独立 1→0 修改
   若含入 CRC，每次 MarkOneFail 后 CRC 会不匹配，破坏重试机制 */
#define CTRL_CRC_LEN        12

#ifdef __cplusplus
}
#endif

#endif /* __OTA_PROTOCOL_DEF_H */
