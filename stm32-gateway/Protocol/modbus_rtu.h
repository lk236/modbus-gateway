#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include <stdint.h>

/* ======================== Modbus 功能码枚举 ======================== */
#define MODBUS_FC_READ_COILS            0x01  /* 读线圈 */
#define MODBUS_FC_READ_DISCRETE_INPUTS  0x02  /* 读离散输入 */
#define MODBUS_FC_READ_HOLDING_REGS     0x03  /* 读保持寄存器 (本项目主要用这个) */
#define MODBUS_FC_READ_INPUT_REGS       0x04  /* 读输入寄存器 */
#define MODBUS_FC_WRITE_SINGLE_COIL     0x05  /* 写单线圈 */
#define MODBUS_FC_WRITE_SINGLE_REG      0x06  /* 写单寄存器 */
#define MODBUS_FC_WRITE_MULTI_COILS     0x0F  /* 写多线圈 */
#define MODBUS_FC_WRITE_MULTI_REGS      0x10  /* 写多寄存器 */

/* RTU 层参数 */
#define MODBUS_RTU_TIMEOUT_MS    400   /* 单字节接收超时 (ms) */
#define MODBUS_RTU_RETRIES       5     /* 最大重试次数 */
#define MODBUS_RTU_FRAME_MAX     256   /* 最大帧长 */

/* Modbus RTU 操作状态 */
typedef enum {
    MODBUS_OK               = 0,  /* 成功 */
    MODBUS_ERROR_TIMEOUT    = 1,  /* 超时 (从机无响应) */
    MODBUS_ERROR_CRC        = 2,  /* CRC 校验错误 */
    MODBUS_ERROR_EXCEPTION  = 3,  /* 从机返回异常响应 */
    MODBUS_ERROR_INVALID    = 4,  /* 参数无效 */
    MODBUS_BUSY             = 5,  /* 总线忙 (非阻塞 API 用, 稍后再试) */
} Modbus_Status;

/* 非阻塞 RTU 事务 (Modbus TCP 网关多 Socket 用) */
typedef struct {
    const uint8_t *tx;         /* RTU 发送帧 (事务期间由调用方保持有效) */
    uint16_t tx_len;
    uint8_t  retries_left;     /* 剩余重试次数 */
    uint32_t deadline;         /* 当前等待的截止 tick */
    uint8_t  active;           /* 事务是否在途 */
} ModbusAsyncTx;

uint16_t Modbus_CRC16(uint8_t *buf, uint16_t len);
void Modbus_RTU_Init(void);
Modbus_Status Modbus_ReadHoldingRegs(uint8_t slave_addr, uint16_t start_addr,
                                      uint16_t reg_count, uint16_t *data);
Modbus_Status Modbus_SendRecvRTU(const uint8_t *tx_frame, uint16_t tx_len,
                                  uint8_t *rx_frame, uint16_t *rx_len);
const char* Modbus_StatusStr(Modbus_Status status);

/* ---- 非阻塞事务 API ---- */
void Modbus_AsyncInit(ModbusAsyncTx *t);
/* 开始事务(非阻塞拿总线):
 *   MODBUS_OK   → 已发送, 进入等待(用 AsyncPoll 轮询)
 *   MODBUS_BUSY → 总线被占(健康检查等), 稍后重试
 *   其他        → 参数/CRC 错误, 事务未开始 */
Modbus_Status Modbus_AsyncBegin(ModbusAsyncTx *t, uint8_t *tx_frame, uint16_t tx_len);
/* 轮询在途事务:
 *   MODBUS_BUSY → 仍在等待
 *   MODBUS_OK / MODBUS_ERROR_EXCEPTION → 帧已到(rx/rx_len 有效), 事务结束
 *   MODBUS_ERROR_TIMEOUT → 重试耗尽, 事务结束 */
Modbus_Status Modbus_AsyncPoll(ModbusAsyncTx *t, uint8_t *rx, uint16_t *rx_len);
/* 中止在途事务(释放总线), 未开始则为 no-op */
void Modbus_AsyncAbort(ModbusAsyncTx *t);

#endif
