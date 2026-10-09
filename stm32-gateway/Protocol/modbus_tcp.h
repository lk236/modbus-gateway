#ifndef __MODBUS_TCP_H
#define __MODBUS_TCP_H

#include <stdint.h>

/* Modbus TCP 默认端口 */
#define MODBUS_TCP_PORT         502

/* 最大帧长 (MBAP 7 + PDU 253) */
#define MODBUS_TCP_FRAME_MAX    260

/* MBAP 头长度 */
#define MBAP_HEADER_LEN         7

/* Modbus TCP 操作状态 */
typedef enum {
    MODBUS_TCP_OK              = 0,  /* 成功 */
    MODBUS_TCP_ERROR_TIMEOUT   = 1,  /* 超时 */
    MODBUS_TCP_ERROR_CRC       = 2,  /* CRC 错误 (当前未使用) */
    MODBUS_TCP_ERROR_EXCEPTION = 3,  /* 从机异常 */
    MODBUS_TCP_ERROR_INVALID   = 4,  /* 无效参数 */
    MODBUS_TCP_ERROR_NODATA    = 5,  /* 无数据 */
} ModbusTCP_Status;

/* MBAP (Modbus Application Protocol) 头结构 */
typedef struct {
    uint16_t transaction_id;  /* 事务标识符 (应答时回传) */
    uint16_t protocol_id;     /* 协议标识符 (Modbus = 0x0000) */
    uint16_t length;          /* 后续字节长度 (UnitID + PDU) */
    uint8_t  unit_id;         /* 从机地址 (映射到 RTU 的 Addr) */
} MBAP_Header;

uint8_t ModbusTCP_BuildMBAP(uint8_t *tcp_buf, uint16_t trans_id, uint16_t len, uint8_t unit_id);

uint8_t ModbusTCP_ParseMBAP(const uint8_t *tcp_buf, uint16_t tcp_len, MBAP_Header *mbap);

uint16_t ModbusTCP_RTUtoTCP(const uint8_t *rtu_buf, uint16_t rtu_len,
                            uint8_t *tcp_buf, uint16_t trans_id, uint8_t unit_id);

uint16_t ModbusTCP_TCPtoRTU(const uint8_t *tcp_buf, uint16_t tcp_len, uint8_t *rtu_buf);

ModbusTCP_Status ModbusTCP_HandleRequest(const uint8_t *tcp_buf, uint16_t tcp_len,
                                          uint8_t *resp_buf, uint16_t *resp_len);

const char* ModbusTCP_FuncName(uint8_t func_code);

#endif
