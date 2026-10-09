#ifndef __MODBUS_MASTER_H
#define __MODBUS_MASTER_H

/*
 * modbus_master.h — Modbus TCP 主站封装接口
 *
 * 基于 libmodbus 库，提供简洁的 Modbus TCP 主站功能。
 * 支持连接管理、读保持寄存器、写单寄存器。
 *
 * 使用示例:
 *   ModbusMaster mb;
 *   if (Modbus_Connect(&mb, "192.168.1.100", 502) == 0) {
 *       uint16_t data[2];
 *       if (Modbus_ReadHoldingRegs(&mb, 1, 0x0300, 2, data) == 0) {
 *           printf("Temp=%.1f Hum=%.1f\n", data[0]/10.0, data[1]/10.0);
 *       }
 *       Modbus_Close(&mb);
 *   }
 */

#include <stdint.h>
#include <stdbool.h>

/* Modbus 主机上下文 */
typedef struct {
    void    *ctx;          /* modbus_t* (不透明的指针) */
    char     host[64];     /* 网关 IP */
    int      port;         /* 网关端口 */
    int      connected;    /* 是否已连接 */
    uint32_t last_err_ms;  /* 上次错误时刻 (ms) */
    int      err_count;    /* 连续错误计数 */
} ModbusMaster;

/**
 * @brief 连接 Modbus TCP 网关
 * @param mb    主机上下文（调用前需 memset 或 Modbus_Init）
 * @param host  网关 IP 地址
 * @param port  网关端口（通常 502）
 * @return 0=成功, -1=失败
 */
int Modbus_Connect(ModbusMaster *mb, const char *host, int port);

/**
 * @brief 断开连接
 */
void Modbus_Disconnect(ModbusMaster *mb);

/**
 * @brief 自动重连（如果已断开则尝试重连）
 * @return 0=已连接或重连成功, -1=重连失败
 */
int Modbus_Reconnect(ModbusMaster *mb);

/**
 * @brief 读保持寄存器 (功能码 03)
 * @param mb        主机上下文
 * @param slave     从机地址
 * @param addr      寄存器起始地址
 * @param count     读取数量
 * @param dest      输出缓冲区
 * @return 0=成功, -1=失败
 */
int Modbus_ReadHoldingRegs(ModbusMaster *mb, int slave,
                           int addr, int count, uint16_t *dest);

/**
 * @brief 读输入寄存器 (功能码 04)
 */
int Modbus_ReadInputRegs(ModbusMaster *mb, int slave,
                         int addr, int count, uint16_t *dest);

/**
 * @brief 写单寄存器 (功能码 06)
 */
int Modbus_WriteSingleReg(ModbusMaster *mb, int slave,
                          int addr, uint16_t value);

/**
 * @brief 检查当前是否已连接
 */
int Modbus_IsConnected(ModbusMaster *mb);

/**
 * @brief 获取上次错误信息
 */
const char *Modbus_LastError(ModbusMaster *mb);

#endif /* __MODBUS_MASTER_H */
