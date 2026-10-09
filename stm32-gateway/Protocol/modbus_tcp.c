/*
 * ============================================================================
 * modbus_tcp.c — Modbus TCP ↔ Modbus RTU 协议转换(网关的"翻译官")
 * ============================================================================
 *
 * 【两种帧格式对比】
 *   TCP 帧: [MBAP 7 字节][功能码][数据...]
 *   RTU 帧: [地址 1 字节][功能码][数据...][CRC16 2 字节]
 *
 * 【转换规则】
 *   TCP→RTU:  MBAP.UnitID → 从机地址; 去掉 MBAP; 补上 CRC16
 *   RTU→TCP:  从机地址 → MBAP.UnitID; 去掉 CRC;  补上 MBAP
 *
 * 【为什么会有这个文件】
 *   这是整个网关最核心的"协议翻译"逻辑: 同一份 Modbus 请求, 以太网侧走
 *   Modbus-TCP, 现场总线侧走 Modbus-RTU, 网关负责在两者之间来回转换。
 *   网线上的 PLC / 上位机只会说 TCP, 现场的传感器只会说 RTU, 网关就得当翻译官。
 *
 * 【MBAP 与 RTU 的本质差异】
 *   - TCP 有 7 字节报文头(内含"事务 ID"用来配对请求与响应), 没有 CRC ——
 *     因为 TCP 本身是可靠传输, 不需要应用层再校验;
 *   - RTU 没有报文头, 但有 CRC16 —— 串口物理层不可靠, 必须自己校验。
 * ============================================================================
 */
#include "modbus_tcp.h"   /* 本模块接口: MBAP 结构体、转换函数声明 */
#include "modbus_rtu.h"   /* 需要复用 Modbus_CRC16() 和功能码常量 */
#include <string.h>       /* memcpy */
#include <stdio.h>        /* printf 调试打印 */

/* ============================================================================
 * MBAP 头构造 / 解析
 *
 * MBAP (Modbus Application Protocol Header) 是 Modbus-TCP 独有的 7 字节头部:
 *   [0..1]  Transaction ID  事务标识 (客户端自己分配的流水号, 用于配对请求/响应)
 *   [2..3]  Protocol ID     协议标识, 恒为 0x0000
 *   [4..5]  Length          后续字节数 = 1(UnitID) + PDU 长度
 *   [6]     Unit ID         单元标识 (相当于 RTU 里的从机地址)
 * 所有多字节字段都是大端序(网络字节序)。
 * ============================================================================
 */

/* 构造 MBAP 头(7 字节): [TransID(2)][ProtoID(2)=0][Length(2)][UnitID(1)]
 * 参数 len 传的是"Length 字段的值", 即 UnitID + PDU 的字节数。
 * @return 恒为 7 (MBAP 头长度) */
uint8_t ModbusTCP_BuildMBAP(uint8_t *tcp_buf, uint16_t trans_id, uint16_t len, uint8_t unit_id)
{
    tcp_buf[0] = (trans_id >> 8) & 0xFF;   /* Transaction ID 高字节(大端先发) */
    tcp_buf[1] = trans_id & 0xFF;          /* Transaction ID 低字节 */
    tcp_buf[2] = 0x00;                     /* Protocol ID 高字节 = 0 */
    tcp_buf[3] = 0x00;                     /* Protocol ID 低字节 = 0 (Modbus 恒为 0) */
    tcp_buf[4] = (len >> 8) & 0xFF;        /* Length 高字节 */
    tcp_buf[5] = len & 0xFF;               /* Length 低字节 */
    tcp_buf[6] = unit_id;                  /* Unit ID(从机地址), 单字节无需拆分 */

    return MBAP_HEADER_LEN;                /* 返回头长度 7 */
}

/* 解析 MBAP 头(把大端字节流还原成结构体字段)
 * @return 7 = 成功; 0 = 数据不足 7 字节, 不是一个完整头 */
uint8_t ModbusTCP_ParseMBAP(const uint8_t *tcp_buf, uint16_t tcp_len, MBAP_Header *mbap)
{
    if (tcp_len < MBAP_HEADER_LEN) {   /* 不够 7 字节, 连头都不完整 */
        return 0;
    }

    /* 大端 → 主机序: 高字节在前, 左移 8 位再或上低字节 */
    mbap->transaction_id = ((uint16_t)tcp_buf[0] << 8) | tcp_buf[1];
    mbap->protocol_id    = ((uint16_t)tcp_buf[2] << 8) | tcp_buf[3];
    mbap->length         = ((uint16_t)tcp_buf[4] << 8) | tcp_buf[5];
    mbap->unit_id        = tcp_buf[6];     /* 单字节直接取 */

    return MBAP_HEADER_LEN;                /* 返回头长度 7 */
}

/* ============================================================================
 * RTU → TCP: 去 CRC(2B), 把地址装进 UnitID, 前面加 MBAP 头
 *
 *   RTU 响应帧: [addr][func][data...][CRC_lo][CRC_hi]        (rtu_len 字节)
 *   TCP 响应帧: [MBAP 7B][func][data...]                     (7 + rtu_len - 3 字节)
 *
 *   RTU 的"地址字节"被吃掉(由 MBAP.UnitID 代替),
 *   RTU 的"CRC"也被去掉(Modbus-TCP 不用 CRC)。
 * 参数 trans_id 由调用方从原请求的 MBAP 里取, 原样回传, 客户端靠它配对。
 * @return TCP 帧总字节数*/
uint16_t ModbusTCP_RTUtoTCP(const uint8_t *rtu_buf, uint16_t rtu_len,
                            uint8_t *tcp_buf, uint16_t trans_id, uint8_t unit_id)
{
    if (rtu_len < 3) return 0;             /* 至少要有 addr + func + 1 字节 CRC 才算帧 */

    /* PDU = 整帧去掉 [addr(1) + CRC(2)] = 剩下的 func + data */
    uint16_t pdu_len = rtu_len - 3;

    /* 组 MBAP 头: Length 字段 = UnitID(1) + PDU 长度 */
    ModbusTCP_BuildMBAP(tcp_buf, trans_id, pdu_len + 1, unit_id);

    /* 拷贝 [func + data] 到 MBAP 之后, 跳过 RTU 的地址字节(rtu_buf + 1) */
    memcpy(tcp_buf + MBAP_HEADER_LEN, rtu_buf + 1, pdu_len);

    return MBAP_HEADER_LEN + pdu_len;      /* 返回 TCP 帧总长 */
}

/* ============================================================================
 * TCP → RTU: UnitID 当从机地址, 去掉 MBAP(7B), 补上 CRC(2B)
 *
 *   TCP 请求帧: [MBAP 7B][func][data...]                     (tcp_len 字节)
 *   RTU 请求帧: [addr][func][data...][CRC_lo][CRC_hi]
 * @return RTU 帧总字节数 = 1(addr) + PDU + 2(CRC); 非法输入返回 0 */
uint16_t ModbusTCP_TCPtoRTU(const uint8_t *tcp_buf, uint16_t tcp_len, uint8_t *rtu_buf)
{
    if (tcp_len < MBAP_HEADER_LEN) return 0;   /* 连 MBAP 头都不够 */

    MBAP_Header mbap;
    ModbusTCP_ParseMBAP(tcp_buf, tcp_len, &mbap);   /* 上面已保证 tcp_len>=7, 这里必定成功 */

    /* PDU = func + data。Modbus 规定 PDU 最短 1 字节、最长 253 字节 */
    uint16_t pdu_len = tcp_len - MBAP_HEADER_LEN;
    if (pdu_len == 0 || pdu_len > 253) return 0;    /* 超范围视为非法帧 */

    /* 把 MBAP 的 Unit ID 作为 RTU 的从机地址 */
    rtu_buf[0] = mbap.unit_id;
    memcpy(rtu_buf + 1, tcp_buf + MBAP_HEADER_LEN, pdu_len);   /* 接着拷贝 PDU */

    /* 给 RTU 帧补 CRC16(注意: 低字节在前) */
    uint16_t crc = Modbus_CRC16(rtu_buf, 1 + pdu_len);         /* 对 [addr+PDU] 算 CRC */
    rtu_buf[1 + pdu_len]     = crc & 0xFF;                     /* CRC 低字节 */
    rtu_buf[1 + pdu_len + 1] = (crc >> 8) & 0xFF;              /* CRC 高字节 */

    return 1 + pdu_len + 2;                                    /* 返回 RTU 帧总长 */
}

/* 功能码 → 可读名称, 只用于打印调试 */
const char* ModbusTCP_FuncName(uint8_t func_code)
{
    switch (func_code) {
        case MODBUS_FC_READ_COILS:            return "RD_COIL";
        case MODBUS_FC_READ_DISCRETE_INPUTS:  return "RD_DISC";
        case MODBUS_FC_READ_HOLDING_REGS:     return "RD_HOLD";
        case MODBUS_FC_READ_INPUT_REGS:       return "RD_INP";
        case MODBUS_FC_WRITE_SINGLE_COIL:     return "WR_COIL";
        case MODBUS_FC_WRITE_SINGLE_REG:      return "WR_REG";
        case MODBUS_FC_WRITE_MULTI_COILS:     return "WRM_COIL";
        case MODBUS_FC_WRITE_MULTI_REGS:      return "WRM_REG";
        default:                              return "?";
    }
}

/* ============================================================================
 * Modbus TCP 请求处理主入口(阻塞式, 单事务场景用)
 *
 * 完整流程: 解析 MBAP → TCPtoRTU → 发 RS-485 → 收响应 → RTUtoTCP
 *
 * 【注意】TcpServerTask 的多 Socket 场景实际用的是 app_modbus.c 里的非阻塞路径,
 *   这个函数保留作"单事务 / 阻塞"的参考实现, 也方便单测。
 * ============================================================================
 */
ModbusTCP_Status ModbusTCP_HandleRequest(const uint8_t *tcp_buf, uint16_t tcp_len,
                                          uint8_t *resp_buf, uint16_t *resp_len)
{
    MBAP_Header mbap;

    /* 1. 解析 MBAP 头(顺便校验长度够不够) */
    if (ModbusTCP_ParseMBAP(tcp_buf, tcp_len, &mbap) == 0) {
        return MODBUS_TCP_ERROR_INVALID;
    }

    /* 2. 校验 PDU 长度合法性(1~253) */
    uint16_t pdu_len = tcp_len - MBAP_HEADER_LEN;
    if (pdu_len < 1 || pdu_len > 253) return MODBUS_TCP_ERROR_INVALID;

    uint8_t func_code = tcp_buf[MBAP_HEADER_LEN];  /* 功能码: PDU 的第 1 字节 */
    uint8_t unit_id   = mbap.unit_id;              /* 从机地址: 来自 MBAP.UnitID */

    /* 3. TCP 帧 → RTU 帧(去 MBAP、加 CRC) */
    uint8_t rtu_tx[MODBUS_RTU_FRAME_MAX];          /* RTU 请求帧缓冲 */
    uint16_t rtu_tx_len = ModbusTCP_TCPtoRTU(tcp_buf, tcp_len, rtu_tx);
    if (rtu_tx_len == 0) return MODBUS_TCP_ERROR_INVALID;

    printf("[TCP->RTU] slave=%d fc=%s len=%d\r\n", unit_id, ModbusTCP_FuncName(func_code), rtu_tx_len);

    /* 4. 通过 RS-485 与从机交互(这里用阻塞收发) */
    uint8_t rtu_rx[MODBUS_RTU_FRAME_MAX];          /* RTU 响应帧缓冲 */
    uint16_t rtu_rx_len = 0;
    Modbus_Status rtu_status = Modbus_SendRecvRTU(rtu_tx, rtu_tx_len, rtu_rx, &rtu_rx_len);

    if (rtu_status == MODBUS_OK) {
        /* 5a. 正常 → RTU 响应转成 TCP 响应回传(事务 ID 原样带回) */
        *resp_len = ModbusTCP_RTUtoTCP(rtu_rx, rtu_rx_len, resp_buf, mbap.transaction_id, unit_id);
        printf("[RTU->TCP] OK, %d bytes\r\n", *resp_len);
        return MODBUS_TCP_OK;
    }

    if (rtu_status == MODBUS_ERROR_EXCEPTION) {
        /* 5b. 从机明确返回异常(如非法地址) → 原样转发给 TCP 客户端 */
        *resp_len = ModbusTCP_RTUtoTCP(rtu_rx, rtu_rx_len, resp_buf, mbap.transaction_id, unit_id);
        printf("[RTU->TCP] EXCEPTION\r\n");
        return MODBUS_TCP_ERROR_EXCEPTION;
    }

    /* 5c. 通信失败(超时 / CRC 错) → 网关自己构造一个标准异常响应给客户端
     *     异常帧格式: [MBAP][fc|0x80][异常码], Length = 3 (UnitID + FC + 异常码)
     *     异常码 0x0B = "网关目标设备无响应"(Gateway Target Device Failed To Respond) */
    printf("[RTU->TCP] FAIL: %s\r\n", Modbus_StatusStr(rtu_status));
    ModbusTCP_BuildMBAP(resp_buf, mbap.transaction_id, 3, unit_id);
    resp_buf[MBAP_HEADER_LEN]     = func_code | 0x80;  /* 异常标志位: 功能码最高位置 1 */
    resp_buf[MBAP_HEADER_LEN + 1] = 0x0B;              /* 异常码 0x0B: 网关目标设备无响应 */
    *resp_len = MBAP_HEADER_LEN + 2;                   /* 7 + 2 = 9 字节 */
    return MODBUS_TCP_ERROR_TIMEOUT;
}
