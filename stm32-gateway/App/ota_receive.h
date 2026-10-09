/* ============================================================================
 * ota_receive.h — OTA 固件接收模块 (应用侧)
 *
 * 通过 W5500 Socket 4 (port 506) 接收上位机固件,
 * 写入 W25Q64 暂存区, 校验后写控制块触发 Bootloader 升级。
 * ============================================================================
 */
#ifndef __OTA_RECEIVE_H
#define __OTA_RECEIVE_H

#include "ota_protocol_def.h"
#include <stdint.h>

/* 初始化 OTA 模块 (在 TcpServerTask 启动时调用) */
void OTA_Init(void);

/* Socket 4 收到数据时调用 */
void OTA_OnData(uint8_t sn, const uint8_t *data, uint16_t len);

/* 主循环中调用 (非阻塞擦除 tick + 超时检测) */
void OTA_Tick(void);

/* 中止当前 OTA */
void OTA_Abort(void);

/* 查询 OTA 是否正在进行 (日志任务借此暂停) */
int OTA_IsInProgress(void);

/* 发送 1 字节响应 */
void OTA_SendResponse(uint8_t sn, uint8_t code);

#endif /* __OTA_RECEIVE_H */
