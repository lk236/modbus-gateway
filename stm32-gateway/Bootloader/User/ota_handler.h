/* ============================================================================
 * ota_handler.h — OTA 协议处理接口
 *
 * 【模块职责】
 *   故障态 Bootloader 的 OTA 服务端核心逻辑: 环形缓冲区、粘包解帧、
 *   命令/数据处理、异步扇区擦除、Socket 4 状态轮询。
 *
 * 【调用关系 (谁调谁)】
 *   main → Bootloader_EnterFailsafe → 主循环里反复调:
 *       OTA_Tick()        推进异步擦除 + 收包超时
 *       OTA_PollSocket4() 轮询 TCP 连接, 收到字节就 OTA_OnData
 *   收到数据 → OTA_OnData(入环形缓冲) → ota_try_parse_frame(解帧)
 *            → OTA_HandleCmd / OTA_HandleData(处理)
 * ============================================================================
 */
#ifndef __OTA_HANDLER_H
#define __OTA_HANDLER_H

#include <stdint.h>
#include "app_config.h"                 // 需要 OTA 公共定义 (帧结构、地址常量等)

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== OTA 阶段 ======================== */
extern OTAStage g_ota_stage;            // 当前 OTA 阶段 (IDLE/RECEIVING/VERIFYING), 供外部观察

/* ======================== 公开 API ======================== */

/**
 * @brief 将 W5500 收到的原始数据送入环形缓冲区
 * @param data 刚 recv 出来的一段字节
 * @param len  字节数
 * 说明: 内部会关中断保护入环, 并更新"最近收数据时间"用于超时检测。
 */
void OTA_OnData(uint8_t *data, uint16_t len);

/**
 * @brief 尝试从环形缓冲区解析一帧并处理 (粘包解帧)
 * @return 1=已消费(解出一帧或丢弃了垃圾字节), 0=数据不足待收
 * 说明: 调用方通常写成 while(ota_try_parse_frame()) {} 把已到的整帧全部处理掉。
 */
int ota_try_parse_frame(void);

/**
 * @brief 中止当前 OTA 过程 (重置所有状态)
 * 说明: 连接断开 / 收包超时 / 出错 / 主动取消时都会调用。
 */
void OTA_Abort(void);

/**
 * @brief 后台擦除 tick (每次主循环调用)
 * 说明: 推进"每轮只发一条擦除命令/查一次完成"的非阻塞擦除状态机。
 */
void OTA_Tick(void);

/**
 * @brief 轮询 Socket 4 状态 (TCP 服务端)
 * 说明: 按 socket 状态 (CLOSED/LISTEN/ESTABLISHED/...) 做接收/断开/关闭处理。
 */
void OTA_PollSocket4(void);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_HANDLER_H */
