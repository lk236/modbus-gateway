/* ============================================================================
 * ota.h — OTA 固件下发模块接口
 *
 * 通过 TCP 506 端口向目标网关下发 .ota 固件文件。
 * 支持 BUSY 重试 (不计重试次数)、30s BUSY 总超时、
 * FINISH 3×6s 重试、CANCEL 1s×2 重试。
 * ============================================================================
 */
#ifndef __OTA_H
#define __OTA_H

#include "ota_protocol_def.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* OTA 上下文 */
typedef struct {
    int         gateway_id;
    char        host[64];
    int         port;               /* OTA 端口 (默认 506) */

    /* 固件信息 */
    char        fw_path[256];
    uint32_t    fw_size;            /* 纯固件大小 (不含 Header) */
    uint32_t    fw_crc32;
    uint32_t    fw_version;

    /* 进度 */
    OTAStatus   status;
    int         block_total;
    int         block_sent;
    int         progress_pct;
    uint32_t    last_activity;
    char        err_msg[128];
} OTAContext;

/**
 * @brief 异步启动 OTA 升级 (独立线程)
 * @param gateway_id 目标网关 ID
 * @param fw_path    .ota 固件文件路径
 * @return 0=成功, -1=已有 OTA 进行中, -2=文件错误
 */
int OTA_StartAsync(int gateway_id, const char *fw_path);

/**
 * @brief 获取 OTA 状态
 * @param gateway_id 目标网关 ID
 * @param out        输出状态
 * @return 0=有数据, -1=无此网关的 OTA
 */
int OTA_GetStatus(int gateway_id, OTAContext *out);

/**
 * @brief 取消进行中的 OTA
 * @param gateway_id 目标网关 ID
 * @return 0=已取消, -1=无进行中的 OTA
 */
int OTA_Cancel(int gateway_id);

#ifdef __cplusplus
}
#endif

#endif /* __OTA_H */
