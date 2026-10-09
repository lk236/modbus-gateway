/* ============================================================================
 * boot_jump.h — Bootloader 跳转与故障态接口
 *
 * 【两个核心函数】
 *   JumpToApp()               : 把 CPU "干净地交给" App 运行 (改 MSP/VTOR 后跳 PC)
 *   Bootloader_EnterFailsafe(): App 不可用/升级反复失败时, 开网络死循环等新固件 (永不返回)
 *
 * 【跳转为什么不是一句函数调用】
 *   Cortex-M 启动靠向量表前两项 [初始 MSP, 复位入口]。App 链接在 0x08004000,
 *   所以要: 读它的向量表 → 设 MSP → 把 VTOR 指向 0x08004000 (否则中断会跑回
 *   Bootloader) → 跳到入口。详见 boot_jump.c 的注释。
 * ============================================================================
 */
#ifndef __BOOT_JUMP_H
#define __BOOT_JUMP_H

#include <stdint.h>
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 跳转到应用 (校验向量表 → 关中断 → 反初始化 → 跳转)
 * @param app_addr 应用起始地址 (如 0x08004000)
 * 说明: 若 App 向量表校验不通过, 会转入故障态而不是跳过去。
 */
void JumpToApp(uint32_t app_addr);

/**
 * @brief 进入故障态 — 初始化网络, 进入 OTA 接收循环 (永不返回)
 * 说明: 在端口 506 开 TCP 服务端, 等上位机重新推送固件。
 */
void Bootloader_EnterFailsafe(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOT_JUMP_H */
