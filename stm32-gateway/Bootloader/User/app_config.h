/* ============================================================================
 * app_config.h — Bootloader 板级配置
 *
 * 【这个文件的两大作用】
 *   1) 把 "OTA 公共定义" 按正确顺序引进来 (必须先定义 OTA_FLASH_xxx 宏,
 *      再包含 ota_ctrl_block.h, 因为后者要用这些宏去读写 Flash)。
 *   2) 集中管理板级参数: 引脚、网络地址、缓冲区大小 —— 换硬件只改这里。
 *
 * 【外设与引脚总览 (联调时对照接线)】
 *   W25Q64 Flash : SPI1, CS = PA4
 *   W5500 以太网 : SPI1 (与 Flash 共用), CS = PB12, RST = PB1
 *   LED          : PC13 (低电平亮)
 *   调试串口      : USART1, PA9=TX, 115200 8N1
 *
 * 编译: Keil MDK-ARM, ARMCC V5, -O1 优化 (Keil Optim=1, 见 Bootloader.uvprojx:314)
 * 链接: 0x08000000, 最大 16KB
 * ============================================================================
 */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== OtaCommon 共享定义 ======================== */
/* 必须先定义 OTA_FLASH_xxx 宏, 再包含 ota_ctrl_block.h
   —— ota_ctrl_block.h 是 Bootloader 和 App 共用的"控制块读写"实现,
      它把"怎么读写 Flash"抽象成这几个宏, 由各工程自己指向具体的底层函数。
      这样一份逻辑可以同时跑在 Bootloader 和 App 里, 避免复制粘贴两份。 */
#include "stm32f10x.h"
#include <string.h>
#include "bsp_w25q64.h"                 // 本工程用 BSP_W25Q64_xxx 作为底层 Flash 操作

/* 把公共控制块的头文件里的抽象宏, 绑定到 W25Q64 驱动 */
#define OTA_FLASH_READ(addr, buf, len)       BSP_W25Q64_Read(addr, buf, len)      // 读
#define OTA_FLASH_WRITE(addr, buf, len)      BSP_W25Q64_Write(addr, (uint8_t*)(buf), len)  // 写
#define OTA_FLASH_SECTOR_ERASE(addr)         BSP_W25Q64_SectorErase(addr)         // 擦扇区
#define OTA_FLASH_IS_BUSY()                  BSP_W25Q64_IsBusy()                  // 查忙
#define OTA_FLASH_WAIT_BUSY()                do { volatile uint32_t _w = 500; while(_w--); } while(0)
/* WAIT_BUSY 是一段微延时 (空转 500 次), 用于两次查忙之间稍作等待 */

#include "ota_crc.h"                    // CRC32 / CRC16 查表实现
#include "ota_protocol_def.h"           // OTA 协议: 帧结构、命令码、地址常量
#include "ota_ctrl_block.h"             // OTA 控制块读写 (依赖上面几个 OTA_FLASH_xxx 宏)

/* ======================== LED 引脚 ======================== */
#define LED_PORT            GPIOC
#define LED_PIN             GPIO_PIN_13     // PC13, 低电平点亮

/* ======================== W25Q64 引脚 ======================== */
#define W25Q64_CS_PORT      GPIOA
#define W25Q64_CS_PIN       GPIO_PIN_4      // 片选 PA4 (SPI1)

/* ======================== W5500 引脚 ======================== */
#define W5500_CS_PORT       GPIOB
#define W5500_CS_PIN        GPIO_PIN_12     // 片选 PB12 (SPI1)
#define W5500_RST_PORT      GPIOB
#define W5500_RST_PIN       GPIO_PIN_1      // 复位 PB1

/* ======================== 网络参数 ======================== */
/* 故障态 Bootloader 会以此身份开 TCP 服务端 (端口 506) 等待上位机推固件。
   注意: 它是硬编码的, 实际部署如需改动, 这里和上位机 linux-host 配置要保持一致。 */
#define OTA_MAC             {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}
#define OTA_IP              {192, 168, 1, 100}
#define OTA_SUBNET          {255, 255, 255, 0}
#define OTA_GW              {192, 168, 1, 1}

/* ======================== OTA 环形缓冲区 ======================== */
/* 必须是 2 的幂 —— 因为环形缓冲用 &(SIZE-1) 代替取模实现绕回。
   2048 字节足够容纳"最大 TCP 分段 + 未解完的半包"。 */
#define OTA_RX_BUF_SIZE     2048

/* ======================== 擦除相关 ======================== */
#define W25Q64_SECTOR_SIZE  4096U           // W25Q64 扇区大小 4KB

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
