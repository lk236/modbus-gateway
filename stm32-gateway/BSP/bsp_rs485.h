#ifndef __BSP_RS485_H
#define __BSP_RS485_H

#include "main.h"
#include "cmsis_os.h"

#define RS485_DE_RE_PORT    GPIOA
#define RS485_DE_RE_PIN     GPIO_PIN_1

#define RS485_TX_MODE()     HAL_GPIO_WritePin(RS485_DE_RE_PORT, RS485_DE_RE_PIN, GPIO_PIN_SET)
#define RS485_RX_MODE()     HAL_GPIO_WritePin(RS485_DE_RE_PORT, RS485_DE_RE_PIN, GPIO_PIN_RESET)

/* DMA 帧捕获缓冲大小: Modbus RTU 最大帧 256 字节, 留 4 字节余量避免 DMA 绕回 */
#define RS485_FRAME_BUF_SIZE   260

/* RS-485 操作返回值 */
typedef enum {
    RS485_OK      = 0,
    RS485_ERROR   = 1,
    RS485_TIMEOUT = 2,
} RS485_Status;

void BSP_RS485_Init(void);
RS485_Status BSP_RS485_Send(uint8_t *buf, uint16_t len);

/* ---- DMA + IDLE 帧捕获接口 ---- */
/* 重新武装接收: 停 DMA、清帧标志、从缓冲起点重新捕获 (每次发送请求前/后调用) */
void BSP_RS485_StartRxCapture(void);
/* USART2 IRQ 中调用: IDLE 中断 = 一帧完整到达, 记录帧长 */
void BSP_RS485_OnIdleISR(void);
/* 帧是否已完整到达 (IDLE 置位, StartRxCapture 清零) */
uint8_t BSP_RS485_FrameReady(void);
/* 当前帧字节数 */
uint16_t BSP_RS485_FrameLen(void);
/* 当前帧数据指针 (有效到下一次 StartRxCapture) */
const uint8_t *BSP_RS485_FramePtr(void);
/* 阻塞等待一帧完整到达 (阻塞路径用, 内部 osDelay 轮询) */
RS485_Status BSP_RS485_WaitFrame(uint32_t timeout_ms);

extern DMA_HandleTypeDef hdma_usart2_rx;
extern volatile uint32_t g_uart_error_count;
extern volatile uint32_t g_uart_last_error;

#endif
