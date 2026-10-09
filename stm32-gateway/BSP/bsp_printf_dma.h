#ifndef __BSP_PRINTF_DMA_H
#define __BSP_PRINTF_DMA_H

#include "usart.h"

/* 初始化 printf DMA 通道 (DMA1_Ch4 = USART1_TX), 在首次 printf 前调用 */
void BSP_PrintfDma_Init(void);
/* 向串口环形缓冲写 1 字节 (fputc 调用), 有界背压: 缓冲满则等 DMA 推进 */
int BSP_PrintfDma_PutChar(int ch);

extern DMA_HandleTypeDef hdma_usart1_tx;

#endif /* __BSP_PRINTF_DMA_H */
