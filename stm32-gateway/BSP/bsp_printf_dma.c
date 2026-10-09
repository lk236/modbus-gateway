/*
 * bsp_printf_dma.c — printf 输出改 DMA 环形缓冲 (USART1)
 * ============================================================================
 * 【要解决的问题】
 *   最朴素的 printf 实现是重写 fputc(), 在里面调用 HAL_UART_Transmit 阻塞发送
 *   一个字符。115200 波特率下, 1 字节 10 位(起始+8数据+停止) → 每字符约 87µs。
 *   短打印无所谓, 但本项目有长达 500+ 字符的调试打印(如 OTA 的 [DBG_DUMP]),
 *   一次就是 500 × 87µs ≈ 43ms。调用 printf 的任务会被卡住 43ms ——
 *   对于 60s 才跑一次的任务没所谓, 但对 Modbus 响应/看门狗心跳这种实时性任务,
 *   几十毫秒的阻塞会造成响应超时甚至被判定"任务死了"。
 *
 * 【解决思路: 生产者-消费者 + DMA】
 *   printf 只负责把字符塞进一个 256 字节的环形缓冲(关中断, 微秒级完成)就立刻返回,
 *   任务继续往下跑; 真正把字节送到串口的工作交给 DMA 在后台慢慢做。
 *   角色划分:
 *     生产者 = 任意调用 printf 的任务 (写 s_wr 游标)
 *     消费者 = DMA + 发送完成中断 (推进 s_rd 游标)
 *
 * 【顺带解决的两个问题】
 *   1) 多任务互不交错: 每个字符的"写入 + 游标推进"是原子操作(关中断保护),
 *      所以两行不同任务的日志不会互相插字符, 最多是整行前后拼接。
 *   2) 有界背压: 环形满了就等 DMA 腾出空间, 最多等"发完 256 字节"的时间(~22ms),
 *      不会无限阻塞。
 * ============================================================================
 */
#include "bsp_printf_dma.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* 环形缓冲大小。必须是 2 的幂, 这样可以用 & (size-1) 代替 % size 取模(快很多) */
#define PR_RING_SIZE   256

static uint8_t  s_ring[PR_RING_SIZE];     /* 环形缓冲区本体 */
static volatile uint16_t s_rd;            /* 读游标 = 未发送区起点 (TC 中断后推进) */
static volatile uint16_t s_wr;            /* 写游标 = 生产者下一个要写的位置 */
static volatile uint8_t  s_tx_busy;       /* DMA 是否正有一笔在途传输 (1=在途) */
static uint16_t s_tx_len;                 /* 在途长度: 这一笔 DMA 发了多少字节(回调里靠它推进 rd) */

/* USART1 的发送 DMA 句柄 (全局, 中断处理里要用) */
DMA_HandleTypeDef hdma_usart1_tx;

/* ---------------------------------------------------------------------------
 * 初始化: 配置 DMA1_Channel4 做 USART1 发送
 * --------------------------------------------------------------------------- */
void BSP_PrintfDma_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();                          /* 开 DMA1 时钟 */

    hdma_usart1_tx.Instance = DMA1_Channel4;   /* DMA1_Ch4 = USART1_TX */
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH; /* 方向: 内存→外设(发送) */
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;     /* 外设地址固定(USART1->DR) */
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;         /* 内存地址递增 */
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    /* NORMAL 模式: 搬完就停(不是 CIRCULAR)。因为每次发送的长度不同,
     * 需要每次重新调用 HAL_UART_Transmit_DMA 指定新长度, 循环模式反而不好用 */
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;      /* 打印不着急, 给低优先级 */
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);       /* 挂到串口句柄 */

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* F1 HAL 的 DMA-TX 完成链是两段式: DMA TC 只置 USART_CR1_TCIE,
     * 必须由 USART1 的 TC 中断 (HAL_UART_IRQHandler) 收尾才会调用
     * HAL_UART_TxCpltCallback → 续发下一段。不使能 USART1 中断则
     * gState 卡在 BUSY_TX, 第一个分片发出后输出永远停住。
     *
     * 通俗说: DMA 只知道"数据搬完了", 但最后一个字节还在串口移位寄存器里没发出去。
     * STM32F1 的 HAL 用"串口 TC 中断"来确认真正的发送完毕。所以两个中断都要开。 */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* 状态清零 */
    s_rd = 0;
    s_wr = 0;
    s_tx_busy = 0;
    s_tx_len = 0;
}

/* ---------------------------------------------------------------------------
 * 从环形缓冲发出下一段连续数据
 *
 * 【调用约定】必须在关中断的临界区里调用 —— 因为它会读 s_wr/s_rd 并改 s_tx_busy,
 * 这些变量会被发送完成中断改动, 不保护会产生竞争。
 *
 * 关键点: 为什么 chunk 不能跨"绕回点"?
 *   DMA 是异步搬运内存的, 它按"起始地址 + 长度"线性读取。
 *   如果让 DMA 从 s_ring[250] 开始连读 20 字节, 它会读到 s_ring[269] ——
 *   越界了! 环形缓冲的"绕回"只是逻辑上的, 物理内存是连续的。
 *   所以每次最多发"从 s_rd 到缓冲区末尾"这一段, 下一段等回调里再发。
 * --------------------------------------------------------------------------- */
static void PrintfDma_StartNext(void)
{
    if (s_tx_busy) return;                       /* 已有一笔在途, 不能重叠启动 */

    /* 计算"待发送字节数": 用无符号差值再取模, 天然处理游标绕回。
     * 例: wr=10, rd=250 → (10-250) & 255 = (65536+10-250)&255 = 20 → 正确 */
    uint16_t avail = (s_wr - s_rd) & (PR_RING_SIZE - 1);
    if (avail == 0) return;                      /* 没数据要发 */

    uint16_t chunk = avail;
    if (chunk > (PR_RING_SIZE - s_rd)) {
        chunk = PR_RING_SIZE - s_rd;   /* 不跨绕回, 保证 DMA 读的是稳定数据 */
    }

    s_tx_len = chunk;                            /* 记下本笔长度, 回调里靠它推进 rd */
    s_tx_busy = 1;                               /* 标记在途 */

    if (HAL_UART_Transmit_DMA(&huart1, &s_ring[s_rd], chunk) != HAL_OK) {
        /* 理论不发生; 万一失败 → 丢弃未发送数据, 恢复可用 */
        s_tx_busy = 0;
        s_rd = s_wr;                             /* 游标追平 → 缓冲视为空, 避免卡死 */
    }
}

/* ---------------------------------------------------------------------------
 * 把 1 个字符写进环形缓冲 (fputc 的重定向目标, printf 最终会调到这里)
 *
 * 【临界区处理】先保存 PRIMASK(中断屏蔽状态) 再关中断, 退出时恢复原值。
 *   为什么不用普通的"关中断→开中断"? 因为本函数可能从"已经关中断的上下文"
 *   被调用(虽然是罕见情况)。如果无条件开中断, 就会破坏外层的临界区。
 *   保存-恢复的写法叫"嵌套安全的临界区", 是 RTOS 编程的规范做法。
 *
 * 【背压】环形满了(留一格用于区分满/空)就释放中断、osDelay(1) 让 DMA 有机会推进,
 *   然后再关中断重试。最坏等待 = 发完 256 字节的时间(约 22ms), 是有界的。
 *   判断满: (wr + 1) & mask == rd —— 故意留 1 格不用, 用来区分"满"和"空"
 *   (否则 wr==rd 既可能是空也可能是满, 无法区分)。
 *
 * 返回值: 返回写入的字符 (fputc 的约定: 成功返回写入的字符)
 * --------------------------------------------------------------------------- */
int BSP_PrintfDma_PutChar(int ch)
{
    uint32_t primask = __get_PRIMASK();   /* 保存当前中断屏蔽状态 */
    __disable_irq();                      /* 关中断, 进入临界区 */

    /* 环形满 (含在途数据) → 等 DMA 推进 (有界背压, 最坏 = 256 字节发送时间) */
    while (((s_wr + 1) & (PR_RING_SIZE - 1)) == s_rd) {
        __set_PRIMASK(primask);           /* 先恢复中断, 否则 osDelay 会死锁(需 SysTick 中断) */
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            osDelay(1);                   /* 让出 CPU, 等 DMA/中断推进 rd */
        }
        /* 调度器未启动 (理论不发生: 首次 printf 在调度器启动后) → 空转 */
        primask = __get_PRIMASK();        /* 重新保存 */
        __disable_irq();                  /* 重新进临界区, 再判断一次是否仍有空间 */
    }

    /* 写入字符并推进写游标。游标用 & mask 取模实现绕回 */
    s_ring[s_wr] = (uint8_t)ch;
    s_wr = (s_wr + 1) & (PR_RING_SIZE - 1);

    /* 如果 DMA 空闲, 立刻启动新的一笔(否则等正在途的那笔发完自动续发) */
    if (!s_tx_busy) {
        PrintfDma_StartNext();
    }

    __set_PRIMASK(primask);               /* 恢复原中断状态, 退出临界区 */
    return ch;                            /* fputc 约定: 返回写入的字符 */
}

/* ---------------------------------------------------------------------------
 * DMA 一段发完 → 续发下一段
 *
 * 调用链 (F1 平台):
 *   DMA1_Channel4_IRQHandler → HAL_DMA_IRQHandler → 置 USART_CR1_TCIE
 *     → USART1_IRQHandler → HAL_UART_IRQHandler → 见 TC 标志
 *     → HAL_UART_TxCpltCallback (就是本函数)
 *
 * 本函数做的三件事:
 *   1) 推进读游标: rd += 刚发出去的长度 (说明这些字节已经上线了, 空间可回收)
 *   2) 清在途标志
 *   3) 立刻尝试续发下一段(如果生产者在等待期间又写了数据)
 *
 * 注意本函数运行在中断上下文里, 所以 PrintfDma_StartNext 里不做关中断——
 * 中断里本来就不会被同优先级中断打断(且 StartNext 不涉及与任务的共享竞争)。
 * --------------------------------------------------------------------------- */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        s_rd = (s_rd + s_tx_len) & (PR_RING_SIZE - 1);   /* 1) 归还已发送的空间 */
        s_tx_len = 0;                                    /* 清长度 */
        s_tx_busy = 0;                                   /* 2) 清在途标志 */
        PrintfDma_StartNext();                           /* 3) 有存货就继续发 */
    }
}
