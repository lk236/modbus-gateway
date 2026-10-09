/*
 * bsp_rs485.c — RS-485 半双工串口驱动 (DMA + IDLE 帧捕获版)
 * ============================================================================
 * 【RS-485 是什么】
 *   RS-485 是工业现场最常用的"总线式"串口物理层。和普通串口(TTL)最大的区别:
 *     1) 差分信号: 用 A/B 两根线的"电压差"表示 0/1, 抗干扰能力强, 能拉几百米
 *        (TTL 串口几厘米就乱了), 适合车间/机房这种电磁环境差的场合。
 *     2) 多点总线: 一条总线上可以挂 32 个设备, 靠"从机地址"区分(Modbus 就是这么干的)。
 *     3) 半双工: 收发共用同一对差分线, 同一时刻只能"要么发、要么收",
 *        所以必须有方向控制引脚。
 *
 * 【硬件连接】
 *   USART2: TX=PA2, RX=PA3 (复用为串口功能)
 *   PA1 = DE/RE 方向控制: 高电平=发送(Driver Enable), 低电平=接收(Receiver Enable)
 *   波特率 9600, 8 数据位, 无校验, 1 停止位 (8N1)
 *
 * 【接收方案的演进 (本文件的核心)】
 *   老方案: 每收 1 字节进一次中断 → 存进缓冲 → 再靠"400ms 无新字节"来判断一帧结束。
 *           缺点: ① 每字节一次中断, 9600 波特率下每 1ms 就中断一次, CPU 开销大;
 *                 ② 靠超时判帧尾, 白白多等 200~400ms 才敢解析, 响应慢。
 *   新方案: DMA1_Ch6 以"循环模式"把收到的字节直接搬进内存(不占 CPU),
 *           再用 USART 的 IDLE 中断来判帧:
 *           "总线连续空闲一个字符时间以上" = 这一帧发完了。
 *           优点: ① CPU 几乎不参与接收; ② 帧一到就能立刻解析, 没有帧尾等待。
 *
 * 【IDLE 中断是什么】
 *   串口线上没有数据时是持续的高电平(空闲态)。IDLE 中断的触发条件是:
 *   "接收线从有数据变回空闲, 并且保持了一个完整字符的时间"。
 *   所以它天然就是"一帧结束"的信号, 而且硬件自动判断, 比软件超时精确得多。
 * ============================================================================
 */
#include "bsp_rs485.h"
#include "usart.h"
#include <stdio.h>

/* ---- DMA + IDLE 帧捕获的运行时状态 ---- */
static uint8_t   s_frame_buf[RS485_FRAME_BUF_SIZE];  /* DMA 目标缓冲, 同时也是解析源 */
static volatile uint16_t s_frame_len;                /* 当前帧字节数 (IDLE 中断里算出来) */
static volatile uint8_t  s_frame_ready;              /* 帧就绪标志: IDLE 置 1, 重新武装时清 0 */

/* USART2 的接收 DMA 句柄。定义为全局, 是因为 HAL 的 USART2_IRQHandler 里要用到 */
DMA_HandleTypeDef hdma_usart2_rx;

/* 485 总线互斥锁: 保证同一时刻只有一笔"发→收"事务在总线上进行 */
static osMutexId_t g_rs485_mutex;

/* 串口错误统计 (供调试/告警使用, volatile 因为中断里会改) */
volatile uint32_t g_uart_error_count;
volatile uint32_t g_uart_last_error;

/* ---------------------------------------------------------------------------
 * 重新武装 DMA 帧捕获 (为一笔新事务做准备)
 *
 * 每次发完请求要收响应前, 都必须调用它把状态清干净, 否则会拿到上一帧的残渣。
 *
 * 步骤:
 *   1) 清掉帧长度/就绪标志
 *   2) 如果 DMA 还在忙, 先 Abort 掉(不然无法重新配置)
 *   3) 清 USART 的残留标志位 —— 这一步很关键
 *   4) 重新开接收使能位 RE
 *   5) 重新启动 DMA 接收
 *
 * 【第 3 步的 "读 SR 再读 DR" 是什么玄机?】
 *   STM32 的 USART 有几个标志位(RXNE/ORE/IDLE)是"读 SR 后读 DR 才清"的硬件约定,
 *   光读 SR 不清, 光写 DR 也不清。这是老式外设常见的"清标志靠读序列"的设计。
 *   ORE(Overrun Error, 溢出错误)尤其麻烦: 一旦置位且不清, 后续接收全废,
 *   所以这里必须干净地清掉。
 * --------------------------------------------------------------------------- */
void BSP_RS485_StartRxCapture(void)
{
    s_frame_len = 0;                        /* 帧长清零 */
    s_frame_ready = 0;                      /* 就绪标志清零 */

    /* DMA 若还在搬运, 先强制终止, 否则没法重新配置 */
    if (hdma_usart2_rx.State == HAL_DMA_STATE_BUSY) {
        HAL_DMA_Abort(&hdma_usart2_rx);
    }

    /* 清 USART 残留标志 (约定: 读 SR 再读 DR 才能清) */
    if (USART2->SR & USART_SR_IDLE) { (void)USART2->SR; (void)USART2->DR; }  /* 清 IDLE */
    if (USART2->SR & USART_SR_RXNE) { (void)USART2->DR; }                    /* 清 RXNE(读 DR 即可) */
    if (USART2->SR & USART_SR_ORE)  { (void)USART2->DR; }                    /* 清 ORE */

    SET_BIT(USART2->CR1, USART_CR1_RE);     /* 确保接收使能 (CR1 的 RE 位) */

    /* HAL 库内部状态要手动置 READY, 否则 HAL_UART_Receive_DMA 会返回 BUSY 拒绝启动 */
    huart2.RxState = HAL_UART_STATE_READY;
    /* 启动 DMA 接收: 长度给整个缓冲区大小(循环模式会一直转), 实际长度事后由计数器反推 */
    HAL_UART_Receive_DMA(&huart2, s_frame_buf, RS485_FRAME_BUF_SIZE);
}

/* ---------------------------------------------------------------------------
 * IDLE 中断处理 (由 USART2_IRQHandler 调用)
 *
 * 触发时机: 总线从"有数据"回到"持续空闲" → 说明一帧完整收到了。
 *
 * 【帧长怎么算出来的? 这是本方案最巧妙的一步】
 *   DMA 有一个"剩余传输计数器": 启动时等于缓冲区大小, 每搬 1 字节就减 1。
 *   所以 "已经收到的字节数 = 缓冲区大小 - 剩余计数"。
 *   不用自己维护长度变量, 硬件帮我们数好了。
 *
 * 【为什么要判断 len > 0?】
 *   一次"请求→响应"事务中, 总线上会出现两次 IDLE:
 *     ① 我们发完请求后, 总线静默(等从机反应) → 触发一次 IDLE, 但此时还没收到任何字节
 *     ② 从机发完响应后, 总线又静默 → 触发第二次 IDLE, 这次才是真正的"响应帧结束"
 *   靠 len > 0 把第 ① 次忽略掉, 只认真正收到数据后的那次。
 * --------------------------------------------------------------------------- */
void BSP_RS485_OnIdleISR(void)
{
    /* 读 DMA 剩余计数, 反推已接收字节数 */
    uint16_t cnt = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    uint16_t len = RS485_FRAME_BUF_SIZE - cnt;

    /* len==0: 请求→响应间隙的空闲期 IDLE (总线静默无数据), 忽略
     * 只有真正收到字节后的 IDLE 才代表一帧结束 */
    if (len > 0) {
        s_frame_len = len;          /* 记录帧长 */
        s_frame_ready = 1;          /* 通知上层: 有完整一帧了 */
    }
}

/* 查询标志 / 取帧长 / 取帧缓冲首地址 (给上层解析用, 三个都是"零开销"的读取) */
uint8_t BSP_RS485_FrameReady(void) { return s_frame_ready; }
uint16_t BSP_RS485_FrameLen(void)  { return s_frame_len; }
const uint8_t *BSP_RS485_FramePtr(void) { return s_frame_buf; }

/* ---------------------------------------------------------------------------
 * 阻塞等待一帧完整到达
 *
 * 用途: 健康检查任务这种"发完请求就等着要结果"的阻塞式路径。
 * 原理: 轮询 s_frame_ready 标志位, 每 1ms 检查一次, 超时就返回。
 *
 * 为什么用 osDelay(1) 而不是空转? 空转会独占 CPU, 让低优先级任务饿死;
 * osDelay 会让出 CPU, 其它任务还能跑。
 *
 * 返回值: RS485_OK=收到帧; RS485_TIMEOUT=超时(总线故障/从机没回)
 * --------------------------------------------------------------------------- */
RS485_Status BSP_RS485_WaitFrame(uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();   /* 记录起始时刻 */
    while (!s_frame_ready) {                   /* 标志没置位就一直等 */
        if ((osKernelGetTickCount() - start) >= timeout_ms) {   /* 用 tick 差值算超时 */
            return RS485_TIMEOUT;
        }
        osDelay(1);                            /* 让出 1ms CPU */
    }
    return RS485_OK;
}

/* ---------------------------------------------------------------------------
 * RS-485 驱动初始化
 *
 * 做五件事:
 *   1) 先把方向引脚置成"接收"(上电默认不能占着总线)
 *   2) 创建互斥锁
 *   3) 配置 DMA1_Channel6 接收
 *   4) 配好中断优先级并使能 USART2 / DMA 中断
 *   5) 启动第一次接收捕获 + 打开 IDLE 中断
 * --------------------------------------------------------------------------- */
void BSP_RS485_Init(void)
{
    RS485_RX_MODE();                        /* 1) 默认接收态: PA1 拉低 */

    /* 2) 创建互斥锁, 优先级继承防优先级反转 */
    osMutexAttr_t mutex_attr = { NULL, osMutexPrioInherit, NULL, 0 };
    g_rs485_mutex = osMutexNew(&mutex_attr);

    /* 3) 配置 DMA1_Ch6 = USART2_RX, 循环模式 (每次事务从缓冲起点重新开始)
     * 注意: F1 硅片上 USART2_RX 的 DMA 请求线固定在 Ch6 (Ch5 是 USART1_RX 的),
     * 这是硬件连线决定的, 不能随便选通道 */
    __HAL_RCC_DMA1_CLK_ENABLE();                          /* 先开 DMA1 时钟 */
    hdma_usart2_rx.Instance = DMA1_Channel6;              /* 通道 6 */
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY; /* 方向: 外设→内存(接收) */
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;     /* 外设地址固定(永远是 USART2->DR) */
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;         /* 内存地址递增(依次存进缓冲) */
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设按字节 */
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;     /* 内存按字节 */
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;              /* 循环模式: 搬满自动回到开头 */
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;     /* 高优先级: 串口数据不能丢 */
    HAL_DMA_Init(&hdma_usart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);       /* 把 DMA 句柄挂到串口句柄上 */

    /* 4) 配置中断。抢占优先级 6 / 子优先级 0。
     *    (数值越小优先级越高; 6 是给外设中断的常用档位, 低于 FreeRTOS 的
     *     PendSV/SysTick, 高于普通任务) */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    /* 5) 先武装好接收, 再打开 IDLE 中断(顺序不能反, 否则可能丢第一帧) */
    BSP_RS485_StartRxCapture();
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);          /* 使能 IDLE 中断 */
}

/* ---------------------------------------------------------------------------
 * 【阻塞式】RS-485 发送一帧
 *
 * 半双工总线的"发"必须做完整的"切换→发送→切回"三步曲, 中间任何一步漏掉
 * 都会导致总线被占死或收到自己发的回显。
 *
 * 流程详解:
 *   1) 拿互斥锁 —— 防止两个任务同时抢总线
 *   2) 停掉接收 DMA —— 因为接下来要发, 不能同时收(会收到自己的回声)
 *   3) 清残留标志
 *   4) 切到发送态 (PA1 拉高, 485 收发器把差分线接到发送器上)
 *   5) 【关键】关掉接收使能 RE 和接收中断 —— 半双工下一边发一边收,
 *      STM32 会把刚发出去的字节又从 RX 收回来, 产生"回显"。
 *      关掉 RE 从硬件上杜绝回显。
 *   6) HAL_UART_Transmit 发送(阻塞式, 超时 200ms)
 *   7) 【关键】等 TC(Transmission Complete) 标志 —— 注意不是等 HAL 函数返回!
 *      HAL 返回只说明"数据都写进发送寄存器了", 最后 1 个字节可能还在移位输出。
 *      如果这时就切回接收态, 最后一个字节会被截断。
 *   8) 切回接收态, 空转一小会儿等收发器芯片切换稳定(硬件切换需要时间)
 *   9) 清标志 + 重新武装 DMA, 准备接收从机的响应
 *   10) 放锁
 *
 * 返回值: RS485_OK / RS485_ERROR
 * --------------------------------------------------------------------------- */
RS485_Status BSP_RS485_Send(uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0) {
        return RS485_ERROR;                 /* 参数检查 */
    }

    /* 1) 拿锁, 保证总线独占 */
    if (g_rs485_mutex != NULL) {
        osMutexAcquire(g_rs485_mutex, osWaitForever);
    }

    /* 2) 终止正在进行的 DMA 接收 */
    HAL_UART_AbortReceive(&huart2);

    /* 3) 清残留标志 (防止上一帧的残渣被当成这一帧的响应) */
    if (USART2->SR & USART_SR_RXNE) { (void)USART2->DR; }
    if (USART2->SR & USART_SR_ORE)  { (void)USART2->DR; }

    /* 4) 切到发送方向 */
    RS485_TX_MODE();
    /* 5) 关接收使能 + 关接收中断, 杜绝自收自发的回显 */
    CLEAR_BIT(USART2->CR1, USART_CR1_RE | USART_CR1_RXNEIE);

    /* 6) 阻塞发送 */
    HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, buf, len, 200);
    if (ret != HAL_OK) {
        /* 发送失败时打印状态寄存器, 便于定位是超时还是硬件问题 */
        printf("[RS485] HAL_TX error=%d, SR=0x%04lX\r\n", ret, USART2->SR);
    }

    /* 7) 死等 TC 标志: 确保最后一个字节真正从线上移出去 (否则会截断) */
    while (!(USART2->SR & USART_SR_TC));

    /* 8) 切回接收方向 */
    RS485_RX_MODE();

    /* 空转约 2000 次(几百微秒), 给 485 收发器芯片方向切换留出建立时间。
     * 这段看着"浪费", 但对半双工总线是必须的: 收发器切换有 ns~µs 级延迟,
     * 立刻开收可能把总线切换的毛刺收成数据。 */
    for (volatile uint32_t i = 0; i < 2000; i++);

    /* 9) 再清一次标志(这段时间可能收进了毛刺), 然后重新武装 DMA 收响应 */
    if (USART2->SR & USART_SR_RXNE) { (void)USART2->DR; }
    if (USART2->SR & USART_SR_ORE)  { (void)USART2->DR; }

    /* 切回接收后重新武装 DMA 帧捕获 */
    BSP_RS485_StartRxCapture();

    /* 10) 放锁 */
    if (g_rs485_mutex != NULL) {
        osMutexRelease(g_rs485_mutex);
    }

    return (ret == HAL_OK) ? RS485_OK : RS485_ERROR;
}

/* ---------------------------------------------------------------------------
 * 串口错误回调 (由 HAL 在检测到 UART 错误时调用)
 *
 * 常见错误码 (huart->ErrorCode):
 *   HAL_UART_ERROR_ORE  溢出: 数据来了但没被及时取走, 后面的覆盖了前面的
 *   HAL_UART_ERROR_NE   噪声: 线上有干扰, 采样到的电平不干净
 *   HAL_UART_ERROR_FE   帧错误: 停止位不对(波特率不匹配/线接错)
 *   HAL_UART_ERROR_PE   校验错误
 *
 * 处理策略: 只统计不恢复。把 DMA 停掉、把 HAL 状态置回 READY, 交给下一次
 *   StartRxCapture 重新武装。这是"自愈式"设计: 出错不崩, 下一笔事务自动恢复。
 *
 * 注意: 这个函数名是 HAL 库规定的弱符号(weak), 我们在此重写它 ——
 *       整个工程里只能有一个 HAL_UART_ErrorCallback 定义。
 * --------------------------------------------------------------------------- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {            /* 只关心 485 用的 USART2 */
        g_uart_error_count++;                   /* 错误计数 +1 */
        g_uart_last_error = huart->ErrorCode;   /* 记下最后一次错误码 */

        /* 停掉 DMA, 交给下一次事务的 StartRxCapture 重新武装。
         * 这里直接改 State 而不是调 HAL_DMA_Abort, 是因为可能已经在中断上下文里,
         * 避免嵌套调用复杂逻辑。 */
        if (hdma_usart2_rx.State == HAL_DMA_STATE_BUSY) {
            __HAL_DMA_DISABLE(&hdma_usart2_rx);
            hdma_usart2_rx.State = HAL_DMA_STATE_READY;
        }
        huart2.RxState = HAL_UART_STATE_READY;  /* HAL 串口状态也置回可用 */
    }
}
