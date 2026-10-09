/* ============================================================================
 * hw_init.c — 硬件初始化实现 (StdPerip 标准库)
 *
 * 【这个文件负责什么】
 *   给 Bootloader 建立"最小可运行环境": 系统时钟、串口(printf 输出)、
 *   GPIO、SPI1、独立看门狗, 以及一个毫秒时基 (g_tick) 和简易延时。
 *
 * 【时钟链路 ("72MHz 怎么来的")]
 *   HSE 8MHz 晶振 → PLL 倍频 x9 → SYSCLK 72MHz
 *                  ├─ AHB(HCLK)  = SYSCLK/1 = 72MHz
 *                  ├─ APB2(PCLK2) = HCLK/1  = 72MHz  (GPIO/SPI1/USART1 挂在这)
 *                  └─ APB1(PCLK1) = HCLK/2  = 36MHz  (最高只能 36MHz!)
 *   另外: Flash 在 48MHz<f≤72MHz 时必须设 2 个等待周期, 并开预取缓冲,
 *        否则 CPU 取指速度跟不上主频, 会跑飞。
 *
 * 【无 RTOS 的中断栈注意点】
 *   Bootloader 非常小, 跑的是裸机 (无 FreeRTOS)。SysTick 中断只做 g_tick++,
 *   是唯一定时来源。注意 delay_ms 里"每 10ms 喂狗": 因为 IWDG 使能后无法关闭。
 *
 * 编译: Keil MDK-ARM, ARMCC V5, -O1 优化 (Keil Optim=1, 见 Bootloader.uvprojx:314)
 * MCU:  STM32F103 (72MHz)
 * ============================================================================
 */
#include "hw_init.h"                    // 本模块声明 (含 g_tick、各 Init 原型)
#include "../Hardware/MySPI.h"          // SPI1 底层收发 (MySPI_Init / MySPI_SwapByte)
#include "app_config.h"                 // 板级配置 (LED/SPI 引脚宏等)
#include <stdio.h>                      // printf / FILE 相关

/* 禁用半主机模式 — printf 走物理串口而非调试器
   半主机(semihosting)是 ARM 调试器的一种 I/O 机制: 不写这行时, printf 会通过
   BKPT 指令把字符"发给调试器"显示在 Keil 的 Console。但产品脱机运行时没有调试器,
   会卡死在 BKPT 上。加 __use_no_semihosting 后, 链接器改用我们重写的 fputc
   (见下方), 字符就真的从 USART1 发出去了。 */
#pragma import(__use_no_semihosting)

/* ======================== 全局变量 ======================== */
volatile uint32_t g_tick = 0;   /* 毫秒计数器: SysTick 每 1ms +1。volatile 防止编译器优化掉 */

/* ======================== SysTick ======================== */
/* SysTick 中断服务函数 — 每 1ms 被硬件调用一次。
   名字固定, 由启动文件里的向量表指向这里 (不能改名)。
   里面只做最简单的自增, 保证中断尽快返回 (中断里忌做耗时/阻塞操作)。 */
void SysTick_Handler(void)
{
    g_tick++;                   // 时基 +1ms
}

/* 取当前毫秒时基。返回值是无符号, "g_tick - start" 的减法天然能抗回绕
   (即使 g_tick 溢出回 0, 无符号减法结果仍然正确)。 */
uint32_t GetTick(void)
{
    return g_tick;
}

/* ======================== 简易延时 ======================== */
/* 忙等延时 N 毫秒。注意: 因为 IWDG 一旦开启无法关闭, 这里"每 10ms 喂一次狗",
   否则延时超过看门狗超时(约 27.3s)就会复位 —— 不过通常延时都很短。 */
void delay_ms(uint32_t ms)
{
    uint32_t start = g_tick;                    // 记下起始时刻
    while ((g_tick - start) < ms) {             // 无符号减法, 抗回绕
        /* 每 10ms 喂狗 — 避免长延时期间看门狗超时 */
        if ((g_tick - start) % 10 == 0) {
            IWDG_ReloadCounter();               // 重装计数器 (喂狗)
        }
    }
}

/* ======================== 时钟配置 (72MHz, StdPerip) ========================
 * HSE 8MHz → PLL x9 → SYSCLK 72MHz
 * APB1 = 36MHz, APB2 = 72MHz
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_DeInit();                               // 先把 RCC 复位到默认状态, 保证从已知状态开始

    /* 使能 HSE — 打开外部 8MHz 晶振 (比内部 HSI 精度高得多) */
    RCC_HSEConfig(RCC_HSE_ON);
    if (RCC_WaitForHSEStartUp() == SUCCESS)     // 等晶振起振稳定 (这是阻塞等待, 正常几十 ms 内)
    {
        /* FLASH 预取 + 2 等待周期 (72MHz 需要)
           —— 主频 >48MHz 时, Flash 取指必须插入等待周期, 并开启预取缓冲来弥补性能 */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);      // 2 个等待周期

        /* AHB = SYSCLK / 1 = 72MHz */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* APB2 = HCLK / 1 = 72MHz (GPIO/SPI1/USART1 所在总线, 可跑满 72MHz) */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* APB1 = HCLK / 2 = 36MHz (APB1 上限就是 36MHz, 超了会不稳) */
        RCC_PCLK1Config(RCC_HCLK_Div2);

        /* PLL: HSE 8MHz x9 = 72MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);    // 时钟源 HSE 不分频, 倍频 9
        RCC_PLLCmd(ENABLE);                     // 启动 PLL
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);    // 等 PLL 锁定就绪

        /* SYSCLK = PLL — 把系统时钟源从 HSI 切到 PLL */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        while (RCC_GetSYSCLKSource() != 0x08);  /* 0x08 = PLL — 确认切换成功 (读 SWS 位) */
    }

    SystemCoreClockUpdate();  /* 更新 SystemCoreClock = 72000000 — 供 SysTick_Config 计算使用 */
}

/* ======================== USART1 初始化 (StdPerip, 115200 8N1) ======================== */
/* 串口是 Bootloader 唯一的"日志出口": 上面所有 printf 都从这里发出去。 */
void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef gpio;                      // GPIO 配置结构体
    USART_InitTypeDef usart;                    // 串口配置结构体

    /* 开 USART1 和 GPIOA 的时钟 —— 用任何外设前必须先开它的时钟, 否则寄存器写不进去 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    /* PA9 = TX (推挽复用) — 复用推挽才能让 USART 硬件控制引脚电平 */
    GPIO_StructInit(&gpio);                     // 先把结构体填成默认值, 避免随机值
    gpio.GPIO_Pin   = GPIO_Pin_9;               // PA9
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;          // 复用推挽输出
    gpio.GPIO_Speed = GPIO_Speed_50MHz;         // 输出速度 50MHz (115200 波特率足够)
    GPIO_Init(GPIOA, &gpio);

    /* PA10 = RX (浮空输入) — 接收脚不用内部上下拉, 由外部/对方驱动 */
    gpio.GPIO_Pin   = GPIO_Pin_10;              // PA10
    gpio.GPIO_Mode  = GPIO_Mode_IN_FLOATING;    // 浮空输入
    GPIO_Init(GPIOA, &gpio);

    /* USART1: 115200, 8N1 */
    USART_StructInit(&usart);                   // 默认 8 数据位 / 无校验 / 1 停止位
    usart.USART_BaudRate = 115200;              // 波特率 115200
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;   // 收发都开 (实际只用 TX)
    USART_Init(USART1, &usart);
    USART_Cmd(USART1, ENABLE);                  // 使能 USART1
}

/* printf 重定向到 USART1 (物理串口)
   标准库的 printf 最终会调用 fputc() 逐字符输出。这里重写 fputc,
   把每个字符写进 USART1 的发送寄存器, 就实现了"printf → 串口"。
   __FILE / __stdout / _sys_exit 是配合 __use_no_semihosting 必须提供的符号。 */
struct __FILE { int handle; };                  // 标准库要求的最小 FILE 结构
FILE __stdout;

void _sys_exit(int x) { x = x; }                // 退出桩函数 (无操作系统, 不需要真正退出)

int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);        // 把字符塞进发送数据寄存器 (DR)
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);   // 等 TXE=1: 发送寄存器已空, 可收下一字节
    return ch;                                  // 返回字符本身 (标准约定)
}

/* ======================== GPIO 初始化 (StdPerip) ======================== */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    /* 依次开 GPIOA/B/C 时钟 (三个端口的引脚都会用到) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* W25Q64 CS = PA4 (推挽输出)
       CS 空闲时必须拉高, 否则芯片会被误选中, 干扰 SPI 总线上的其它设备。 */
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin   = GPIO_Pin_4;               // PA4
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;         // 推挽输出 (软件控制片选)
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    GPIO_SetBits(GPIOA, GPIO_Pin_4);            // 初始拉高 = 不选中

    /* W5500 CS = PB12 (推挽输出) */
    gpio.GPIO_Pin   = GPIO_Pin_12;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);           // 空闲拉高

    /* W5500 RST = PB1 (推挽输出) — 复位脚, 默认拉高(不复位) */
    gpio.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_1);

    /* LED PC13 (推挽输出, 低电平亮) — 蓝丸板 PC13 驱动的是 VCC 侧的 LED, 所以低电平点亮 */
    gpio.GPIO_Pin   = GPIO_Pin_13;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &gpio);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);   /* 灭 — 初始不点亮 */
}

/* ======================== SPI1 初始化 (通过 MySPI) ======================== */
/* 本工程 SPI 配置细节在 Hardware/MySPI.c 里 (模式0、MSB、分频128=约 562.5kHz)。 */
void MX_SPI1_Init(void)
{
    MySPI_Init();                               // 直接复用底层库的 SPI1 初始化
}

/* ======================== IWDG 初始化 (StdPerip) ======================== */
/* 独立看门狗: 由 40kHz 内部 LSI 驱动, 一旦使能就无法用软件关闭。
   超时 ≈ 256(预分频) * 4095(重装值) / 40000Hz ≈ 26.2s (手册标称最大约 27.3s)。
   工程含义: 任何超过约 26s 的阻塞都必须插 IWDG_ReloadCounter() 喂狗。 */
void MX_IWDG_Init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);   // 解除 PR/RLR 寄存器的写保护 (必须先做)
    IWDG_SetPrescaler(IWDG_Prescaler_256);          // 预分频 256 (把 40kHz LSI 降到约 156Hz)
    IWDG_SetReload(4095);                           // 重装值 4095 (最大, 尽量拉长超时)
    IWDG_ReloadCounter();                           // 先喂一次, 把计数器装载好
    IWDG_Enable();                                  // 使能看门狗 —— 从此刻起无法关闭!
}
