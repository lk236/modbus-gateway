/* ============================================================================
 * boot_jump.c — Bootloader 跳转与故障态实现
 *
 * 【本文件解决两个问题】
 *   1) JumpToApp(): 怎么把 CPU "干净地交给"另一个程序 (App) 去跑?
 *      —— 这不是一句函数调用! 要复位外设、改栈指针、改向量表基址, 最后才跳 PC。
 *   2) Bootloader_EnterFailsafe(): 当 App 不可用/升级反复失败时, 怎么"自救"?
 *      —— 初始化网络, 开一个 TCP 服务端, 死循环等上位机重新推固件。
 *
 * 【为什么跳转要这么处理】
 *   CPU 复位后靠两个寄存器启动: MSP(主栈指针) 和 PC(程序计数器)。
 *   Cortex-M 的中断向量表前两项正是 [初始 MSP 值, 复位入口地址]。
 *   App 被编译链接到 0x08004000, 它的向量表就在 0x08004000。
 *   所以跳转的本质是: 模拟一次"从向量表启动" ——
 *     读 App 的 [0]=栈顶, [4]=入口 → 设 MSP → 设 VTOR 指向 App 向量表 → 跳到入口。
 *   如果不重设 VTOR, App 里发生中断时会跑到 Bootloader 的向量表 (地址 0x08000000),
 *   完全错位, 必崩。
 * ============================================================================
 */
#include "boot_jump.h"
#include "hw_init.h"        /* MX_GPIO_Init / MX_SPI1_Init / delay_ms / IWDG_ReloadCounter */
#include "bsp_w5500.h"      /* 故障态需要 W5500 网络 */
#include "ota_handler.h"    /* OTA_Tick / OTA_PollSocket4 / OTA_Abort */
#include "core_cm3.h"       /* CMSIS 内核函数: __disable_irq / __set_MSP / SCB 等 */
#include "stm32f10x.h"                  // Device header

/* ======================== 跳转到应用 ======================== */

void JumpToApp(uint32_t app_addr)
{
    /* ---- 阶段 1: 校验 App 有效性 (此时中断仍在开, 便于失败时进故障态) ----
       App 的向量表前 8 字节: [0]=初始 MSP(栈顶), [4]=复位入口(PC)。
       如果这两个值不在合理范围, 说明 App 区是空的或被刷坏, 不能跳。 */
    uint32_t app_sp = *(volatile uint32_t*)app_addr;            // 读 App 向量表第 0 项: 栈顶
    uint32_t app_pc = *(volatile uint32_t*)(app_addr + 4);      // 读第 1 项: 复位入口地址
    if (app_sp < APP_RAM_START || app_sp > APP_RAM_END) {       // 栈顶必须落在 SRAM 范围 0x20000000~0x20005000
        Bootloader_EnterFailsafe();                             // 不合法 → 不敢跳, 进故障态等固件
    }
    if (app_pc < APP_ADDR || app_pc > (APP_ADDR + APP_MAX_SIZE)) {  // 入口必须落在 App 代码区
        Bootloader_EnterFailsafe();
    }

    /* ---- 阶段 2: 关中断 → 反初始化 → 跳转 ----
       下面这些操作必须"绝对原子", 任何中断插进来都会用错向量表/时钟, 所以先全关中断。 */
    __disable_irq();                            // 关全局中断 (PRIMASK=1)

    /* 关内核级异常与定时器: 让 Cortex-M 回到"刚复位"的干净状态 */
    SysTick->CTRL = 0;                          // 关 SysTick (否则跳过去后它还在中断)
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;  // 清挂起的 PendSV/SysTick 异常
    __set_BASEPRI(0);                           // 清 BASEPRI (优先级屏蔽寄存器)
    __set_FAULTMASK(0);                         // 清 FAULTMASK (否则 Fault 都被屏蔽)
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;             // 关闭 NVIC 全部 8 组中断使能位
        NVIC->ICPR[i] = 0xFFFFFFFF;             // 清除全部挂起的中断标志
    }

    /* 先 DeInit SPI1 (直接寄存器操)
       —— 顺序讲究: 必须先关外设本身, 再关它的时钟。反过来会留下"时钟没了但外设还使能"的
          不一致状态, 有些外设此时会锁总线。 */
    SPI1->CR1 = 0;                              // 清 SPI1 控制寄存器1 (复位 SPI1)
    SPI1->CR2 = 0;
    SPI1->I2SCFGR = 0;

    /* 再关外设时钟 — 把 Bootloader 用过的外设全部断电, 让 App 从零开始配 */
    RCC->APB1ENR &= ~(RCC_APB1ENR_SPI2EN | RCC_APB1ENR_USART2EN
                     | RCC_APB1ENR_I2C1EN);     // APB1 上的 SPI2/USART2/I2C1
    RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN
                     | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN
                     | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN);   // APB2 上的 SPI1/USART1/GPIO/AFIO
    RCC->AHBENR  &= ~RCC_AHBENR_DMA1EN;         // AHB 上的 DMA1

    /* 复位 GPIO + 清 ODR — CRL/CRH 写 0x44444444 表示"每个引脚都配成浮空输入",
       即把之前配的输出/复用模式全部还原成上电默认态。ODR=0 清输出电平。 */
    GPIOA->CRL = 0x44444444; GPIOA->CRH = 0x44444444; GPIOA->ODR = 0;
    GPIOB->CRL = 0x44444444; GPIOB->CRH = 0x44444444; GPIOB->ODR = 0;
    GPIOC->CRL = 0x44444444; GPIOC->CRH = 0x44444444; GPIOC->ODR = 0;
    AFIO->MAPR = 0;                             // 清引脚重映射配置

    /* 切 HSI, 关 PLL + HSE — 把主频退回内部 8MHz 默认时钟。
       为什么必须退? App 自己会重新配时钟; 若保留 Bootloader 的 PLL 状态,
       App 再配一遍时可能遇到"PLL 已开"的边界情况。退回最保险。 */
    RCC->CFGR &= ~RCC_CFGR_SW;                  // 清时钟源选择位
    RCC->CFGR |= RCC_CFGR_SW_HSI;               // 选 HSI 作系统时钟
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);     // 等切换真正生效
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);  // 关 PLL、关 HSE 晶振

    /* IWDG 保持运行 — 应用 main 第一行必须喂狗
       —— 看门狗一旦使能不能关。所以 App 启动后必须尽快 IWDG_ReloadCounter()。 */

    /* 设 MSP, PSP, VTOR — 这是"跳转"的精髓三连:
         MSP  = App 的初始栈顶 (从 App 向量表读的 app_sp)
         PSP  = 0 (Bootloader 没用进程栈, 清掉)
         VTOR = app_addr (让中断向量表指向 App, 否则中断会跑回 Bootloader) */
    __set_MSP(app_sp);                          // 切换主栈指针到 App 的栈
    __set_PSP(0);                               // 清进程栈指针
    SCB->VTOR = app_addr;                       // 设置向量表偏移寄存器 (关键!)

    /* 跳转 — 把 app_pc 强转成"无参无返回的函数指针"并调用。
       本质是让 CPU 跳去执行 App 的复位入口 (那里面是 App 的启动汇编: 建栈→清BSS→进 main)。 */
    ((void(*)(void))app_pc)();

    while (1);                                  // 理论上不会执行到这里 (跳过去就不回来了), 兜底
}

/* ======================== 故障态 ======================== */

/* 进入故障态: App 不可用 或 升级反复失败时的"自救模式"。
   这里初始化网络, 把本机变成一个小型 TCP 服务端 (端口 506), 然后死循环:
   等上位机连上来重新推送固件。一旦推完并复位, 又会回到 main 重新走升级流程。
   本函数永不返回。 */
void Bootloader_EnterFailsafe(void)
{
    /* 初始化 GPIO, SPI — 跳转前可能把外设都关了, 这里重新把需要的开回来 */
    MX_GPIO_Init();
    MX_SPI1_Init();

    /* 初始化 W5500 — 失败说明网络芯片有问题, 那就彻底没办法了, 只能喂狗死等 */
    if (BSP_W5500_Init() != 0) {
        /* W5500 初始化失败: 死等 — 保持喂狗让板子不重启, 便于用调试器排查硬件 */
        while (1) {
            IWDG_ReloadCounter();
        }
    }

    /* 配置网络参数 — 从 app_config.h 的宏取 (IP 192.168.1.100, 端口 506) */
    uint8_t mac[6] = OTA_MAC;                   // MAC 地址
    uint8_t ip[4]  = OTA_IP;                    // 本机 IP
    uint8_t sub[4] = OTA_SUBNET;                // 子网掩码
    uint8_t gw[4]  = OTA_GW;                    // 网关
    BSP_W5500_SetNetwork(mac, ip, sub, gw);     // 写进 W5500 的通用寄存器

    /* Socket 4 → port 506 — 用 4 号 socket 开一个 TCP 监听服务端 */
    BSP_W5500_SocketInit(4, OTA_PORT);          // 配成 TCP 模式并 OPEN
    BSP_W5500_SocketListen(4);                  // 进入 LISTEN 状态, 等上位机连接

    /* 重置 OTA 状态 — 清干净上一次可能残留的状态, 从 IDLE 开始 */
    OTA_Abort();

    /* 主循环 — 10ms 一个节拍, 轮询 socket 状态、驱动 OTA 状态机, 并喂狗 */
    while (1) {
        delay_ms(10);                           // 节拍 (delay_ms 内部也会喂狗)

        OTA_Tick();                             // 处理异步擦除 + 收包超时检测
        OTA_PollSocket4();                      // 轮询连接/收数据/断连
        IWDG_ReloadCounter();                   // 主循环每圈喂一次狗
    }
}
