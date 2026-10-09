/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* ============================================================================
 * main.c — 系统入口和启动流程
 *
 * 这个文件是单片机通电后执行的第一段代码, 按以下顺序运行:
 *
 *   第 1 步: main() 开头 → 调用 HAL_Init() 初始化硬件抽象层
 *   第 2 步: SystemClock_Config() → 把系统时钟设为 72MHz
 *   第 3 步: MX_XXX_Init() → 逐个初始化各个外设 (GPIO / I2C / SPI / USART)
 *   第 4 步: osKernelInitialize() → 启动 FreeRTOS 实时操作系统
 *   第 5 步: MX_FREERTOS_Init() → 创建系统第一个任务 (StartDefaultTask)
 *   第 6 步: osKernelStart() → 把控制权交给 FreeRTOS, 任务开始运行
 *
 * 第 6 步之后, main() 就不再干活了, 后面那个 while(1) 只是保底。
 * 真正的业务代码都在 freertos.c 的 StartDefaultTask 里启动。
 * ============================================================================
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"      /* FreeRTOS 的 CMSIS-RTOS v2 封装 (任务/队列/信号量) */
#include "i2c.h"            /* I2C1 → 连接 DS3231 RTC 时钟芯片 */
#include "spi.h"            /* SPI1 → W25Q64 (CS=PA4); SPI2 → W5500 网卡 (CS=PB12) */
#include "usart.h"          /* USART1 → 调试串口输出 (printf) / USART2 → RS-485 总线 */
#include "gpio.h"           /* GPIO 引脚初始化 */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "stdio.h"       /* printf 重定向到 USART1 调试串口 */
	#include "bsp_printf_dma.h"  /* printf DMA 环形缓冲 (DMA1_Ch4) */
	
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* ============================================================================
 * main() — 单片机主入口
 *
 * 执行顺序(从上往下):
 *   ┌─ 1. HAL_Init()         ─ MCU 硬件抽象层初始化
 *   ├─ 2. SystemClock_Config() ─ 系统时钟设为 72MHz (8MHz晶振 x9 倍频)
 *   ├─ 3. MX_GPIO_Init()     ─ GPIO 引脚初始化 (所有引脚默认状态)
 *   ├─ 4. MX_I2C1_Init()     ─ I2C1 总线初始化 (接 DS3231 RTC 时钟芯片)
 *   ├─ 5. MX_SPI1_Init()     ─ SPI1 初始化 (接 W25Q64 SPI Flash, CS=PA4)
 *   ├─ 6. MX_USART2_Init()   ─ USART2 初始化 (RS-485 总线, 9600波特率)
 *   ├─ 7. MX_SPI2_Init()     ─ SPI2 初始化 (接 W5500 以太网控制器, CS=PB12)
 *   ├─ 8. MX_USART1_Init()   ─ USART1 初始化 (调试串口, printf输出用)
 *   ├─ 9. osKernelInitialize() ─ 启动 FreeRTOS 内核
 *   ├─10. MX_FREERTOS_Init() ─ 创建 defaultTask (就是 StartDefaultTask)
 *   └─11. osKernelStart()    ─ 启动任务调度器 → 控制权交给 FreeRTOS
 *
 * 第 11 步之后, 系统就交给 FreeRTOS 了。FreeRTOS 会运行:
 *   - StartDefaultTask  (freertos.c) → 初始化所有业务模块, 然后每 1s 喂狗
 *   - TcpServerTask     (app_modbus.c) → 4 个 TCP Socket, 用状态机管理
 *   - HealthCheckTask   (app_sensor.c) → 每 60s 检查 RS-485 总线健康
 *   - WdgSupervisor     (app_task.c) → 监控其他任务心跳, 决定是否喂硬件看门狗
 *   - LogTask           (app_task.c) → 把事件日志写到 W25Q64 Flash
 *
 * @retval int  (实际上永远不会返回, 因为 FreeRTOS 接管了 CPU)
 * ============================================================================
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* (STM32CubeMX 保留区: 用户可以在 HAL_Init() 之前加自己的初始化) */
  /* OTA: 设置中断向量表偏移到应用首地址 (Bootloader 跳转后执行) */
	SCB->VTOR = 0x8004000;
  /* IWDG: 喂狗 — Bootloader 跳转后保持 IWDG 运行, 此处必须第一时间喂 */
  IWDG->KR = 0xAAAA;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
	//SCB->VTOR = FLASH_BASE | 0x4000;
  /* ---- 第 1 步: HAL 初始化 ----------------------------------------------- */
  /* HAL_Init() 做三件事:
   *   1) 设置 Flash 预取缓冲
   *   2) 设置 SysTick 定时器 (作为 HAL 的时基, 1ms 中断一次)
   *   3) 初始化 NVIC (嵌套向量中断控制器) */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* (STM32CubeMX 保留区: 可以在 HAL_Init 和 SystemClock_Config 之间加代码) */
  /* USER CODE END Init */

  /* ---- 第 2 步: 配置系统时钟 --------------------------------------------- */
  /* SystemClock_Config() 把时钟调到 72MHz:
   *   外部 8MHz 晶振(HSE) → PLL 倍频 x9 = 72MHz → SYSCLK
   *   APB1 总线 = 36MHz  (APB1 最高只能 36MHz)
   *   APB2 总线 = 72MHz  (APB2 最高 72MHz) */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* (STM32CubeMX 保留区: 时钟配置完成后可以加代码) */
  /* USER CODE END SysInit */

  /* ---- 第 3~8 步: 初始化各个外设 ----------------------------------------- */
  /* 注意顺序: 先 GPIO (所有引脚默认状态), 再 I2C, 再 SPI, 再 USART */
  MX_GPIO_Init();               /* 所有 GPIO 初始化 (输入/输出/复用功能) */
  MX_I2C1_Init();               /* I2C1 → DS3231 RTC 时钟芯片 */
  MX_SPI1_Init();               /* SPI1 → W25Q64 SPI Flash (片选 PA4) */
  MX_USART2_UART_Init();        /* USART2 → RS-485 总线 (半双工, 9600波特率) */
  MX_SPI2_Init();               /* SPI2 → W5500 以太网控制器 (片选 PB12) */
  /* USER CODE BEGIN 2 */
  MX_USART1_UART_Init();        /* USART1 → 调试串口 (printf 输出, 115200波特率) */
  BSP_PrintfDma_Init();         /* printf 改 DMA 环形缓冲输出 (首 printf 前必须) */
  /* USER CODE END 2 */

  /* ---- 第 9~10 步: 启动 FreeRTOS 并创建任务 ------------------------------ */
  osKernelInitialize();         /* 初始化 FreeRTOS 内核 (任务调度器、内存管理) */
  MX_FREERTOS_Init();           /* 创建 defaultTask (即 StartDefaultTask 任务) */

  /* ---- 第 11 步: 启动调度器, 把控制权交给 FreeRTOS ----------------------- */
  osKernelStart();              /* 启动任务调度器 → 从此 main() 不再执行 */

  /* ==========================================================================
   * 以下代码正常情况下永远不会执行到。
   * 只有在 FreeRTOS 配置错误、任务全部被删除等极端情况下才会跑到这里。
   * while(1) 死循环是为了兜底, 防止程序跑飞。
   * ========================================================================== */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* ============================================================================
 * SystemClock_Config() — 系统时钟配置
 *
 * 本项目的晶振是 8MHz (HSE), 目标是让 CPU 跑到 72MHz, 计算公式:
 *   系统时钟 SYSCLK = 8MHz(HSE) × 9(PLL倍频) = 72MHz
 *
 * 总线分频:
 *   AHB  (高速总线) = 72MHz  (不分频, CPU、内存、Flash 都在这个总线上)
 *   APB1 (低速外设) = 36MHz  (分频 2, 因为 APB1 最高只能 36MHz)
 *   APB2 (高速外设) = 72MHz  (不分频, GPIO、SPI1、USART1 都在这里)
 *
 * Flash 等待周期: 72MHz 下必须设 2 个等待周期, 否则 CPU 读 Flash 会出错
 * ============================================================================
 */
void SystemClock_Config(void)
{
  /* ---- 这两个结构体用来配置振荡器和时钟 -------------------------------- */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  /* 振荡器配置 (HSE/HSI/PLL) */
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  /* 时钟树配置 (SYSCLK/AHB/APB) */

  /* ---- 配置振荡器: HSE(8MHz) + PLL(×9) = 72MHz ------------------------- */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  /* 告诉 HAL 要配 HSE */
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                    /* 打开外部 8MHz 晶振 */
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;     /* HSE 不分频 (8MHz 直接进 PLL) */
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                    /* 内部 8MHz 振荡器也开着, 做备用 */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                /* 打开 PLL */
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;        /* PLL 的输入源选 HSE */
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;                /* PLL 倍频 9 倍: 8×9=72MHz */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();  /* 如果时钟配置失败 → 死循环 */
  }

  /* ---- 配置总线时钟: 把 72MHz 分配到各个总线 ---------------------------- */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;    /* 系统时钟 = PLL 输出 72MHz */
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;           /* AHB 不分频 = 72MHz */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;            /* APB1 分频 2 = 36MHz */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;            /* APB2 不分频 = 72MHz */

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();  /* 如果总线时钟配置失败 → 死循环 */
  }
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    /* printf 重定向: 写入 DMA 环形缓冲, 非阻塞, 由 DMA1_Ch4 后台搬运 */
    return BSP_PrintfDma_PutChar(ch);
}

/* USER CODE END 4 */

/**
  * @brief  HAL 错误处理 — 关中断后死循环
  *         如果发生无法恢复的 HAL 错误 (时钟/外设配置失败) 则进入此处
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 关闭所有中断, 防止看门狗复位前再产生错误中断 */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
