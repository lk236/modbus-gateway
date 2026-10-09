/* ============================================================================
 * main.c — Bootloader 主程序 (精简版)
 *
 * 【这个文件在整个系统里的位置】
 *   上电 → 芯片从 0x08000000 取向量表, 先跑 Bootloader (本文件), 而不是 App。
 *   Bootloader 只干三件事:
 *     1) 初始化最小硬件 (时钟 / GPIO / SPI / 串口 / 看门狗)
 *     2) 看 W25Q64 里的 OTA 控制块: 有没有"待安装的新固件"? 该不该回滚?
 *     3) 要么把新固件搬进内部 Flash 并校验, 要么直接跳去 App
 *
 *   ┌─ 上电 ─┐    ┌──────────────┐   有有效控制块?  ┌──── 是 ────┐
 *   │ 复位    │ →  │ 读控制块     │ ──── 否 ────→    │ 直接跳 App │
 *   └────────┘    └──────────────┘                  └───────────┘
 *                        │ 是
 *                        ▼
 *              fail_count >= 3 ? ── 是 ─→ 从 W25Q64 备份恢复(回滚)
 *                        │ 否
 *                        ▼
 *              备份当前 App → 擦内部Flash → 搬固件 → CRC32 校验
 *                        │                    │
 *                     校验OK                校验失败
 *                        │                    │
 *                   清 magic,跳App        回滚 或 进故障态(网络收 OTA)
 *
 * 【为什么 Bootloader 要单独存在】
 *   内部 Flash 从 0x08004000 起才是 App。Bootloader 占了最前面的 16KB,
 *   它永远不被擦除 (升级时只擦 App 区), 所以就算 App 被刷坏, Bootloader
 *   还能跑, 还能进"故障态"从网络重新收固件 —— 这就是"刷不死"的保障。
 *
 * 【本文件的三个关键宏 (都在 app_config.h / ota_protocol_def.h)】
 *   APP_ADDR      = 0x08004000  App 起始地址 (Bootloader 16KB 之后)
 *   APP_MAX_SIZE  = 0xC000       App 区最大 48KB (0x08004000 ~ 0x08010000)
 *   STAGING_ADDR  = 0x010000     W25Q64 里"新固件暂存区"的地址
 *
 * 编译: Keil MDK-ARM, ARMCC V5, -O1 优化 (Keil Optim=1, 见 Bootloader.uvprojx:314)
 * 链接: 0x08000000, 最大 16KB
 * ============================================================================
 */
#include "stm32f10x.h"                  // Device header — 寄存器定义 (RCC/FLASH/NVIC...)
#include "app_config.h"                 // 板级配置: 引脚宏、网络参数、OTA 公共头文件
#include "hw_init.h"                    // 硬件初始化声明: 时钟/GPIO/SPI/延时/喂狗
#include "bsp_w25q64.h"                 // 外部 SPI Flash 驱动 (存控制块/暂存固件/备份)
#include "bkp_fault.h"                  // 备份寄存器故障标志 (掉电不丢的 1 个标记位)
#include "rollback.h"                   // 回滚: 备份当前 App / 恢复旧固件
#include "boot_jump.h"                  // JumpToApp / Bootloader_EnterFailsafe

#include <stdio.h>          /* printf — 走串口 USART1, 用于打印启动日志 */

/* ======================== main ======================== */

int main(void)
{
    /* ---- 第 0 步: 建立"最小可运行环境" ---- */
    SystemClock_Config();                   // 配 PLL: HSE 8MHz → 72MHz (SystemCoreClock=72e6)
    SysTick_Config(SystemCoreClock / 1000); /* 1ms — SysTick 每 1ms 中断一次, g_tick++ (时基) */

    MX_GPIO_Init();                         // 初始化 PA4/PB12/PB1/PC13 (CS、RST、LED), 默认拉高
    MX_USART1_UART_Init();      /* 串口输出 — printf 走 PA9(TX) 115200 8N1 */
    MX_SPI1_Init();                         // 初始化 SPI1 (W25Q64 与 W5500 共用这条总线)

    /* IWDG — 独立看门狗: 一旦使能就无法关闭 (除非复位)。
       所以从这一行往后, 任何超过约 27.3s 的死循环都会导致复位。
       各耗时循环里必须插 IWDG_ReloadCounter() 喂狗。 */
    MX_IWDG_Init();

    /* W25Q64 初始化 — 读 JEDEC ID 校验芯片在不在 (MID 应为 0xEF)。
       通信失败说明硬件有问题: 此时控制块也读不了, 无法升级, 只能尝试跳 App。 */
    if (BSP_W25Q64_Init() != 0) {
        /* W25Q64 通信失败 → 尝试跳应用 */
        goto JUMP_TO_APP;
    }

    {   /* --- 块作用域: 防止 goto 绕过变量初始化 ---
            C 语言规定 goto 不能跳入"有初始化的变量作用域"。这里把后续所有
            局部变量 (fault/ctrl/ret/...) 圈进一个块里, 让 goto JUMP_TO_APP
            从块外跳走时合法 (跳过的是整个块, 而不是块内部的声明)。 */

    /* ---- 第 1 步: 处理"上次启动留下的故障标记" ---- */
    /* 检查 BKP 故障标志 — BKP_DR1 是备份域寄存器, 主电源断电但 VBAT 还在时内容不丢。
       ota_handler 写控制块失败时会置这个标记 (见 OTA_HandleCmd 的 CMD_FINISH 分支)。 */
    uint16_t fault = BKP_ReadFaultFlag();   // 读标记 (FAULT_MAGIC=0xFA 表示"上次写控制块失败")
    (void)fault;            /* 暂不使用 — 读出来只是为了流程完整/便于调试打断点 */
    BKP_ClearFaultFlag();                   // 读完就清, 避免影响下次判断

    /* ---- 第 2 步: 读 OTA 控制块, 决定"升级 / 回滚 / 跳转" ---- */
    /* 读取 OTA 控制块 — 它是 W25Q64 0x007000 处 19 字节的小结构, 记录:
       magic(是否有效) / firmware_size / firmware_crc32 / fail_flag[3] / last_error
       Bootloader 和 App 共用同一份定义 (OtaCommon/ota_ctrl_block.h), 禁止各写一份。 */
    OTACtrlBlock ctrl;
    int ret = OTA_ReadCtrlBlock(&ctrl);     // 返回 0 表示 magic 对且 CRC16 校验通过

    if (ret == 0) {
        /* ---- 控制块有效 → 说明"有一段新固件正在等待安装" ---- */

        /* 失败次数耗尽 → 尝试从 W25Q64 备份恢复
           fail_flag 是 3 个字节, 每失败一次把其中一个 0xFF 改成 0x00。
           3 个都变成 0x00 就说明连续 3 次升级都没成功, 不能再盲目重试了
           (否则可能陷入"刷坏→重启→再刷坏"的无限重启地狱), 必须回滚。 */
        if (GetFailCount(&ctrl) >= 3) {
            printf("[BOOT] fail_count=3, trying rollback...\r\n");
            if (Rollback_Restore() == 0) {          // 从 W25Q64 备份区把旧固件写回内部 Flash
                uint32_t zero = 0;
                BSP_W25Q64_Write(CTRL_BLOCK_ADDR, (uint8_t*)&zero, 4);  // 清 magic → 控制块失效
                printf("[BOOT] Rollback OK, jumping to app\r\n");
                JumpToApp(APP_ADDR);                // 回滚成功, 直接跳 App (不返回)
            }
            printf("[BOOT] No backup, jumping to app (if valid)\r\n");
            goto JUMP_TO_APP;                       // 连备份都没有, 只能赌 App 还能跑
        }

        /* 校验固件参数 — 三重检查, 参数不合法说明控制块被写坏/固件异常。
           任一不满足都标记一次失败然后复位 (靠 fail_flag 计数最终触发回滚)。 */
        if (ctrl.firmware_size < APP_MIN_SIZE ||       // 太小 (最小 1KB): 不可能是合法固件
            ctrl.firmware_size > APP_MAX_SIZE ||       // 太大 (超 48KB): 装不进 App 区
            ctrl.firmware_size % 2 != 0) {             // 非偶数: STM32F1 内部 Flash 只能按半字(2B)写
            OTA_MarkFail(ERR_SIZE_INVALID, ctrl.firmware_size, ctrl.firmware_crc32);
            NVIC_SystemReset();                        // 软复位, 重新走一遍流程
        }

        /* ===== 执行升级 (含回滚) ===== */
        uint32_t fw_size = ctrl.firmware_size;  // 把参数取到局部变量, 后续多次使用
        uint32_t fw_crc  = ctrl.firmware_crc32; // 期望的 CRC32 (纯固件, 不含 Header)

        /* 0. 备份当前 App 到 W25Q64
           在擦除 App 之前先把"现在能跑的固件"整份存进 W25Q64 的备份区。
           这样万一新固件有问题, 还能恢复回来。bkp_ok=0 表示备份失败 ——
           此时不中止升级, 只是"没有后悔药", 后面回滚分支会检查 bkp_ok。 */
        int bkp_ok = (Rollback_BackupCurrentApp(fw_size) == 0);
        if (!bkp_ok) {
            printf("[BOOT] Backup FAIL, continuing anyway\r\n");
        }

        /* 1. 按 1KB 页擦除内部 Flash
           NOR Flash 的铁律: 只能 1→0, 要写新数据必须先擦成 0xFF。
           STM32F103 内部 Flash 的页大小是 1KB (中容量), 擦除粒度就是页。 */
        Flash_Unlock();                         // 解 Flash 写保护 (FLASH->KEYR 写序列)
        uint32_t start_addr = APP_ADDR;         // 从 App 起始地址开始擦
        uint32_t end_addr = APP_ADDR + fw_size; // 擦到"App起始 + 固件大小"
        if (end_addr > APP_ADDR + APP_MAX_SIZE) // 防御: 不允许越过 App 区上界
            end_addr = APP_ADDR + APP_MAX_SIZE;
        end_addr = (end_addr + 1023) & ~1023;   /* 1KB 对齐 — 向上取整到页边界 */
                                                /* (x+1023)&~1023 即"凑到 1KB 的整数倍" */

        for (uint32_t addr = start_addr; addr < end_addr; addr += 1024) {   // 逐页(1KB)擦
            if (Flash_ErasePage(addr) != 0) {
                printf("[BOOT] Erase fail, trying rollback...\r\n");
                OTA_MarkFail(ERR_FLASH_WRITE, fw_size, fw_crc);         // 记一次失败
                Flash_Lock();                                           // 擦失败也要重新上锁
                if (bkp_ok && Rollback_Restore() == 0) {                // 有备份就恢复
                    JumpToApp(APP_ADDR);                                // 恢复成功 → 跳 App
                }
                Bootloader_EnterFailsafe();                             // 没备份 → 网络等新固件
            }
            IWDG_ReloadCounter();               // 每擦一页喂一次狗 (擦一页约 20~40ms)
        }

        /* 2. 从 W25Q64 固件暂存区写入内部 Flash
           注意固件此刻还在 W25Q64 (STAGING_ADDR), 内部尺寸按半字(2 字节)搬运 ——
           因为 STM32F1 的 Flash 编程最小单位就是半字 (FLASH_ProgramHalfWord)。 */
        for (uint32_t off = 0; off < fw_size; off += 2) {
            uint16_t val;
            BSP_W25Q64_Read(STAGING_ADDR + off,
                           (uint8_t*)&val, 2);          // 从外部 Flash 读 2 字节 (小端写入 val)
            if (Flash_ProgramHalfWord(APP_ADDR + off, val) != 0) {      // 写进内部 Flash
                printf("[BOOT] Write fail at off=%lu, trying rollback...\r\n",
                       (unsigned long)off);
                OTA_MarkFail(ERR_FLASH_WRITE, fw_size, fw_crc);
                Flash_Lock();
                if (bkp_ok && Rollback_Restore() == 0) {
                    JumpToApp(APP_ADDR);
                }
                Bootloader_EnterFailsafe();
            }
            if ((off / 1024) % 4 == 0) IWDG_ReloadCounter();  // 每 4KB(2048个半字)喂一次狗
        }                                       // 搬运期间若不喂狗会超时复位

        /* 3. 流式 CRC32 回读校验
           写完不能信任: 必须把内部 Flash 里的内容重新读出来算 CRC32,
           和上位机下发的期望值比对。分段(256B)读是为了省 RAM 并定期喂狗。 */
        uint32_t calc_crc = 0xFFFFFFFF;         // CRC32 初值 (与 CRC32_ZIP 内部一致)
        uint8_t crc_buf[256];                   // 流式缓冲, 每次处理 256 字节
        for (uint32_t off = 0; off < fw_size; off += 256) {
            uint32_t chunk = fw_size - off;     // 剩余字节
            if (chunk > 256) chunk = 256;       // 本段最多 256
            memcpy(crc_buf, (void*)(APP_ADDR + off), chunk);   // 直接从 Flash 地址读 (内存映射)
            calc_crc = CRC32_ZIP_Update(calc_crc, crc_buf, chunk); // 增量更新 (不取反)
            if ((off / 256) % 16 == 0) IWDG_ReloadCounter();   // 每 16 段(4KB)喂一次狗
        }
        calc_crc ^= 0xFFFFFFFF;                 // 收尾取反 (CRC32-ZIP 的最后一步)

        Flash_Lock();                           // 校验完, 重新上锁保护

        if (calc_crc == fw_crc) {
            /* ---- 成功: 清 magic + 回读 3 次 ----
               控制块的 magic 是"还有待安装固件"的标志。清掉它 = 宣告升级完成,
               下次上电就不会再走升级分支。这里连续写+回读 3 次, 是防止
               单次 SPI 写失败导致 magic 没清干净 (那会导致无限重复安装)。 */
            uint32_t zero = 0;
            int clear_ok = 0;
            for (int r = 0; r < 3; r++) {
                BSP_W25Q64_Write(CTRL_BLOCK_ADDR, (uint8_t*)&zero, 4);  // 把 magic 写成 0
                uint32_t rb;
                BSP_W25Q64_Read(CTRL_BLOCK_ADDR, (uint8_t*)&rb, 4);     // 回读确认
                if (rb == 0) { clear_ok = 1; break; }                   // 确认清零 → 退出
            }
            if (!clear_ok) {
                Bootloader_EnterFailsafe();         // 3 次都没清掉 → 不敢跳 App, 进故障态
            }
            /* 升级成功: 更新 Rollback 为新固件
               新固件既然已经验证通过、能跑了, 它就变成了"当前版本",
               把它作为下一次升级时的回滚基线 (否则会回滚到更旧的版本)。 */
            if (bkp_ok) {
                Rollback_MarkValid(fw_size, fw_crc);
            }
            goto JUMP_TO_APP;                       // 跳去新 App
        } else {
            /* CRC 不匹配 → 尝试回滚 — 搬进去的固件和期望的不一致, 不能运行 */
            printf("[BOOT] CRC mismatch, trying rollback...\r\n");
             if (bkp_ok && Rollback_Restore() == 0) {
                JumpToApp(APP_ADDR);                // 恢复旧固件并跳转
            }
            OTA_MarkFail(ERR_STAGING_CRC, fw_size, fw_crc);   // 无备份: 记账后再进故障态
            Bootloader_EnterFailsafe();
        }
    }
    /* ret != 0: 控制块无效 (magic 不匹配或 CRC 错) → 无需升级, 直接往下跳 App */

    }   /* --- 块作用域结束 --- */

JUMP_TO_APP:
    JumpToApp(APP_ADDR);                        // 最终出口: 校验 App 向量表后跳转
    /* 不返回 — JumpToApp 内部要么跳走, 要么进故障态死循环 */
    return 0;
}
