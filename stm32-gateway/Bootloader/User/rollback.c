/* ============================================================================
 * rollback.c — OTA 升级回滚实现 (StdPerip 标准库)
 *
 * 【回滚是什么, 为什么必须有】
 *   升级 = 擦掉旧固件 → 写入新固件。擦除不可逆, 一旦新固件有问题
 *   (CRC 错、断电、写坏), 设备就变成砖头。回滚机制就是"后悔药":
 *   升级前把"当前能跑的固件"整份备份到 W25Q64, 升级失败时再搬回来。
 *
 * 【Flash 分区布局 (W25Q64 8MB, 三个关键区域)】
 *   0x007000  控制块 (19B)      : 有没有待安装固件、失败了几次
 *   0x010000  固件暂存区        : 上位机推上来的新固件先放这 (最大 48KB)
 *   0x01C000  固件备份区        : 升级前把旧 App 备份到这 (最大 48KB)
 *   (0x080000 起是日志区)
 *   注意: 回滚用的"源"是 0x01C000, "目标"是内部 Flash 0x08004000。
 *
 * 【内部 Flash 操作三条铁律】
 *   1) 只能 1→0: 写之前必须先擦成 0xFF。
 *   2) 擦除单位 = 页 (STM32F103 中容量是 1KB); 编程单位 = 半字 (2 字节)。
 *   3) 操作前必须 FLASH_Unlock(), 完成后 FLASH_Lock() 防误写。
 * ============================================================================
 */
#include "rollback.h"       // 回滚 API 声明 + RollbackCtrlBlock 结构
#include "bsp_w25q64.h"     // 外部 Flash 读写/擦除 (备份介质)
#include "hw_init.h"        /* IWDG_ReloadCounter — 长耗时操作中途喂狗 */
#include "ota_crc.h"        // CRC32_ZIP / CRC32_ZIP_Update — 备份与恢复的完整性校验

#include <string.h>         // memcpy
#include <stdio.h>          /* printf — 打印备份/恢复进度日志 */

/* ======================== Flash 操作 (StdPerip) ======================== */

/* 内部 Flash 操作的一层薄封装: 把标准库的"状态枚举返回值"统一成"0 成功 / -1 失败",
   方便上层用 if(... != 0) 判断。这几个函数只做转发, 逻辑都封装在 ST 标准库里。 */

void Flash_Unlock(void)
{
    FLASH_Unlock();                             // 解写保护 (内部往 FLASH_KEYR 写两个魔数序列)
}

void Flash_Lock(void)
{
    FLASH_Lock();                               // 上写保护 (防止跑飞时代码误改 Flash)
}

int Flash_ErasePage(uint32_t page_addr)
{
    return (FLASH_ErasePage(page_addr) == FLASH_COMPLETE) ? 0 : -1;     // 擦一页(1KB)
}

int Flash_ProgramHalfWord(uint32_t addr, uint16_t data)
{
    return (FLASH_ProgramHalfWord(addr, data) == FLASH_COMPLETE) ? 0 : -1;  // 写一个半字(2B)
}

/* ======================== Rollback 操作 ======================== */

/* 判断"备份是否有效": 备份区控制块里 magic == "RBBK" 才算有效。 */
int Rollback_IsValid(void)
{
    uint32_t magic;
    BSP_W25Q64_Read(ROLLBACK_CTRL_ADDR, (uint8_t*)&magic, 4);   // 读控制块第 0 个字段
    return (magic == ROLLBACK_MAGIC) ? 1 : 0;                   // 0x5242424B = "RBBK"
}

/* 标记备份有效 — 升级成功后调用, 把"当前固件"登记为新的回滚基线。
   写入顺序很讲究: 先写 size/crc (字节 4 起), 最后才写 magic (字节 0)。
   因为 magic 是"整块有效"的唯一标志, 最后写 magic 可以保证: 若中途断电,
   magic 还没写 → 整个控制块仍被视为无效 → 不会拿着半个控制块去恢复。 */
void Rollback_MarkValid(uint32_t fw_size, uint32_t fw_crc)
{
    RollbackCtrlBlock ctrl;
    ctrl.magic    = ROLLBACK_MAGIC;             // "RBBK"
    ctrl.fw_size  = fw_size;                    // 固件大小
    ctrl.fw_crc   = fw_crc;                     // 固件 CRC32
    ctrl.reserved = 0xFFFFFFFF;                 // 保留位, 填全 1 (Flash 空白值)

    BSP_W25Q64_SectorErase(ROLLBACK_CTRL_ADDR); // 控制块位于扇区 0x007000, 先擦 (4KB 一擦)
    while (BSP_W25Q64_IsBusy());                // 等擦除完成
    BSP_W25Q64_Write(ROLLBACK_CTRL_ADDR + 4, (uint8_t*)&ctrl.fw_size, 12);  // 先写 size+src+reserved (12B)
    BSP_W25Q64_Write(ROLLBACK_CTRL_ADDR, (uint8_t*)&ctrl.magic, 4);         // 最后写 magic (4B)
}

/* 使备份失效 — 恢复成功后调用, 把 magic 清 0。
   否则下次上电又会看到"有备份", 可能重复走恢复流程。 */
void Rollback_Invalidate(void)
{
    uint32_t zero = 0;
    BSP_W25Q64_Write(ROLLBACK_CTRL_ADDR, (uint8_t*)&zero, 4);   // magic = 0 → IsValid 返回 0
}

/* 备份当前 App 到 W25Q64 备份区 (0x01C000)。
   流程: 擦备份扇区 → 逐块 256B 从内部 Flash 搬到 W25Q64 → 双向 CRC 比对 → 登记有效。
   返回 0 表示备份成功且校验通过。 */
int Rollback_BackupCurrentApp(uint32_t fw_size)
{
    printf("[BOOT] Backup app to W25Q64 0x%06lX (%lu B)...\r\n",
           (unsigned long)ROLLBACK_BACKUP_ADDR, (unsigned long)fw_size);

    /* 1. 按扇区(4KB)擦除备份区 — NOR Flash 写前必擦 */
    for (uint32_t a = ROLLBACK_BACKUP_ADDR;
         a < ROLLBACK_BACKUP_ADDR + fw_size; a += 4096) {
        BSP_W25Q64_SectorErase(a);              // 擦一个 4KB 扇区
        while (BSP_W25Q64_IsBusy());            // 等擦除完成 (W25Q64 擦一扇区约 45ms)
        IWDG_ReloadCounter();                   // 每擦一扇区喂狗 (擦多个扇区可能耗时很久)
    }

    /* 2. 逐块 256B 搬运: 从内部 Flash 读 → 写进 W25Q64 备份区。
       内部 Flash 是"内存映射"的, 可以直接用 memcpy 从地址读;
       W25Q64 不是内存映射的, 必须走 SPI 的 BSP_W25Q64_Write。 */
    uint8_t buf[256];
    for (uint32_t off = 0; off < fw_size; off += sizeof(buf)) {
        uint32_t chunk = fw_size - off;         // 剩余字节
        if (chunk > sizeof(buf)) chunk = sizeof(buf);   // 本块最多 256
        memcpy(buf, (void*)(APP_ADDR + off), chunk);    // 从内部 Flash 直接读 (映射地址)
        if (BSP_W25Q64_Write(ROLLBACK_BACKUP_ADDR + off, buf, chunk) != 0)
            return -1;                          // 写失败 → 备份失败
        if ((off / 256) % 16 == 0) IWDG_ReloadCounter();    // 每 16 块(4KB)喂一次狗
    }

    /* 3. 双向 CRC 校验 — 算"源(内部Flash)"和"目标(W25Q64备份)"的 CRC32, 必须相等 */
    uint32_t app_crc = CRC32_ZIP((const uint8_t*)APP_ADDR, fw_size);    // 源: 直接算整段

    /* 从 W25Q64 回读并计算 CRC (不能直接指针读, 0x01C000 不是内存地址)
       —— 这是本文件一个易错点: 内部 Flash 能 *(uint32_t*) 直接读,
          W25Q64 的 0x01C000 只是"芯片内地址", 必须经 SPI 读出来再算。 */
    uint32_t bkp_crc = 0xFFFFFFFF;              // 流式 CRC 初值
    uint8_t verify_buf[256];
    for (uint32_t off = 0; off < fw_size; off += sizeof(verify_buf)) {
        uint32_t chunk = fw_size - off;
        if (chunk > sizeof(verify_buf)) chunk = sizeof(verify_buf);
        BSP_W25Q64_Read(ROLLBACK_BACKUP_ADDR + off, verify_buf, chunk);
        bkp_crc = CRC32_ZIP_Update(bkp_crc, verify_buf, chunk);     // 增量更新
    }
    bkp_crc ^= 0xFFFFFFFF;                      // 收尾取反, 得到最终 CRC32

    if (bkp_crc != app_crc) {                   // 两者不等 → 备份内容坏了
        printf("[BOOT] Backup CRC mismatch (app=0x%08lX bkp=0x%08lX)\r\n",
               (unsigned long)app_crc, (unsigned long)bkp_crc);
        return -1;
    }

    Rollback_MarkValid(fw_size, app_crc);       // 校验通过 → 登记这份备份有效
    printf("[BOOT] Backup OK\r\n");
    return 0;
}

/* 从备份区恢复固件到内部 Flash。
   流程: 检查有效 → 读参数 → 擦 App 区 → 逐半字搬回 → CRC 校验 → 使备份失效。
   返回 0 成功; -1 无效 / -2 擦失败 / -3 写失败 / -4 校验失败。 */
int Rollback_Restore(void)
{
    if (!Rollback_IsValid()) return -1;         // 没有有效备份, 直接失败

    RollbackCtrlBlock ctrl;
    BSP_W25Q64_Read(ROLLBACK_CTRL_ADDR, (uint8_t*)&ctrl, sizeof(ctrl));    // 读回控制块
    uint32_t fw_size = ctrl.fw_size;            // 取出备份的固件大小

    printf("[BOOT] Restore from W25Q64 0x%06lX (%lu B)...\r\n",
           (unsigned long)ROLLBACK_BACKUP_ADDR, (unsigned long)fw_size);

    Flash_Unlock();                             // 开写保护
    uint32_t end_addr = APP_ADDR + fw_size;     // 要擦写的结束地址
    if (end_addr > APP_ADDR + APP_MAX_SIZE) end_addr = APP_ADDR + APP_MAX_SIZE;  // 不越界
    end_addr = (end_addr + 1023) & ~1023;       // 1KB 页对齐 (向上取整)

    /* 1. 擦除 App 区 (按 1KB 页) */
    for (uint32_t addr = APP_ADDR; addr < end_addr; addr += 1024) {
        if (Flash_ErasePage(addr) != 0) {
            Flash_Lock(); return -2;            // 擦失败 → 上锁退出
        }
        IWDG_ReloadCounter();                   // 喂狗
    }

    /* 2. 逐半字搬运: W25Q64 备份区 → 内部 Flash。
       用半字是因为 STM32F1 内部 Flash 编程最小单位是 2 字节。 */
    for (uint32_t off = 0; off < fw_size; off += 2) {
        uint16_t val;
        BSP_W25Q64_Read(ROLLBACK_BACKUP_ADDR + off, (uint8_t*)&val, 2);     // 读 2B (小端)
        if (Flash_ProgramHalfWord(APP_ADDR + off, val) != 0) {
            Flash_Lock(); return -3;            // 写失败 → 上锁退出
        }
        if ((off / 1024) % 4 == 0) IWDG_ReloadCounter();    // 每 4KB 喂一次狗
    }
    Flash_Lock();                               // 写保护重新上锁

    /* 3. 校验恢复结果 — 把恢复好的 App 区重新算 CRC32, 与控制块里存的比对 */
    uint32_t calc_crc = CRC32_ZIP((const uint8_t*)APP_ADDR, fw_size);
    if (calc_crc != ctrl.fw_crc) return -4;     // 不一致 → 恢复失败

    printf("[BOOT] Restore OK\r\n");
    Rollback_Invalidate();                      // 成功 → 清 magic, 防止重复恢复
    return 0;
}
