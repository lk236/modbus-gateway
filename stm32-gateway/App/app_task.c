/*
 * ============================================================================
 * app_task.c — 系统服务: 软件看门狗 + 异步日志写入
 * ============================================================================
 *
 * 两个不属于业务逻辑、却决定系统可靠性的系统级服务:
 *
 *   1. 软件看门狗 WdgSupervisor
 *      —— 监控 4 个业务任务的心跳。只要有一个任务卡死, 就"不再喂硬件狗",
 *         让 IWDG 在 6.4s 后把整个系统复位, 从死机中自动恢复。
 *
 *   2. 异步日志 LogTask
 *      —— 业务代码只把日志"丢进队列"就立刻返回; 真正耗时的 W25Q64 擦/写
 *         由这个低优先级任务在后台慢慢做, 不阻塞 TCP 收发和 485 轮询。
 *
 * 看门狗做了两层保护: 硬件 IWDG 只认"有没有人喂它", 分不清是谁的问题;
 * 软件看门狗负责"点名", 必须全员到齐才喂狗。
 * ============================================================================
 */
#include "app_task.h"     /* 本模块接口: Log_Push / Wdg_Heartbeat / 各 Init */
#include "bsp_ds3231.h"   /* 日志条目需要 RTC 提供时间戳 */
#include "bsp_iwdg.h"     /* 硬件看门狗喂狗: BSP_IWDG_Reload() */
#include <stdio.h>        /* printf */
#include <string.h>       /* memset */

/* 日志消息队列: 业务侧 Log_Push 往里放条目, LogTask 取出后异步写 Flash */
static osMessageQueueId_t g_log_queue;

/* 各任务的最新心跳时间戳(来自 osKernelGetTickCount, 单位 tick = 1ms)。
 * 下标是任务的"逻辑编号", 由 app_task.h 里的 WDG_TASK_ID_xxx 枚举定义 */
static volatile uint32_t g_wdg_heartbeats[WDG_TASK_COUNT];

/* 每个逻辑编号对应的任务名, 只在打印报警时用于显示 */
static const char *g_wdg_task_names[WDG_TASK_COUNT] = {
    "Default",
    "TCP",
    "Sensor",
    "Log"
};

/* 某个任务超过 15s 没报心跳 → 判定它挂掉了(不是"死了", 是"长时间没汇报") */
#define WDG_ALIVE_TIMEOUT_MS    15000
/* 看门狗监控任务每 2s 检查一次所有心跳 */
#define WDG_CHECK_INTERVAL_MS   2000

/*
 * Wdg_Supervisor_Task — 软件看门狗监控任务(优先级 Low1)
 *
 * 硬件 IWDG 只要没人按时喂 6.4s 后就复位系统, 但它分不清"正常空转"还是
 * "某个任务卡死"——随便哪个任务活着就喂狗的话, 其他任务卡死时保护就失效。
 * 故加一层软件看门狗把关: 必须 4 个任务心跳全新鲜才喂硬件 IWDG; 有一个超时
 * 未报心跳就停止喂狗, 等 IWDG 超时复位系统。
 *
 * 关键点:
 *   1) 要"连续两次全员存活"才喂狗(consecutive_alive >= 2)。任务刚创建时心跳
 *      数组为 0, 首次检查可能因"还没启动完"而误判, 这个门槛避免开机初期误喂。
 *   2) osDelayUntil 是"睡到指定绝对时刻"的固定周期延时, 检查间隔精确等于 2s,
 *      不会因被高优先级任务抢占而漂移。
 */
static void Wdg_Supervisor_Task(void *argument)
{
    uint8_t consecutive_alive = 1;   /* 连续"全员存活"的次数, 初始为 1 */
    TickType_t last_wake = osKernelGetTickCount();  /* 记录本任务首次运行的时刻 */

    printf("[WDG] Supervisor started, timeout=%dms, max_miss=%dms\r\n",
           IWDG_TIMEOUT_MS, WDG_ALIVE_TIMEOUT_MS);

    for (;;) {
        /* 算出下一次该醒来的【绝对时刻】并睡到那时(固定节拍, 防漂移) */
        last_wake += pdMS_TO_TICKS(WDG_CHECK_INTERVAL_MS);
        osDelayUntil(last_wake);

        /* ---- 逐个任务检查心跳是否新鲜 ---- */
        uint8_t all_alive = 1;   /* 先假设全员存活, 发现一个超时就置 0 */
        for (uint8_t i = 0; i < WDG_TASK_COUNT; i++) {
            uint32_t now = osKernelGetTickCount();
            uint32_t elapsed = now - g_wdg_heartbeats[i];  /* 距上次心跳过了多久(无符号减法, 抗回绕) */
            if (elapsed > WDG_ALIVE_TIMEOUT_MS) {
                /* 该任务超过 15s 没报心跳 → 判定它挂了, 打印报警 */
                printf("[WDG] DEAD: %s, last=%lums ago\r\n",
                       g_wdg_task_names[i], elapsed);
                all_alive = 0;
            }
        }

        /* ---- 根据点名结果决定要不要喂硬件狗 ---- */
        if (all_alive) {
            /* 全员都活着: 累加"连续存活"计数(上限 255, 防止 uint8 溢出回绕) */
            consecutive_alive = (consecutive_alive < 255) ? (consecutive_alive + 1) : 255;
            if (consecutive_alive >= 2) {
                BSP_IWDG_Reload();   /* 稳定存活后, 才真正喂硬件 IWDG */
            }
        } else {
            /* 有任务挂了: 清零连续计数, 且本次【不喂狗】→ 硬件狗将在 6.4s 后复位系统 */
            consecutive_alive = 0;
        }
    }
}

/*
 * Wdg_Heartbeat — 各任务上报"我还活着"
 *
 * 每个任务在自己主循环里周期性调用, 把当前 tick 写到对应槽位;
 * 软件看门狗检查这些时间戳"新不新鲜"。
 *
 * 多任务并发写该数组但无需加锁: 每个任务只写自己那一个下标, 而 uint32_t 在
 * Cortex-M3 上是单条 STR 指令(原子), 不存在读到半个数的撕裂问题。
 */
void Wdg_Heartbeat(uint8_t task_id)
{
    if (task_id < WDG_TASK_COUNT) {   /* 边界保护, 防止越界写坏别的内存 */
        g_wdg_heartbeats[task_id] = osKernelGetTickCount();
    }
}

/*
 * Wdg_TaskInit — 创建软件看门狗监控任务
 * 由 defaultTask 在【使能硬件 IWDG 之前】调用, 保证先有"把关人"再开硬件狗。
 */
void Wdg_TaskInit(void)
{
    /* 所有心跳先清零, 表示"所有任务都还没启动" */
    memset((void *)g_wdg_heartbeats, 0, sizeof(g_wdg_heartbeats));

    const osThreadAttr_t attr = {
        .name       = "WdgSupervisor",
        .stack_size = 192 * 4,        /* 768 字节, 该任务逻辑简单, 够用 */
        .priority   = osPriorityLow1, /* 低优先级, 不抢业务任务的时间片 */
    };
    osThreadId_t handle = osThreadNew(Wdg_Supervisor_Task, NULL, &attr);
    if (handle == NULL) {
        printf("[WDG] Supervisor task create failed!\r\n");
    } else {
        printf("[WDG] Supervisor task OK\r\n");
    }
}

/*
 * Log_Task — 异步日志写入任务
 *
 * W25Q64 擦/写很慢(擦一个扇区 ~10ms 甚至更多), 业务代码同步写日志会阻塞
 * 关键流程(TCP 收发、485 轮询)。故用最低优先级任务专门干这活:
 *   业务代码 → Log_Push(只入队, 立刻返回)
 *   本任务   → 从队列取出条目, 慢慢写 Flash
 * 队列本身还起缓冲作用, 吸收业务瞬间产生的多条日志。
 */
static void Log_Task(void *argument)
{
    W25Q64_LogEntry entry;   /* 从队列里取出的一条日志 */

    for (;;) {
        Wdg_Heartbeat(WDG_TASK_ID_LOG);   /* 上报"日志任务还活着" */

        /* 阻塞等队列消息。1000ms 超时是必需的:
         * 就算一直没日志, 也至少每 1s 醒一次, 好让上面的心跳持续上报 ——
         * 否则长时间没有日志时, 本任务会被软件看门狗判成"死"(15s 门限)。 */
        osStatus_t status = osMessageQueueGet(g_log_queue, &entry, NULL, 1000);
        if (status == osOK) {
            BSP_W25Q64_LogWrite(&entry);   /* 真正把日志条目写进 W25Q64 Flash */
        }
    }
}

/*
 * Log_Push — 业务侧"非阻塞"写日志入口
 *
 * 任意模块要记录事件时调用:
 *   type: 事件类型(系统启动 / TCP 连接 / TCP 断开 / 485 故障 / 配置更新等枚举)
 *   data: 附加数据, 例如端口号
 * 内部封装成 W25Q64_LogEntry, 打上当前时间戳后放进队列并立即返回;
 * 真正的 Flash 写入由 LogTask 异步完成。生产者-消费者 + 异步化设计:
 * 调用方耗时从"十几毫秒"降到"几微秒", 代价是掉电时队列里的日志会丢
 * (日志允许有损, 可接受)。
 */
void Log_Push(uint8_t type, uint16_t data)
{
    W25Q64_LogEntry entry;

    entry.timestamp = BSP_DS3231_GetUnixTimestamp();  /* RTC 提供 Unix 秒时间戳 */
    entry.type      = type;
    entry.reserved  = 0;
    entry.data      = data;

    /* 第三个参数 0 = 非阻塞投递: 队列满时直接丢弃(日志有损可接受, 绝不阻塞业务) */
    osMessageQueuePut(g_log_queue, &entry, 0, 0);
}

/*
 * Log_TaskInit — 创建日志消息队列 + 日志写入任务
 * 由 defaultTask 在初始化 RTC、W25Q64 之后调用。
 */
void Log_TaskInit(void)
{
    /* 创建能装 64 条日志的队列, 每个元素是一个 W25Q64_LogEntry */
    g_log_queue = osMessageQueueNew(64, sizeof(W25Q64_LogEntry), NULL);
    if (g_log_queue == NULL) {
        printf("[LOG] Queue create failed!\r\n");
        return;   /* 队列都建不出来, 后面任务也别建了 */
    }

    const osThreadAttr_t attr = {
        .name       = "LogTask",
        .stack_size = 256 * 4,       /* 1KB */
        .priority   = osPriorityLow, /* 最低优先级: 让出 CPU 给业务任务 */
    };
    osThreadId_t handle = osThreadNew(Log_Task, NULL, &attr);
    if (handle == NULL) {
        printf("[LOG] Task create failed!\r\n");
    } else {
        printf("[LOG] Task created OK\r\n");
    }
}
