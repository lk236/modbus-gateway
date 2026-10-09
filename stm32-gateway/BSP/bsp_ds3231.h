#ifndef __BSP_DS3231_H
#define __BSP_DS3231_H

#include "main.h"

#define DS3231_I2C_ADDR      0xD0

/* DS3231 日期时间结构体 (所有字段 BCD 解码后的值) */
typedef struct {
    uint8_t year;    /* 年份后两位 (如 26 = 2026) */
    uint8_t month;   /* 月份 1-12 */
    uint8_t date;    /* 日期 1-31 */
    uint8_t weekday; /* 星期几 1-7 */
    uint8_t hour;    /* 小时 0-23 */
    uint8_t minute;  /* 分钟 0-59 */
    uint8_t second;  /* 秒钟 0-59 */
} DS3231_DateTime;

uint8_t  BSP_DS3231_Init(void);
uint8_t  BSP_DS3231_GetTime(DS3231_DateTime *dt);
uint8_t  BSP_DS3231_SetTime(DS3231_DateTime *dt);
uint32_t BSP_DS3231_GetUnixTimestamp(void);
void     BSP_DS3231_SetUnixTimestamp(uint32_t ts);

#endif
