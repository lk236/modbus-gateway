/* ============================================================================
 * bsp_w5500.h — W5500 以太网控制器驱动接口
 *
 * SPI1 总线, CS=PB12, RST=PB1。
 * 仅支持 Socket 4 TCP 服务端, 用于 OTA 升级 (故障态下开端口 506 等固件)。
 *
 * 【典型用法】
 *   BSP_W5500_Init()                 → 复位并校验芯片
 *   BSP_W5500_SetNetwork(...)        → 配 MAC/IP/掩码/网关
 *   BSP_W5500_SocketInit(4, 506)     → 配成 TCP + 端口
 *   BSP_W5500_SocketListen(4)        → 进入监听
 *   之后循环: 查状态 / 收数据 / 发响应
 * ============================================================================
 */
#ifndef __BSP_W5500_H
#define __BSP_W5500_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Socket 状态常量 ========================
 * 对应 W5500 的 Sn_SR 寄存器; 值取自 W5500 数据手册 (TCP 状态机)。 */
#define W5500_Sn_SR_CLOSED      0x00    // 已关闭
#define W5500_Sn_SR_INIT        0x13    // 已打开 (TCP 初始态)
#define W5500_Sn_SR_LISTEN      0x14    // 监听中 (等待客户端连接)
#define W5500_Sn_SR_ESTABLISHED 0x17    // 连接已建立 (可收发)
#define W5500_Sn_SR_CLOSE_WAIT  0x1C    // 对端已关闭, 等待本端关闭
#define W5500_Sn_SR_TIME_WAIT   0x1B    // 等待 2MSL 后彻底关闭

/* ======================== 公开 API ======================== */
int     BSP_W5500_Init(void);                       // 复位+校验版本, 返回 0 成功
void    BSP_W5500_SetNetwork(const uint8_t *mac, const uint8_t *ip,
                             const uint8_t *subnet, const uint8_t *gw);   // 配网络参数
void    BSP_W5500_SocketInit(uint8_t sn, uint16_t port);   // 打开 socket (TCP+端口)
void    BSP_W5500_SocketListen(uint8_t sn);         // 进入监听
void    BSP_W5500_SocketDisconnect(uint8_t sn);     // 主动断开
void    BSP_W5500_SocketClose(uint8_t sn);          // 关闭 socket
uint8_t BSP_W5500_SocketStatus(uint8_t sn);         // 读 socket 状态 (Sn_SR)
uint16_t BSP_W5500_SocketRxAvail(uint8_t sn);       // 接收缓冲可读字节数
uint16_t BSP_W5500_SocketTxFree(uint8_t sn);        // 发送缓冲空闲字节数
uint16_t BSP_W5500_SocketRecv(uint8_t sn, uint8_t *buf, uint16_t max_len);  // 收数据
int8_t  BSP_W5500_SocketSend(uint8_t sn, const uint8_t *data, uint16_t len); // 发数据
uint8_t BSP_W5500_ReadPHYLink(void);                // 读网线链路状态

#ifdef __cplusplus
}
#endif

#endif /* __BSP_W5500_H */
