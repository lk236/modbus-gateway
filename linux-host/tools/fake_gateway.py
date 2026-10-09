#!/usr/bin/env python3
# =============================================================================
# fake_gateway.py — Modbus-TCP 从站模拟器（"假网关"）
#
# 【它是干什么的 / 为什么需要它】
#   linux-host 上位机是一个 Modbus-TCP 主站（Master），要定时去问下位机
#   "温度多少、湿度多少"。但真实的下位机（stm32-gateway STM32 板子）只有一块，
#   而且调试时经常不在手边 —— 想测"多网关""断线重连""热加载"这些功能就没法测。
#
#   于是用 Python 写一个纯软件的"从站"顶替硬件：它在指定端口监听，收到主站的
#   读寄存器请求就按格式回一个响应。因为是纯软件，可以：
#     · 一条命令起好几个实例 → 测多网关
#     · 随时 kill 掉 / 重启 → 测断线重连与隔离性
#     · 指定返回任意温湿度值 → 测报警阈值与去抖
#   这样上位机的全部逻辑都能在没有硬件的情况下调通（见 tools/test_all.sh）。
#
# 【实现什么协议】
#   Modbus-TCP（注意不是 Modbus-RTU）。TCP 帧比 RTU 多 7 字节 MBAP 头、且不带 CRC
#   （校验交给 TCP 层），帧结构见下方"解析请求"处的注释。只实现本项目的功能码：
#     0x03 读保持寄存器  ← 主站实际在用这个
#     0x04 读输入寄存器  ← 按 03 同样处理
#     0x06 写单寄存器    ← 为了让上位机的"写"功能也能测
#
# 【寄存器映射（必须与 stm32-gateway 的 modbus_map.h 完全一致）】
#     0x0300 = 温度，int16，单位 0.1℃，例如 0x0107 = 263 → 26.3℃
#     0x0301 = 湿度，int16，单位 0.1%RH，例如 0x0226 = 550 → 55.0%
#
# 用法:
#     python3 fake_gateway.py [端口] [温度℃] [湿度%RH]
#
# 示例:
#     python3 fake_gateway.py 5021              # 端口5021, 26.3℃ 55.0%(默认值)
#     python3 fake_gateway.py 5022 28.5 60.0    # 端口5022, 28.5℃ 60.0%
#     python3 fake_gateway.py 5023 30.0         # 端口5023, 30.0℃ 55.0%(湿度用默认)
# =============================================================================
import socket      # TCP 套接字：listen / accept / recv / send
import struct      # 按大端序打包/解包 Modbus 帧里的多字节字段
import sys         # sys.argv 读命令行参数
import signal      # 捕获 Ctrl+C / kill，实现优雅退出（关掉监听套接字）

# ---- 命令行参数 ----
# 三元表达式 chain：有参数就用参数，没有就用默认值。
# int(sys.argv[1]) 若传了非数字会抛 ValueError 直接崩 —— 对调试工具来说够用了。
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5021      # 监听端口，默认 5021
TEMP_C = float(sys.argv[2]) if len(sys.argv) > 2 else 26.3  # 温度(℃)，默认 26.3
HUM_PCT = float(sys.argv[3]) if len(sys.argv) > 3 else 55.0 # 湿度(%RH)，默认 55.0

# 浮点 → 寄存器原始值：乘 10 再四舍五入成整数（与 stm32-gateway modbus_map.h 的定标一致）
# round() 而不是 int()，是为了避免 26.3*10=262.9999... 被截断成 262（少 0.1℃）
TEMP_RAW = int(round(TEMP_C * 10))   # 例：26.3 → 263 = 0x0107
HUM_RAW = int(round(HUM_PCT * 10))   # 例：55.0 → 550 = 0x0226

# ---- 全局变量，用于信号处理优雅退出 ----
running = True    # 主循环的开关。收到 SIGINT/SIGTERM 时置 False，循环退出


def signal_handler(sig, frame):
    """信号回调：Ctrl+C 或 kill 时被调用

    为什么要专门处理：默认行为是"立即终止"，但那样监听套接字不会 close()，
    端口会短暂处于 TIME_WAIT / 被占用状态，马上重启假网关可能 bind 失败。
    置标志位走正常流程，能保证 finally 里把 srv.close() 执行掉。
    """
    global running       # 声明改的是全局变量，否则这里会新建一个同名局部变量
    print(f"\n[假网关] 收到信号 {sig}, 正在退出...")
    running = False


# 注册信号处理器：SIGINT = Ctrl+C，SIGTERM = kill <pid> 的默认信号
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ---- 启动 TCP 服务器 ----
# AF_INET = IPv4，SOCK_STREAM = TCP（Modbus-TCP 走 TCP，端口一般 502）
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# SO_REUSEADDR：允许在 TIME_WAIT 状态下重新 bind 同一端口。
# 调试时经常"改了参数重跑"，没有这行会频繁报 "Address already in use"。
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

# 设置 1 秒超时：让阻塞的 accept() 每秒醒一次。
# 这样按下 Ctrl+C 时，信号回调置的 running=False 能在一秒内被主循环看到；
# 否则 accept() 会一直阻塞，进程要等有连接进来才响应退出。
srv.settimeout(1.0)

# 0.0.0.0 = 监听本机所有网卡（本地调试 / 跨机访问都行）
srv.bind(('0.0.0.0', PORT))
# 5 = 内核 backlog：已完成三次握手但还没被 accept 的连接最多排 5 个。
# 单主站轮询场景够用了，排不下会多等一轮，不影响功能。
srv.listen(5)

# ---- 启动信息（方便对照 test_all.sh 的输出来排查） ----
print(f"[假网关] 模拟 Modbus-TCP 从站")
print(f"[假网关] 监听 TCP:{PORT}")
print(f"[假网关] 温度={TEMP_C}℃  (raw=0x{TEMP_RAW:04X})")   # 04X：4 位十六进制大写
print(f"[假网关] 湿度={HUM_PCT}% (raw=0x{HUM_RAW:04X})")
print(f"[假网关] Ctrl+C 停止")
print(f"[假网关] 测试命令: mbpoll 127.0.0.1 -p {PORT} -r 0x0300 -c 2 -t 3:int")
print("")

# 寄存器映射表：地址 → 原始值。用 dict 而不是两个变量，
# 是因为功能码 06（写寄存器）会修改它，且读时按地址偏移取值更通用。
#   0x0300 = 温度 (int16, ×0.1)
#   0x0301 = 湿度 (int16, ×0.1)
REGS = {
    0x0300: TEMP_RAW,
    0x0301: HUM_RAW,
}

# ============================ 主循环：一次处理一个连接 ============================
# 注意：这是**单线程**模型 —— 同一时刻只能服务一个连接。
# 对调试工具足够了：上位机每个网关用一条独立连接、且是串行轮询的。
while running:
    try:
        conn, addr = srv.accept()      # 阻塞等新连接（最多阻塞上面设的 1 秒）
    except socket.timeout:
        continue                       # 超时属于正常现象，回到 while 重新判断 running
    except OSError:
        break                          # 套接字被关闭等异常 → 直接退出循环

    print(f"[假网关] 新连接来自 {addr[0]}:{addr[1]}")   # addr = (IP, 端口)
    conn.settimeout(5.0)               # 单条连接上 recv 最多等 5 秒，防止死等占住进程

    try:
        # ---- 收一帧请求 ----
        # 简化处理：不处理 TCP 粘包/半包，假设一帧请求能一次收全。
        # Modbus-TCP 请求都很短（12 字节左右），本地回环下基本都一次到齐。
        data = conn.recv(1024)
        if not data or len(data) < 8:  # 空（对端关闭）或不足 MBAP 头长度 → 无效
            print(f"  → 数据过短 ({len(data)} 字节), 关闭")
            conn.close()
            continue

        # ---- 解析 Modbus-TCP 请求 ----
        # 【MBAP 头，7 字节】= Modbus Application Protocol header
        #   偏移 0~1: 事务ID   (2B, 大端) 主站自己编号，响应必须原样回传，用于配对请求/响应
        #   偏移 2~3: 协议ID   (2B, 大端) Modbus 固定为 0
        #   偏移 4~5: 长度     (2B, 大端) 从"单元ID"开始到帧尾的字节数
        #   偏移 6  : 单元ID   (1B)       从站地址（TCP 下多用于区分网关后面的串口设备）
        # 【之后】= PDU
        #   偏移 7  : 功能码   (1B)
        #   偏移 8~ : 数据     (功能码相关)
        # 对比 Modbus-RTU：RTU 没有这 7 字节头、末尾却有 2 字节 CRC；
        # TCP 靠 TCP 层保证完整性和顺序，所以既无 CRC 也要用 MBAP 头来界定帧边界。
        tid = struct.unpack('>H', data[0:2])[0]    # 事务ID（'H' = 16位无符号）
        pid = struct.unpack('>H', data[2:4])[0]    # 协议ID（应为 0）
        length = struct.unpack('>H', data[4:6])[0] # 长度字段
        uid = data[6]                              # 单元ID（单字节，直接索引取）
        func = data[7]                             # 功能码

        # 打印请求详情（end="" 表示不换行，让同一帧的后续信息接在同一行）
        print(f"  tid=0x{tid:04X} pid=0x{pid:04X} len={length} uid={uid} func=0x{func:02X}", end="")

        # ---------------------------------------------------------------
        # 功能码 0x03：读保持寄存器
        # 请求 PDU: [功能码][起始地址 2B][寄存器数量 2B] → 总帧长 12 字节
        # 响应 PDU: [功能码][字节数 1B][数据 n×2B]
        # ---------------------------------------------------------------
        if func == 0x03:  # 读保持寄存器
            if len(data) < 12:                     # 03 请求固定 12 字节，短了就是坏的
                print("  → 请求数据不足")
                conn.close()
                continue

            reg_addr = struct.unpack('>H', data[8:10])[0]    # 起始寄存器地址
            reg_count = struct.unpack('>H', data[10:12])[0]  # 要读几个寄存器
            print(f" addr=0x{reg_addr:04X} count={reg_count}")

            # 范围检查：Modbus 标准规定一次最多读 125 个寄存器（响应帧长上限）
            if reg_count > 125:
                # Modbus 异常响应格式: [功能码|0x80][异常码]，只需 3 字节 PDU。
                # 这里功能码 0x03 | 0x80 = 0x83，表示"该功能码出错"；
                # 异常码 0x03 = 数据值非法(ILLEGAL DATA VALUE)。
                # MBAP 的 length 字段设为 3 = uid(1) + func(1) + 异常码(1)
                err_resp = struct.pack('>HHHBBB',
                                       tid, pid, 3, uid, func | 0x80, 0x03)
                conn.send(err_resp)
                print(f"  → 异常: count={reg_count} 超过 125")
                conn.close()
                continue

            # 构造正常响应
            # 长度计算：MBAP 的 length 字段 = uid(1) + func(1) + byte_count(1) + 数据(byte_count)
            #                             = 3 + byte_count
            # 每个寄存器 2 字节，所以 byte_count = reg_count * 2
            byte_count = reg_count * 2
            resp = struct.pack('>HHHBB',
                               tid, pid, 3 + byte_count, uid, func)   # MBAP头 + 功能码
            resp += bytes([byte_count])        # 字节数（1 字节）
            for i in range(reg_count):
                # REGS.get(addr, 0)：地址不在表里就返回 0（模拟未定义寄存器读 0）
                val = REGS.get(reg_addr + i, 0)
                resp += struct.pack('>H', val & 0xFFFF)   # 每个寄存器大端 2 字节

            conn.send(resp)
            # 打印本次返回的原始值，方便和上位机日志里的十进制值对照
            values = ", ".join(
                [f"0x{REGS.get(reg_addr + i, 0):04X}" for i in range(reg_count)]
            )
            print(f"  → OK: [{values}]")

        # ---------------------------------------------------------------
        # 功能码 0x04：读输入寄存器
        # 帧结构与 0x03 完全相同，只是语义上 03 读"保持寄存器"、04 读"输入寄存器"。
        # 本项目两者都映射到同一批温湿度寄存器，所以直接复制 03 的处理逻辑。
        # ---------------------------------------------------------------
        elif func == 0x04:  # 读输入寄存器 (和 03 同样处理)
            if len(data) < 12:
                print("  → 请求数据不足")
                conn.close()
                continue

            reg_addr = struct.unpack('>H', data[8:10])[0]
            reg_count = struct.unpack('>H', data[10:12])[0]
            print(f" addr=0x{reg_addr:04X} count={reg_count}")

            byte_count = reg_count * 2
            resp = struct.pack('>HHHBB',
                               tid, pid, 3 + byte_count, uid, func)
            resp += bytes([byte_count])
            for i in range(reg_count):
                val = REGS.get(reg_addr + i, 0)
                resp += struct.pack('>H', val & 0xFFFF)

            conn.send(resp)
            print(f"  → OK (功能码 04 同 03 处理)")

        # ---------------------------------------------------------------
        # 功能码 0x06：写单个保持寄存器
        # 请求 PDU: [功能码][寄存器地址 2B][写入值 2B]
        # 响应 = 原请求原样回显（Modbus 标准规定）
        # ---------------------------------------------------------------
        elif func == 0x06:  # 写单寄存器
            if len(data) < 12:
                print("  → 请求数据不足")
                conn.close()
                continue

            reg_addr = struct.unpack('>H', data[8:10])[0]    # 目标寄存器地址
            reg_val = struct.unpack('>H', data[10:12])[0]    # 要写入的值
            print(f" addr=0x{reg_addr:04X} value=0x{reg_val:04X}")

            # 真正更新内存里的寄存器表，使上位机的写功能也能测出效果
            REGS[reg_addr] = reg_val
            print(f"  → 写入成功: REGS[0x{reg_addr:04X}] = 0x{reg_val:04X}")

            # 写单寄存器响应 = 原请求回显（把收到的前 12 字节原样发回）
            conn.send(data[:12])
            # 顺便按"物理量"打印一下，直观看到写入是否影响了温湿度
            print(f"  → 温度={REGS.get(0x0300, 0)/10:.1f}℃ 湿度={REGS.get(0x0301, 0)/10:.1f}%")

        # ---------------------------------------------------------------
        # 其它功能码：不支持 → 回异常响应
        # ---------------------------------------------------------------
        else:
            print(f"  → 不支持的功能码")
            # 异常码 0x01 = 非法功能码(ILLEGAL FUNCTION)：从站不认这个功能码
            err_resp = struct.pack('>HHHBBB',
                                   tid, pid, 3, uid, func | 0x80, 0x01)
            conn.send(err_resp)

    # recv 超时（连接建立后 5 秒没收到数据）→ 正常关闭，不视为错误
    except socket.timeout:
        print(f"  → 超时, 关闭连接")
    # 兜底：任何其它异常都打印出来，避免整个假网关因单个连接出错而崩掉
    except Exception as e:
        print(f"  → 异常: {e}")
    finally:
        # 无论上面走哪条分支，都关闭这条连接，释放文件描述符
        conn.close()

# 主循环退出（收到信号）→ 关闭监听套接字，释放端口
srv.close()
print("[假网关] 已停止")
