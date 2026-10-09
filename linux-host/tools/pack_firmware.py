#!/usr/bin/env python3
# =============================================================================
# pack_firmware.py — 固件打包脚本（把 Keil 编出的 .bin 加工成 .ota）
#
# 【在整个 OTA 链路里的位置】
#   Keil 编译 → modbus-gateway.bin
#        │
#        ▼   ← 本脚本在这里
#   modbus-gateway_v1.0.ota  (16 字节 Header + 原始固件数据)
#        │
#        ▼
#   linux-host 上位机读取 → 通过 TCP:506 按 512 字节分块下发给 Bootloader
#
# 【为什么要打包，不能直接下发 .bin】
#   1. 下位机需要先知道"要收多少字节""应该校验出什么 CRC32"，才能在收完时判断
#      是否正确、该不该写 Flash。这些元信息没地方放，所以塞进文件头。
#   2. 文件头固定 16 字节 + magic，下位机一眼就能辨认"这是不是我们的固件包"，
#      避免误把别的文件当固件烧进去。
#
# 【产出的 .ota 文件布局】
#   ┌──────────────────────┬────────────────────────────────────────────┐
#   │ FirmwareHeader 16 字节│ 纯固件数据 (已半字对齐, 奇数补 0xFF)         │
#   │ magic/version/size/crc│ size 字段即这里的数据长度                    │
#   └──────────────────────┴────────────────────────────────────────────┘
#   Header 用大端序（网络字节序 >），与下位机 ota_protocol_def.h 里的
#   FirmwareHeader 结构体一一对应 —— 改这里就必须同步改那头，否则解析错位。
#
# 用法:
#   python pack_firmware.py modbus-gateway.bin modbus-gateway_v1.0.ota --version 0x0100
#
# 环境:
#   Python 3.6+（只用标准库，无需 pip 安装任何东西）
# =============================================================================

import struct      # pack/unpack: 把 Python 整数按指定字节序打包成二进制
import sys         # sys.exit(): 出错时返回非 0 退出码，让 make/CI 能感知失败
import os          # os.path.exists(): 检查输入文件是否存在
import argparse    # 命令行参数解析（自带 --help，比手写 sys.argv 更友好）
import zlib        # zlib.crc32(): C 实现的标准 CRC32，比纯 Python 循环快几十倍


def crc32_zip(data: bytes) -> int:
    """计算 CRC32-ZIP（直接用 zlib 内置实现）

    为什么叫 "ZIP"：这个 CRC 的多项式 0x04C11DB7、初值 0xFFFFFFFF、结果异或
    0xFFFFFFFF 正是 ZIP/PNG/以太网使用的标准 CRC32，也是 STM32 硬件 CRC 单元
    的默认配置。下位机 bsp_w25q64.c 里的 CRC32 与 ota_crc.c 里的 CRC32_ZIP 都
    是同一套参数 —— 三处必须完全一致，否则校验永远失败。

    zlib.crc32 返回的是"已异或"后的值，但 Python 中它是 C long，可能带符号位，
    所以最后再用 & 0xFFFFFFFF 截成标准的 32 位无符号数。
    """
    return zlib.crc32(data) & 0xFFFFFFFF


# ---- 与下位机固件头严格对齐的常量（三处常量必须一致） ----
FW_HEADER_MAGIC = 0x504A3142  # "PJ1B" 的 ASCII: 0x50='P' 0x4A='J' 0x31='1' 0x42='B'
                              # 与 ota_protocol_def.h 的 FW_HEADER_MAGIC 对应
FW_HEADER_SIZE = 16           # 字节。4(magic) + 4(version) + 4(size) + 4(crc32)
FW_MIN_SIZE = 1024            # 1KB。太小的文件肯定是编错了（或选错文件），提前拦截


def pack_firmware(bin_path: str, ota_path: str, version: int):
    """打包固件：读 bin → 校验 → 对齐 → 算 CRC → 加头 → 写 ota → 回读自检"""

    # ---- 1. 读取 .bin 原始固件 ----
    with open(bin_path, 'rb') as f:   # 'rb' = 二进制读，绝不能漏 b（否则 Windows 下会坏数据）
        fw_data = f.read()

    original_size = len(fw_data)      # 先把原始大小记下来，后面打印对比用
    print(f"Read {bin_path}: {original_size} bytes")

    # ---- 2. 校验最小大小 ----
    if original_size < FW_MIN_SIZE:   # 小于 1KB 基本可以断定编错了 / 传错文件
        print(f"ERROR: Firmware too small ({original_size} < {FW_MIN_SIZE})")
        sys.exit(1)                   # 非 0 退出码 → 调用方可据此中断流程

    # ---- 3. 半字对齐（奇数长度补 0xFF） ----
    # 下位机是按"半字(16bit)"往 Flash 写的（STM32 的 Flash 编程单位是 16 位），
    # 所以长度必须是偶数。补 0xFF 而不是 0x00，是因为
    #   a) 0xFF 是 Flash 擦除后的空白值，更自然；
    #   b) 万一补位被误当代码执行，0xFFFF 是 ARM 的非法指令，会立刻 HardFault，
    #      比 0x0000（合法指令）"静默跑飞"更容易被发现。
    if len(fw_data) % 2 != 0:
        fw_data += b'\xFF'
        print(f"Padded with 0xFF to {len(fw_data)} bytes (half-word align)")

    # ---- 4. 计算 CRC32 ----
    # 注意：算的是"补位之后"的完整固件，因为下位机收到的就是补位后的数据；
    # 如果这里按补位前的长度算，下位机按补位后校验必然对不上。
    fw_crc32 = crc32_zip(fw_data)
    print(f"CRC32: 0x{fw_crc32:08X}")

    # ---- 5. 构建 FirmwareHeader（16 字节） ----
    # struct.pack 的格式串 '>4sIII' 拆开看：
    #   >  = 大端序(big-endian)，即网络字节序。必须与下位机结构体保持一致
    #   4s = 4 字节字符串（不可用 'I'，否则 0x504A3142 会被大端拆成 50 4A 31 42
    #        也行，但用 4s 直接写 b'PJ1B' 语义更清晰）
    #   I  = unsigned int (4 字节) ×3 → version / size / crc32
    # 合计 4 + 4 + 4 + 4 = 16 字节，与 FW_HEADER_SIZE 吻合
    header = struct.pack('>4sIII',
                         b'PJ1B',                    # magic
                         version & 0xFFFFFFFF,       # 版本号，如 0x0100 = v1.0
                         len(fw_data) & 0xFFFFFFFF,  # 固件大小（补位后的长度）
                         fw_crc32 & 0xFFFFFFFF)      # 固件 CRC32

    # ---- 6. 写出 .ota 文件（先头后体，顺序不能反） ----
    with open(ota_path, 'wb') as f:
        f.write(header)     # 前 16 字节：Header
        f.write(fw_data)    # 紧随其后：固件正文

    total_size = FW_HEADER_SIZE + len(fw_data)
    print(f"Written {ota_path}: {total_size} bytes (header {FW_HEADER_SIZE} + data {len(fw_data)})")
    print(f"Version: 0x{version:04X}")

    # ---- 7. 回读自检（写完立刻读回来验一遍，防止磁盘满/写坏等静默错误） ----
    # 这一步的重要性：OTA 一旦把一个坏包下发到现场设备，设备可能直接变砖。
    # 所以"写出去"不等于"写对了"，必须读回来核对。
    with open(ota_path, 'rb') as f:
        verify_header = f.read(FW_HEADER_SIZE)   # 只读 16 字节头
        verify_data = f.read()                   # 剩下的全是固件正文

    # 按同样的格式串解包读回来的头，逐个字段比对
    v_magic, v_ver, v_size, v_crc = struct.unpack('>4sIII', verify_header)
    assert v_magic == b'PJ1B', "Header magic verification failed"      # magic 写错 → 全盘错位
    assert v_size == len(fw_data), "Header size mismatch"              # 长度字段对不上
    assert v_crc == fw_crc32, "Header CRC mismatch"                    # CRC 字段对不上
    assert verify_data == fw_data, "Firmware data mismatch"            # 正文一个字节都不能差
    print("Self-verification: PASS")

    return 0    # 0 = 成功


def main():
    # argparse：自动生成 --help，并在参数缺失时报错用法，比裸写 sys.argv 稳
    parser = argparse.ArgumentParser(
        description='Pack .bin firmware into .ota format with header')
    parser.add_argument('input', help='Input .bin file')            # 位置参数1：输入
    parser.add_argument('output', help='Output .ota file')          # 位置参数2：输出
    # type=lambda x: int(x, 0) 让 --version 同时接受 0x0100(十六进制) 和 4096(十进制)
    # int(x, 0) 的第二个参数 0 表示"按前缀自动识别进制"
    parser.add_argument('--version', '-v', type=lambda x: int(x, 0),
                        default=0x0100,                          # 不传时默认 v1.0
                        help='Firmware version (hex, e.g. 0x0100)')

    args = parser.parse_args()

    # 输入文件不存在就直接报错退出，避免后面 open() 抛异常时堆栈信息干扰用户
    if not os.path.exists(args.input):
        print(f"ERROR: Input file not found: {args.input}")
        sys.exit(1)

    return pack_firmware(args.input, args.output, args.version)


# 仅当本文件被"直接运行"时才执行 main()；
# 被 import 时不会自动跑 —— 这样其它脚本可以复用 pack_firmware() 函数
if __name__ == '__main__':
    main()
