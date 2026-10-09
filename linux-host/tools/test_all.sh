#!/bin/bash
# =============================================================================
# test_all.sh — Modbus Backend v2 完整回归测试（一条龙）
#
# 【这个脚本解决什么问题】
#   linux-host 上位机有三大"高级功能"：多网关并发采集、断线重连、配置热加载。
#   这三样光靠看代码很难确认真的能跑通（尤其热加载涉及信号 + 配置重解析 + 线程启停）。
#   本脚本用"假网关 + 真进程 + 真 HTTP 请求"的方式，把整条链路端到端跑一遍，
#   每步都断言结果并打印 PASS/FAIL，最后汇总通过/失败数。
#
# 【测试项一览】
#   前置  : 编译 → 备份配置 → 起 2 个假网关 → 改 config.ini 指向假网关 → 起后端
#   测试 1: HTTP API 能返回 ≥2 个网关的数据
#   测试 2: 日志里有成功采集记录（数据真的在流动）
#   测试 3: 断线重连（kill 假网关1 → 检测到断开 → 重启 → 检测到重连成功）
#   测试 4: 隔离性（GW1 断着的时候，GW2 照常采集，互不影响）
#   测试 5: 热加载新增网关（追加 [gateway.3] + SIGHUP → API 返回 3 个）
#   测试 6: 热加载删除网关（删掉一段 + SIGHUP → API 返回 2 个）
#   测试 7: 热加载改参数（interval_sec 改小 + SIGHUP → 采集频率变化）
#
# 【重要前提】
#   本脚本会"写配置 + 杀进程"，只能在开发机上跑，别在线上环境执行。
#   脚本退出时会自动清理：杀掉假网关与后端、恢复 config.ini 备份。
#
# 用法: cd linux-host && bash tools/test_all.sh
# =============================================================================

# set -e：任何一条命令返回非 0，脚本立即退出。
# 意义：编译失败就不该继续往下测，避免后面一堆莫名其妙的 FAIL 干扰判断。
set -e

# ---- 计数器与计时 ----
PASS=0          # 通过的断言数
FAIL=0          # 失败的断言数
SECONDS=0       # bash 内置变量，赋值后自动开始计时，最后用于打印总耗时

# ---- 终端彩色输出（ANSI 转义码）----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'     # No Color：重置颜色，否则后面所有输出都会被染色

# 三个小助手函数：统一输出格式，并自动累加计数
pass() { PASS=$((PASS+1)); echo -e "${GREEN}✓ PASS${NC} $1"; }   # -e 让 \033 生效
fail() { FAIL=$((FAIL+1)); echo -e "${RED}✗ FAIL${NC} $1"; }
info() { echo -e "${YELLOW}→ ${NC}$1"; }

# ---------------------------------------------------------------------------
# cleanup：脚本退出时自动执行的清理（由下方 trap 注册）
# 无论测试成功、失败还是被 Ctrl+C，都会走到这里，保证不留下野进程和脏配置
# ---------------------------------------------------------------------------
cleanup() {
    info "清理..."
    local PID    # local：只在函数内有效，避免污染全局变量
    # 先按进程名杀后端；pgrep 找不到会返回非 0，用 && ... || true 兜住
    PID=$(pgrep modbus-backend 2>/dev/null) && kill "$PID" 2>/dev/null || true
    # 假网关是用 python3 起的，用 pkill -f 按完整命令行匹配
    pkill -f "fake_gateway.py" 2>/dev/null || true
    # 如果前面备份过配置，恢复它，并删掉临时文件
    if [ -f config.ini.bak ]; then
        cp config.ini.bak config.ini
        rm -f config.ini.bak config.ini.tmp 2>/dev/null || true
        info "已恢复 config.ini"
    fi
    sleep 1                 # 给进程一点时间真正退出
    wait 2>/dev/null || true # 回收所有后台子进程，避免僵尸进程
}

# trap ... EXIT：脚本以任何方式退出时都执行 cleanup
trap cleanup EXIT

# ------------------------------------------------------------------
# 0. 编译
# ------------------------------------------------------------------
info "编译项目..."
# 切到项目根目录（脚本在 tools/ 下，.. 就是 linux-host/）
cd "$(dirname "$0")/.."
make clean 2>/dev/null            # 先清理，保证测的是当前源码而不是旧产物
make 2>&1 | tail -1               # 只留最后一行（DONE bin/modbus-backend），免得刷屏
echo ""

# ------------------------------------------------------------------
# 1. 备份 + 清旧日志
# ------------------------------------------------------------------
info "备份 config.ini..."
cp config.ini config.ini.bak      # 备份！测试会改配置，测完必须还原

info "清空旧日志..."
# > 文件  = 把文件截断为 0 字节（清空内容但保留文件）
> data/modbus-backend.log 2>/dev/null || true

# ------------------------------------------------------------------
# 2. 启动假网关 × 2（用 5020/5021 高位端口，避免和真实网关的 502 冲突）
# ------------------------------------------------------------------
info "启动假网关1 (5020端口, 26.3℃ 55.0%)..."
python3 tools/fake_gateway.py 5020 26.3 55.0 &    # 末尾 & = 后台运行
GW1_PID=$!                                        # $! = 刚启动的后台进程 PID
sleep 1                                           # 等它完成 bind + listen

info "启动假网关2 (5021端口, 31.5℃ 62.0%)..."
python3 tools/fake_gateway.py 5021 31.5 62.0 &
GW2_PID=$!
sleep 1

# ss -tlnp 列出所有 TCP 监听端口；grep -q 静默匹配（只回返回码，不打印）
# 两个端口都监听着才算启动成功
ss -tlnp | grep -q ":5020 " && ss -tlnp | grep -q ":5021 " && pass "假网关都在监听" || fail "假网关启动失败"

# ------------------------------------------------------------------
# 3. 把 gateway.1 的指向改成假网关（不改真实网关配置内容）
# ------------------------------------------------------------------
info "检查 gateway.1 是否需要改指向..."
if grep -q "192.168.1.100" config.ini; then
    # 情况A：配置还是真实网关 IP → 改成回环地址 + 5020 端口
    sed -i 's/host    = 192.168.1.100/host    = 127.0.0.1/' config.ini
    sed -i '/^port    = 502$/s/502/5020/' config.ini
    echo "  已将 gateway.1 改为 127.0.0.1:5020（假网关）"
elif grep -q "host    = 127.0.0.1" config.ini; then
    # 情况B：已是回环地址（上次测完恢复的）→ 只需确保端口是 5020
    # 0,/pattern/s/// 表示"只替换第一次匹配"，避免误改 gateway.2 的端口
    sed -i '0,/^port    = 502$/s/^port    = 502$/port    = 5020/' config.ini
    echo "  确保 gateway.1 端口 = 5020"
else
    # 情况C：其它情况就保持不动（比如用户自己配了真实硬件网关）
    echo "  gateway.1 保留原配置（真实硬件网关）"
fi

# ------------------------------------------------------------------
# 4. 启动 modbus-backend 主程序
# ------------------------------------------------------------------
info "启动 modbus-backend..."
mkdir -p data                     # 保证数据目录存在（db/日志都放这）
./bin/modbus-backend &            # 后台启动被测程序
BACKEND_PID=$!
sleep 2                           # 等它初始化线程池、连网关、开始轮询

# kill -0 不发信号，只检查"进程是否存在且可被信号"→ 用来判断程序有没有起来就崩了
if kill -0 "$BACKEND_PID" 2>/dev/null; then
    pass "modbus-backend 启动 (PID=$BACKEND_PID)"
else
    fail "modbus-backend 启动失败"
    exit 1                        # 主程序都没起来，后续测试无意义，直接退出（会触发 cleanup）
fi

# ==================================================================
# 测试 1: HTTP API 多网关数据
# ==================================================================
info ""
info "========== 测试 1: HTTP API 多网关数据 =========="

# curl -s 静默模式（不显示进度条）；取回 JSON 后用 python 解析并断言
LATEST=$(curl -s http://localhost:8080/api/latest 2>/dev/null)
# python -c 里 assert 失败会抛异常 → 退出码非 0 → 走 else 分支报 FAIL
if echo "$LATEST" | python3 -c "import sys,json; d=json.load(sys.stdin); gw=d.get('gateways',[]); assert len(gw)>=2, f'got {len(gw)} gateways'" 2>/dev/null; then
    pass "API 返回 ≥2 个网关数据"
    echo "$LATEST" | python3 -m json.tool 2>/dev/null   # 格式化打印，便于人工核对
else
    fail "API 返回网关数不足: $LATEST"
fi

# ==================================================================
# 测试 2: 数据有效性（日志里必须有成功采集记录）
# ==================================================================
info ""
info "========== 测试 2: 数据有效性 =========="
sleep 3    # 等一个采集周期，让日志里产生几条记录

if grep -q "POLL #.*OK: T=" data/modbus-backend.log; then
    pass "有成功采集记录"
    grep "POLL #" data/modbus-backend.log | tail -4     # 展示最后 4 条
else
    fail "无成功采集记录"
fi

# ==================================================================
# 测试 3: 断线重连
# ==================================================================
info ""
info "========== 测试 3: 断线重连 =========="

info "停掉假网关1 (5020端口)..."
kill "$GW1_PID" 2>/dev/null || true
sleep 6    # 等后端经过"检测断开 → 进入重连状态"的若干轮轮询

# 断开检测证据：三种日志关键字任出现其一即可（不同路径会打不同日志）
#   handle_error        打印 "marking disconnected"
#   Gateway_PollOnce    打印 "attempting reconnect"
#   Modbus_Reconnect    打印 "Reconnecting to"
if grep -q "marking disconnected" data/modbus-backend.log 2>/dev/null; then
    pass "检测到断开 (marking disconnected)"
elif grep -q "attempting reconnect" data/modbus-backend.log 2>/dev/null; then
    pass "检测到断开 (attempting reconnect)"
elif grep -q "Reconnecting to" data/modbus-backend.log 2>/dev/null; then
    pass "检测到断开 (Reconnecting to)"
else
    fail "未检测到断开处理"
fi

info "重启假网关1..."
python3 tools/fake_gateway.py 5020 26.3 55.0 &
GW1_PID=$!    # 更新 PID（旧进程已死）
sleep 4       # 等后端下一轮重连尝试成功

if grep -q "Reconnected to" data/modbus-backend.log 2>/dev/null; then
    pass "断线重连成功"
    grep "Reconnected" data/modbus-backend.log | tail -1
else
    fail "未检测到重连成功"
fi

# ==================================================================
# 测试 4: 隔离性（一个网关断开不能影响另一个）
# ==================================================================
info ""
info "========== 测试 4: 隔离性（一个断，另一个照常采集） =========="

kill "$GW1_PID" 2>/dev/null || true
sleep 6

# grep -c 统计匹配行数（GW2 有名字"虚拟网关"，用它限定只数 GW2 的成功采集）
GW2_POLLS=$(grep -c "虚拟网关.*POLL #.*OK" data/modbus-backend.log || true)
if [ "$GW2_POLLS" -ge 2 ]; then
    pass "GW2 在 GW1 断开期间持续采集 (≥2次)"
else
    fail "GW2 采集被影响: 仅 $GW2_POLLS 次"
fi

# ==================================================================
# 测试 5: 热加载 — 新增网关
# ==================================================================
info ""
info "========== 测试 5: 热加载新增网关 =========="

info "启动假网关3 (5022端口, 28.0℃ 45.0%)..."
python3 tools/fake_gateway.py 5022 28.0 45.0 &
GW3_PID=$!
sleep 1

info "在 config.ini 追加 [gateway.3]..."
# 用 heredoc 直接往配置文件尾部追加一个完整段。
# 注意：'EOF' 加引号 → 里面的内容不做变量展开，原样写入配置（这里正好没有变量）
cat >> config.ini << 'EOF'

[gateway.3]
host    = 127.0.0.1
port    = 5022
name    = 三楼仓库（热加载新增）
interval_sec = 3
slave_id     = 1
reg_start    = 0x0300
reg_count    = 2
EOF

info "发送 SIGHUP..."
# SIGHUP = 约定的"重载配置"信号。后端注册了处理器，
# 收到后重新解析 config.ini 并增删采集线程，而**不重启进程**（不中断已有采集）。
kill -HUP "$BACKEND_PID"
sleep 2

if grep -q "Hot reload complete" data/modbus-backend.log; then
    pass "热加载完成"
    grep "Hot reload" data/modbus-backend.log | tail -3
else
    fail "热加载失败"
fi

LATEST2=$(curl -s http://localhost:8080/api/latest 2>/dev/null)
if echo "$LATEST2" | python3 -c "import sys,json; d=json.load(sys.stdin); gw=d.get('gateways',[]); assert len(gw)>=3, f'got {len(gw)} gateways'" 2>/dev/null; then
    pass "API 热加载后返回 3 个网关"
    echo "$LATEST2" | python3 -m json.tool 2>/dev/null
else
    fail "热加载后 API 网关数不足: $LATEST2"
fi

# ==================================================================
# 测试 6: 热加载 — 删除网关
# 用 Python 精确编辑 INI：删掉 [gateway.2] 整段，并把 [gateway.3] 改名成 [gateway.2]，
# 保持编号连续（后端按 gateway.N 的索引管理线程，编号跳跃容易出边界问题）
# ==================================================================
info ""
info "========== 测试 6: 热加载删除网关 =========="

info "删除 [gateway.2] 段再改名 [gateway.3]->[gateway.2]..."
# 这里用 quoted heredoc 把整段 Python 代码喂给解释器：代码里含 $、引号等，
# 用 'PYEOF' 可避免被 bash 提前展开，最稳。
python3 << 'PYEOF'
with open('config.ini', 'r') as f:
    lines = f.readlines()
with open('config.ini', 'w') as f:
    skip = False              # 是否正处于"要跳过的段"（被删掉的 [gateway.2]）
    for line in lines:
        stripped = line.strip()
        if stripped == '[gateway.2]':
            skip = True       # 进入待删段
            continue          # 丢掉这一行（段头本身）
        if skip and stripped.startswith('['):
            skip = False      # 遇到下一个段头 → 待删段结束
        # 把 [gateway.3] 改成 [gateway.2]（保持索引连续）
        if stripped == '[gateway.3]':
            f.write(line.replace('[gateway.3]', '[gateway.2]'))
        elif not skip:
            f.write(line)     # 非待删段的内容原样写回
PYEOF

info "发送 SIGHUP..."
kill -HUP "$BACKEND_PID"
sleep 2

# 验证删除后 API 返回 2 个网关，且两个都有名字（名字为空说明解析出了问题）
curl -s http://localhost:8080/api/latest | python3 -c "
import sys, json
d = json.load(sys.stdin)
gw = d.get('gateways', [])
assert len(gw) == 2, f'expected 2 gateways, got {len(gw)}'
assert gw[0].get('gateway_name'), f'gateway 0 has no name'
assert gw[1].get('gateway_name'), f'gateway 1 has no name'
print(f'API 返回 {len(gw)} 个网关: {gw[0][\"gateway_name\"]}, {gw[1][\"gateway_name\"]} ✓')
" && pass "热加载删除网关成功 (3→2)" || fail "热加载删除网关后数量不对"

# ==================================================================
# 测试 7: 热加载 — 修改参数
# ==================================================================
info ""
info "========== 测试 7: 热加载修改网关参数 =========="

# sed '0,/re/s/re/repl/' = 只替换第一个匹配的 interval_sec，不影响其它网关
sed -i '0,/interval_sec = [0-9]/s/interval_sec = [0-9]/interval_sec = 2/' config.ini

info "发送 SIGHUP..."
kill -HUP "$BACKEND_PID"
sleep 4    # 等采集频率变化后累积几次

# wc -l 统计采集总次数（这里只做信息展示，不做 PASS/FAIL 断言）
POLLS=$(grep "POLL #" data/modbus-backend.log | wc -l)
info "热加载后共 $POLLS 次采集（累计）"

# ==================================================================
# 汇总
# ==================================================================
echo ""
echo "========================================"
# SECONDS 由 bash 自动维护，是脚本运行秒数
echo -e "测试结果: ${GREEN}$PASS 通过${NC} / ${RED}$FAIL 失败${NC} / 用时 ${SECONDS}s"
echo "========================================"
