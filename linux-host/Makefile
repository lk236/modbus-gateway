# =============================================================================
# Makefile — Modbus Backend v2 (Linux C)
#
# 【怎么用】
#   make          # 增量编译，产出 bin/modbus-backend
#   make clean    # 删除 obj/ 和 bin/，全部重编
#   make run      # 编译后直接启动（自动建 data/ 目录）
#   make install  # 把可执行文件拷到 /usr/local/bin/
#
# 【依赖库（Ubuntu 24.04）】
#   sudo apt install gcc make libmodbus-dev libsqlite3-dev
#   三个库各自的作用：
#     libmodbus  → Modbus 协议收发（modbus_master.c 用它）
#     libsqlite3 → 本地数据库存历史数据（storage.c 用它）
#     pthread    → 多线程（glibc 自带，编译时加 -lpthread 即可）
#     librt      → POSIX 实时扩展（timerfd_create 在这里面，需要 -lrt）
#
# 【v2 相比 v1 的变化】
#   新增多网关并发采集 + HTTP API，因此需要 pthread；
#   单网关版本不需要线程，v1 的 LDFLAGS 里没有 -lpthread。
# =============================================================================

CC      := gcc                       # 编译器。:= 是立即展开赋值（区别于 = 的递归展开）
CFLAGS  := -Wall -Wextra -O2 -g -MMD -MP \
           -D_GNU_SOURCE \
           -Wno-format-truncation \
           -Iinclude
# 各编译选项逐个说明：
#   -Wall -Wextra      打开警告（-Wextra 比 -Wall 更严格）。能提前发现很多低级错误
#   -O2                优化级别 2，兼顾速度与体积
#   -g                 带调试信息，方便 gdb 看变量、定位崩溃行号
#   -MMD -MP           自动生成头文件依赖（.d 文件）。作用：改了某个 .h，
#                      依赖它的 .c 会被自动重编。没有它就得每次 make clean
#   -D_GNU_SOURCE      开 glibc 的扩展接口（如 accept4、pthread_setname_np 等）
#   -Wno-format-truncation  关掉 snprintf 截断告警（拼路径时已知不会截断，报了太吵）
#   -Iinclude          头文件搜索路径加 include/（这样 #include "ota_crc.h" 能找到）
# 行尾的 \ 是续行符，把多行拼成一条逻辑行（\ 必须是行尾最后一个字符，后面不能再有空格）

LDFLAGS := -lmodbus -lsqlite3 -lrt -lpthread
# 链接选项：-lxxx 表示链接 libxxx。注意顺序——被依赖的库要放在后面，
# 所以调换这几个的顺序可能导致 undefined reference（GNU ld 的单遍解析特性）。

# ---- 目录定义 ----
SRCDIR  := src        # 源码目录
OBJDIR  := obj        # 中间产物 .o 目录
BINDIR  := bin        # 最终可执行文件目录
WEBDIR  := web        # 前端静态文件目录（dashboard.html 等）

# wildcard 展开通配符，得到 src/ 下所有 .c 文件列表
SRCS    := $(wildcard $(SRCDIR)/*.c)
# ota_crc.c 由下方独立规则 $(OBJDIR)/ota_crc.o 编译，不加入 SRCS
# （原因：它在 include/ 而非 src/，通配符扫不到，所以单独写一条规则）
OBJS    := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(SRCS))
# patsubst 做"模式替换"：把 src/foo.c 变成 obj/foo.o。
# 得到的就是所有需要编译的目标文件列表。
OBJS    += $(OBJDIR)/ota_crc.o   # += 追加 ota_crc.o

DEPS    := $(OBJS:.o=.d)         # 把每个 .o 换成同名 .d（依赖文件列表）
TARGET  := $(BINDIR)/modbus-backend   # 最终产物路径

# .PHONY 声明"假目标"：这些名字不是文件，只是动作名。
# 不加的话，如果目录下恰好有个叫 clean 的文件，make clean 就会被认为"已是最新"而不执行。
.PHONY: all clean install run

# 默认目标（make 不带参数时的目标）：构建最终可执行文件
all: $(TARGET)

# ---------------------------------------------------------------------------
# 链接：把所有 .o 合成可执行文件
# $@ = 目标名（bin/modbus-backend），$^ = 所有依赖（所有 .o 文件）
# ---------------------------------------------------------------------------
$(TARGET): $(OBJS)
	mkdir -p $(BINDIR)                                     # 保证 bin/ 存在
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "  DONE  $@"                                     # @ 前缀 = 不打印这条命令本身，只打印结果

# ---------------------------------------------------------------------------
# 编译：每个 src/*.c → obj/*.o
# $< = 第一个依赖（对应的 .c 文件）
# ---------------------------------------------------------------------------
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<                            # -c = 只编译成 .o，不链接
	@echo "  CC    $@"

# 单独为 include/ 下的 ota_crc.c 写一条规则（路径与上面不同，无法共用模式规则）
$(OBJDIR)/ota_crc.o: include/ota_crc.c
	mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c -o $@ $<
	@echo "  CC    $@"

# ---------------------------------------------------------------------------
# 清理：删掉全部编译产物，回到"干净"状态
# ---------------------------------------------------------------------------
clean:
	rm -rf $(OBJDIR) $(BINDIR)
	@echo "  CLEAN"

# ---------------------------------------------------------------------------
# 安装：把可执行文件复制到系统 PATH 目录（systemd 服务单元用的就是这个路径）
# ---------------------------------------------------------------------------
install: $(TARGET)
	cp $(TARGET) /usr/local/bin/

# ---------------------------------------------------------------------------
# 运行：先编译，再建数据目录并启动
# ---------------------------------------------------------------------------
run: $(TARGET)
	@echo "  Starting Modbus Backend v2..."
	@mkdir -p data                    # 数据库和日志都写在 data/
	./$(TARGET)

# 引入所有 .d 文件：它们是 -MMD 自动生成的、记录"每个 .o 依赖哪些 .h"的规则。
# 加 -include（前面带横线）表示：文件不存在时不要报错（首次编译时确实还没有）。
-include $(DEPS)
