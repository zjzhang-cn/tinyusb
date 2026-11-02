# nanoCH32V203 开发手册（TinyUSB）

面向第一次上手这块板的人或 AI agent：编译、烧录、监控、调试，以及这块板特有的限制。
文中命令在本机（Linux + WCH-LinkE + GCC12/RISC-V-Embedded）全部实测过；**未实测的会显式标注**。

- 板卡：nanoCH32V203（MuseLab/wuxx），<https://github.com/wuxx/nanoCH32V203>
- BSP 目录：`hw/bsp/ch32v20x/boards/nanoch32v203/`
- 在 TinyUSB 支持列表里的条目：`docs/reference/boards.rst` → `nanoch32v203  nanoCH32V203  ch32v20x`

---

## 1. 板卡速览

| 项目 | 规格 |
|---|---|
| MCU | CH32V203C8T6，RV32IMAC（QingKe V4B），最高 144 MHz |
| Flash | 64 KB（wlink 读到 `FlashSize(64KB)`；本仓 make 路径声明的 224 KB 是假的，见 §9.3） |
| RAM | 20 KB（无外部 RAM） |
| 系统时钟 | 144 MHz，来自板载 8 MHz HSE（`SYSCLK_FREQ_144MHz_HSE`）；USB 时钟 = PLL/3 |
| USB 口 | 两个 Type-C：**USB1 = 仅 device**（Port0 / FSDev）、**USB2 = host 或 device**（Port1 / USBFS） |
| 调试口 | SDI 两线（WCH-LinkE），同时可桥接 USART1 |
| 串口 | USART1：TX = PA9，RX = PA10，**115200 8N1** |
| LED | PA15，**低电平点亮**，开漏输出 |
| 按键 | RST、BOOT（BOOT 用于进 ISP bootloader，不是 GPIO 功能键） |
| 其他板载 | 8 MHz + 32.768 kHz 晶振、FPC-12P LCD 接口、SD 卡槽（SPI） |

### 1.1 BSP 暴露的能力和缺口

| BSP API | 状态 |
|---|---|
| `board_led_write()` | 可用（PA15） |
| `board_button_read()` | **恒返回 false**（无 GPIO 按键接进 BSP） |
| `board_reset_to_bootloader()` | **空实现**（要进 ISP 请手动按住 BOOT 再按 RST） |
| `board_init()` | 144 MHz 时钟、LED、USART1@115200、USB 时钟与两个 USB 外设时钟 |

### 1.2 两条 USB 通道的分工（很重要）

| 物理口 | TinyUSB 端口 | 驱动 | 能力 |
|---|---|---|---|
| USB1 | Port0 | FSDev（`dcd_stm32_fsdev` + WCH 适配） | 只做 device |
| USB2 | Port1 | USBFS（`hcd_ch32_usbfs` / `dcd_ch32_usbfs`） | host（默认）或 device（`RHPORT_DEVICE=1`） |

所以：**device 示例插 USB1；host 示例必须插 USB2；dual 示例（同时 host+device）必须两个口都用**。
详见 §7。

---

## 2. 环境准备

本机环境（换机器时按需改路径）：

| 项 | 值 |
|---|---|
| 工具链 | `~/work/ch32/ch32-docker/Toolchain/RISC-V-Embedded-GCC12/bin/riscv-wch-elf-`（另有 GCC15：`RISC-V-Embedded-GCC15/bin/riscv32-wch-elf-`） |
| 烧录 | `wlink`（`~/work/ch32/ch32-docker/wlink-linux-x64/wlink`） |
| 调试 | WCH fork 的 `openocd`（`~/work/ch32/ch32-docker/OpenOCD/OpenOCD/bin/openocd`） |
| ISP 兜底 | `~/work/ch32/ch32-docker/wchisp-linux-x64/wchisp`（v0.3.0） |
| 环境脚本 | 仓库上一级的 `../ch32.env` |

```bash
cd /home/gehc/work/ch32/tinyusb
source ../ch32.env
```

脚本只设三样：`CROSS_COMPILE`（带尾随 `-` 的绝对前缀）、`OPENOCD_WCH`、`PATH`（追加 wlink 目录）。
**它不会把工具链 `bin/` 加进 `PATH`**——这不影响 make（直接用 `CROSS_COMPILE` 前缀调编译器）和 cmake（把同一个值用 `-DCROSS_COMPILE=` 传进去，见 §3.2），但直接敲 `riscv-wch-elf-gcc` / `gdb` / `objdump` 时会找不到，需要自己加：

```bash
export PATH="$(dirname "$(command -v "${CROSS_COMPILE}gcc")"):$PATH"
```

依赖：CH32 的 SDK 已在树内（`hw/mcu/wch/ch32v20x/EVT`），驱动在 `src/portable/wch/`，**正常不需要拉依赖**。缺了才跑：

```bash
python3 tools/get_deps.py CH32V20X
```

---

## 3. 编译

### 3.1 单个示例（make，最快）

```bash
cd examples/device/cdc_msc
make BOARD=nanoch32v203 all
```

产物：`_build/nanoch32v203/<项目>.elf` / `.bin` / `.hex` / `.elf.map`。

### 3.2 全部示例（cmake，用于回归 / 给 HIL 用）

```bash
cd examples
source ../../ch32.env
cmake -B cmake-build-nanoch32v203 -DBOARD=nanoch32v203 \
      -DCROSS_COMPILE="$CROSS_COMPILE" \
      -DNO_WARN_RWX_SEGMENTS_SUPPORTED=OFF \
      -G Ninja -DCMAKE_BUILD_TYPE=MinSizeRel .
cmake --build cmake-build-nanoch32v203 -j 8
```

实测：**34 个示例全部链接成功，0 失败**（device 25 / dual 2 / host 7）。

构建目录名必须是 `cmake-build-<board>`——HIL 测试按这个约定找固件。

两个参数不能省：

| 参数 | 原因 |
|---|---|
| `-DCROSS_COMPILE=...` | CMake 不读同名环境变量，不传会退回 `riscv-none-elf-`（xpack），和 make 路径不是同一套工具链 |
| `-DNO_WARN_RWX_SEGMENTS_SUPPORTED=OFF` | `hw/bsp/family_support.cmake` 在 GCC ≥ 12 时会加 `LINKER:--no-warn-rwx-segments`，WCH GCC12 自带的 ld **不认识**这个选项，链接直接失败。用 GCC15 时不需要 |

注意 cmake 路径**不带 `-flto`**（只有 `family.mk` 带），所以同样的代码 cmake 产物体积更大。比较体积时不要混用两条路径。

### 3.3 常用构建选项

| 选项（make / cmake） | 作用 |
|---|---|
| `LOG=2` / `-DLOG=2` | `CFG_TUSB_DEBUG=2`，协议栈日志走 printf（USART1）。还有 `LOG=1`（仅错误）、`LOG=3`（含数据） |
| `DEBUG=1` / `-DCMAKE_BUILD_TYPE=Debug` | `-Og -g` |
| `RHPORT_DEVICE=1` / `-DRHPORT_DEVICE=1` | device 从 Port0(FSDev) 切到 Port1(USBFS)；host 固定 Port1（见 §7） |
| `LOGGER=rtt` | **这块板不能用**，见 §5.5 |

### 3.4 什么时候必须删 build 目录

make 这套构建 **不跟踪编译选项变化**，也不跟踪工具链切换：

```bash
rm -rf _build/nanoch32v203      # 改 LOG/DEBUG/RHPORT 之后
rm -rf cmake-build-nanoch32v203 # cmake 侧同理
```

两个真实踩到的症状：

- 加了 `LOG=2` 却“编译成功”但 size 一点没变 → 旧对象被复用。
- `lto1: fatal error: bytecode stream in file ... generated with LTO version 15.1 instead of the expected 12.0` → 该目录里残留着 GCC15 编的对象，用 GCC12 链接。删目录重编即可。

### 3.5 体积参考（64 KB Flash / 20 KB RAM）

| 示例 | text | data | bss | Flash 占用 | RAM 占用 |
|---|---|---|---|---|---|
| `device/cdc_msc` | 13856 | 8252 | 3540 | 22108 B（34%） | 11792 B（58%） |
| `host/hid_controller` | 16460 | 132 | 4228 | 16592 B（25%） | 4360 B（21%） |
| `dual/host_hid_to_device_cdc` | 30268 | 216 | 8280 | 30484 B（47%） | 8496 B（42%） |

RAM = `data + bss`（栈默认 2 KB，也算在 20 KB 里）。**RAM 是最先撞墙的资源**，加缓冲区前先看 `data + bss`。

---

## 4. 烧录

### 4.1 wlink（默认，`make flash` 走这条）

```bash
make BOARD=nanoch32v203 flash
```

```
Connected to WCH-Link v2.19(v39) (WCH-LinkE-CH32V305)
Attached chip: CH32V20X [CH32V203C8T6] (ChipID: 0x20310500)
FlashSize(64KB) UID(...)
Flashing 30484 bytes to 0x08000000 ... Flash done ... Now reset...
```

### 4.2 openocd-wch（备选）

`hw/bsp/ch32v20x/family.mk` 默认把这目标注释掉了，需要显式指定：

```bash
make BOARD=nanoch32v203 flash-openocd-wch OPENOCD="$OPENOCD_WCH"
```

结束时打印 `Error: Unsupported register (enum gdb_regno)(8)`，但 `make` 退出码为 0、镜像已写入并复位——不是失败。
WCH 的 code flash 不可回读，所以这条路径 **不做 verify**。

### 4.3 ISP 兜底（WCH 内置 bootloader，未在本机实测）

WCH-Link 不可用时可以走芯片自带的 USB ISP：

1. 按住板上的 **BOOT**，按一下 **RST**，再松开 BOOT → 芯片进 bootloader，USB1 口出现 WCH 的 ISP 设备；
2. `wchisp flash <file.bin>`（本机有 `wchisp 0.3.0`），或用 WCHISPTool。

### 4.4 探针互斥

`wlink` 和 `openocd` **不能同时**占用 WCH-Link。调试结束要烧录时先 `pkill openocd`。
现象是 `wlink` 报找不到设备 / openocd 报 `wlink_init` 失败。

---

## 5. 监控（日志与数据通道）

### 5.1 UART 是唯一的 printf 出口

`printf()` / `TU_LOG*()` 全部走 **USART1（PA9/PA10，115200 8N1）**。两种接法：

- **本机台架**：WCH-LinkE 的 CDC 已经接到板子的 USART1，PC 上就是一个 `/dev/ttyACMx`（`lsusb` 里 `1a86:8010`，`ID_DRIVER=cdc_acm`）。实测能收到复位横幅：
  ```
  TinyUSB Host HID <-> Device CDC Example
  ```
- 裸板：PA9/PA10 接任意 USB-UART，115200。

UART 是**无缓冲**的：只有正在读的时候才收得到，错过就没了。看启动日志要“先开抓、再复位”：

```bash
stty -F /dev/ttyACM0 raw 115200
timeout 10 bash -c 'while :; do cat /dev/ttyACM0 >> /tmp/uart.log; sleep 0.2; done'
# 另一个终端：wlink reset
```

### 5.2 日志级别

```bash
rm -rf _build/nanoch32v203 && make BOARD=nanoch32v203 LOG=2 all && make BOARD=nanoch32v203 flash
```

`LOG=2` 会打印枚举全过程（Setup/Descriptor/EP 传输），排查主机侧时非常有用；代价是代码从 ~30 KB 涨到 ~41 KB，输出也会拖慢时序（CDC 高吞吐场景下可能出现丢行）。

### 5.3 应用自己的数据通道

示例通常还会开一个 USB 设备口（CDC / MSC / HID）。它和 UART 是两条独立通道：

- CDC：PC 上表现为 `/dev/ttyACMx`（`lsusb` 里 `cafe:40xx`，TinyUSB 的 VID），只能看到应用写的数据。
- 不要写死编号：WCH-Link 和板子 CDC 都是 `cdc_acm`，`/dev/ttyACM0/1/2` 随插拔顺序变化。用 `udevadm info -q property -n /dev/ttyACMx | grep ID_VENDOR_ID` 认。

抓应用数据（复位会重建 tty 节点，所以循环重开而不是 `cat` 一次）：

```bash
stty -F /dev/ttyACM1 raw 115200
timeout 15 bash -c 'while :; do cat /dev/ttyACM1 >> /tmp/cdc.log; sleep 0.2; done'
```

`stty raw` 是为了不让 termios 把 `\r` 改成 `\n`；怀疑内容被终端改过就直接 `xxd /tmp/cdc.log`。

### 5.4 用 GDB 读状态代替打印

比加 printf 更快的手段见 §6.3：直接 `p _hidh_itf[0]` / `p _cdcd_itf[0].rx_stream.ff.wr_idx` 之类。

### 5.5 RTT 不可用

`LOGGER=rtt` 在这块板上 **会 trap**：WCH QingKe 内核上通用的 RISC-V `SEGGER_RTT_LOCK` 会读 `mstatus` CSR 导致 `mcause=2`，而且 SDI 本身也不支持实时流。所以这块板只有 UART 一条日志通道（见 `.claude/skills/rtt/boards.md` 的 Excluded 列表）。

---

## 6. 调试（GDB）

### 6.1 起 gdbserver

```bash
cd /home/gehc/work/ch32/tinyusb
source ../ch32.env
"$OPENOCD_WCH" -f hw/bsp/ch32v20x/wch-riscv.cfg
```

关键输出：

```
Info : WCH-LinkE  mode:RV version 2.19
Info : clock speed 6000 kHz
Info : [wch_riscv.cpu.0] Examined RISC-V core; found 1 harts
Info : [wch_riscv.cpu.0]  XLEN=32, misa=0x40901105
Info : Listening on port 3333 for gdb connections
```

### 6.2 连接

```bash
GDB=~/work/ch32/ch32-docker/Toolchain/RISC-V-Embedded-GCC12/bin/riscv-wch-elf-gdb
$GDB -q examples/device/cdc_msc/_build/nanoch32v203/cdc_msc.elf
```

```gdb
target extended-remote localhost:3333
monitor halt          # 就地停住（不断复位）
```

`monitor <cmd>` 转发给 openocd，常用 `halt`、`resume`、`reset halt`、`reset`。

### 6.3 常用操作（实测）

```gdb
bt                          # 空闲时应停在 tuh_task_ext()/osal_queue_receive()
info registers pc sp gp
p blink_interval_ms         # 普通全局/静态变量都能看（make 带 -ggdb）
p _hidh_itf[0]              # HID 主机实例：daddr / ep_in / mounted / itf_protocol / epin_size
p _usbh_devices[0].connected  # 注意按 daddr-1 索引，devices[0] 才是地址 1
p _cdcd_itf[0].rx_stream.ff.wr_idx   # CDC 收包计数，PC 每写一次就前进
```

- 断点下在 flash 里会自动变成**硬件断点**（`Note: automatically using hardware breakpoints for read-only addresses`），数量有限（一般 4 个），别下太多。
- 实测 `break process_mouse_report` 立即命中，`bt` 能看到完整链路：
  `tuh_task → tuh_task_ext → hidh_xfer_cb → tuh_hid_report_received_cb → process_mouse_report`。
- 想看某个函数是否被调用、调用频率，用断点 + `ignore`/`commands` 比加打印干净。

### 6.4 收尾（别用 `continue &` + `quit`）

这样退会报 `Cannot execute this command while the target is running`（这个 WCH openocd 的脾气）。正确姿势：

```gdb
monitor reset halt
monitor resume
detach
quit
```

只复位重跑不退出也行：`monitor reset halt` → `monitor resume`。复位后 USB 重新枚举，PC 侧设备号会变（`lsusb` 的 `Device N`），`/dev/ttyACMx` 会重建。

### 6.5 VSCode / cortex-debug

`.vscode/launch.json` 里已有模板：`servertype: openocd`、`serverpath` 指 WCH openocd、`gdbPath` 指 `riscv-wch-elf-gdb`、`gdbTarget: localhost:3333`、`postLaunchCommands: ["monitor reset halt","load"]`。把 `executable` 换成目标 ELF 即可。

---

## 7. USB 双口与 RHPORT 配置

`hw/bsp/ch32v20x/family.mk` / `family.cmake` 的规则：

```make
PORT ?= 0
ifeq ($(PORT),0)
  CFLAGS += -DCFG_TUD_WCH_USBIP_FSDEV=1    # device 用 FSDev（Port0）
  CFLAGS += -DCFG_TUH_WCH_USBIP_USBFS=1    # host 用 USBFS（Port1）
else
  CFLAGS += -DCFG_TUD_WCH_USBIP_USBFS=1    # device 改用 USBFS（Port1），此时没有 host
endif
```

| 目标形态 | 配置 | 应用里怎么写 | 硬件接法 |
|---|---|---|---|
| 只做 device（默认） | `RHPORT_DEVICE=0` | `tusb_init(0, DEVICE)` | 线插 **USB1** |
| device 走 USBFS | `RHPORT_DEVICE=1` | `tusb_init(1, DEVICE)` | 线插 **USB2** |
| only host | `RHPORT_DEVICE=0` + host 用 Port1 | `tusb_init(1, HOST)` | 设备插 **USB2** |
| dual（host + device 同时） | `RHPORT_DEVICE=0` | device `tusb_init(0,…)` + host `tusb_init(1,…)` | USB1 接 PC，USB2 接 HID/设备 |

要点：

- **host 只能在 Port1（USB2）**，没有第二个 host 控制器。
- dual 示例里 device 必须留在 Port0，否则 Port1 被 device 占走，host 起不来。
- 例子里一般写成 `BOARD_TUD_RHPORT` / `BOARD_TUH_RHPORT`（由 BSP 定义），不要硬编码 0/1。
- HIL 配置里这块板的两个变体就叫 `nanoch32v203-fsdev` 和 `nanoch32v203-usbfs`（对应 `RHPORT_DEVICE=0/1`）。

---

## 8. 该选哪个示例

全量构建通过（34 个），按需挑：

| 需求 | 示例 | 插哪个口 |
|---|---|---|
| 冒烟测试 / CDC+MSC | `device/cdc_msc` | USB1 |
| HID 键鼠 | `device/hid_composite` | USB1 |
| 音频 | `device/audio_test`、`device/uac2_headset` | USB1 |
| 主机读设备（HID/MSC/CDC/HUB） | `host/hid_controller`、`host/msc_file_explorer`、`host/cdc_msc_hid` | 设备插 USB2 |
| 同时做 host 和 device | `dual/host_hid_to_device_cdc`、`dual/host_info_to_device_cdc` | USB1→PC，USB2→被枚举设备 |
| **两个口都做 device**（HID + CDC） | `dual/hid_cdc_bridge` | USB1→HID 主机，USB2→CDC 主机 |
| DCD 严格回归 | `device/usbtest` | USB1→Linux 主机（跑 `testusb`） |

已知板级行为差异：

- `device/msc_dual_lun` 在**所有**构建里都被板级定义 `CFG_EXAMPLE_MSC_DUAL_READONLY` 改成只读（为了在 20 KB RAM 下跑得动），见 `examples/device/msc_dual_lun/src/msc_disk_dual.c`。
- host 示例对 Low Speed 设备（老键鼠）也支持，实测 Dell MS116 鼠标（`413c:301a`，LS，boot 协议）能正常枚举取报文。
- **两个设备口不能都用 TinyUSB**：设备栈是单实例（§7），`dual/hid_cdc_bridge` 的做法是 TinyUSB 跑 Port0 的 HID + WCH 官方 USBFS 驱动跑 Port1 的 CDC，两者通过 BSP 的弱钩子 `usbfs_isr_hook()` 共享 `USBFS_IRQHandler`。

---

## 9. 已知问题 / FAQ

### 9.1 编译

| 现象 | 原因与处理 |
|---|---|
| `riscv-wch-elf-gcc: command not found`（直接敲命令时） | `ch32.env` 不把工具链放进 PATH；make/cmake 不受影响，手工用工具时按 §2 加 PATH |
| CMake 链接报 `unrecognized option '--no-warn-rwx-segments'` | 加 `-DNO_WARN_RWX_SEGMENTS_SUPPORTED=OFF` |
| `lto1: fatal error: bytecode stream ... LTO version 15.1 instead of 12.0` | build 目录里混了另一套工具链的对象；`rm -rf` 对应 build 目录 |
| 改了 `LOG=2` 但产物没变 | make 不跟踪 flag 变化；先删 `_build/<board>` |
| `objcopy` 报错 | 全量构建里偶发的非关键错误；单独重建该示例即可 |

### 9.2 烧录 / 运行

| 现象 | 原因与处理 |
|---|---|
| `wlink` 找不到设备 | 探针被 openocd 占用（§4.4）；或 `lsusb` 里没有 `1a86:8010`（换线/换口） |
| `flash-openocd-wch` 末尾报 `Unsupported register` | 正常，退出码 0，镜像已写入 |
| 烧完 device 不枚举 | 线插错口：device 只走 **USB1**（`RHPORT_DEVICE=0`）；或时钟配置不是 48/96/144 MHz 之一（`board_init` 里有 `TU_ASSERT(0)`） |
| host 口不认设备 | host 只能是 **USB2**（Port1）；确认没把 `RHPORT_DEVICE=1` 带进 dual 构建 |
| CDC 上一个字节都没有 | CDC 只在应用有数据时才输出；先看 UART 横幅确认固件在跑，再让被测事件发生（按键/移动鼠标） |
| 看起来“卡死” | 先 `monitor halt` + `bt` 看 PC 在哪；空闲停在 `osal_queue_receive` 是正常的 |

### 9.3 资源与板级定义的坑

1. **make 的 Flash 上限是 224 KB，芯片只有 64 KB**：`board.mk` 传 `__FLASH_SIZE=224K`，链接器不会拦；cmake 走 `board.cmake` 的 `LD_FLASH_SIZE 64K` 才是对的。自查体积时按 64 KB 判断。
2. **RAM 只有 20 KB**：`data + bss` 加栈（默认 2 KB）就是全部；`CFG_TUD_*_BUFSIZE` 调大很容易爆。真需要可用 `-Wl,--defsym=__STACK_SIZE=...` 调整栈。
3. **`board_button_read()` 恒 false**：不要指望示例里的按键功能。
4. **`board_reset_to_bootloader()` 是空函数**：进 ISP 要手按 BOOT+RST。
5. **本仓 `examples/build_system/make/toolchain/gcc_common.mk` 有一处本地改动**（注释掉 `-Wl,--no-warn-rwx-segment`），WCH 工具链需要它，别还原。

---

## 10. 参考

| 内容 | 位置 |
|---|---|
| BSP | `hw/bsp/ch32v20x/`（`family.c`/`family.mk`/`family.cmake`、`boards/nanoch32v203/`） |
| 链接脚本 | `hw/bsp/ch32v20x/linker/ch32v20x.ld`（默认 `__flash_size=64K`、`__ram_size=20K`、`__stack_size=2048`） |
| 设备侧驱动 | `src/portable/wch/dcd_ch32_usbfs.c` + `src/portable/st/stm32_fsdev/`（FSDev） |
| 主机侧驱动 | `src/portable/wch/hcd_ch32_usbfs.c` |
| SDK | `hw/mcu/wch/ch32v20x/EVT` |
| 板卡列表 | `docs/reference/boards.rst` |
| HIL 配置 | `test/hil/tinyusb.json` → `nanoch32v203`（**只声明了 device 测试**，`host`/`dual` 为 false；那里记录的 UID 与本机这块板不同） |
| CMake preset | `hw/bsp/BoardPresets.json` → `nanoch32v203` |
