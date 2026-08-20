# RESUME — CH32V203 USBFS 主机驱动 SOF 改造（换设备后继续用）

> 分支：`ch32-usbfs-sof-iso-pacing`　提交：`ef2b6f8e3`（WIP）
> 完整需求/环境文档（仓库外）：`/home/gehc/work/ch32/usbfs_sof_task.md`

## 目标
用 SOF 重新实现 `src/portable/wch/hcd_ch32_usbfs.c`，把等时（ISO）端点限速到 1 次/帧。
缺陷：改造前 `MIC CB=19466 SPK CB=3250 ERR CB=0`（远超 UAC 1.0 的 ~1000/s）。

## 已完成的改动（已提交）
1. 使能 SOF 中断 `USBFS_UIE_HST_SOF`；`sof_handler()` 维护帧计数，`hcd_frame_number()` 返回真实帧号。
2. 等时完成延迟到下一 SOF（`usb_edpt_t.iso_pending`）→ 每端点 1 事务/帧。
3. ISR 增加陈旧 TRANSFER 边沿守卫（仅当 `is_busy` 时才处理；**不在 ISR 内消费 TRANSFER 标志**——那会触发 SIE 多余事务）。
4. 端口复位/设备移除时清理挂起的等时完成。

## 当前已知问题（换设备后要验证/修复的）
- **GET_DESCRIPTOR 的 IN 数据阶段卡住**：第一次 IN `Read 1022 bytes`（应为 8）→ `Read more data` → 之后 `NAK` → 枚举失败。
- 已排除：ISR 消费标志；`hardware_start_xfer` 的 INT_EN/INT_FG 顺序；设备物理连接（重插后基线可正常枚举）。
- 下一步：重点查 `hcd_int_handler` 的 `USB_PID_IN` 分支与 SOF 时序，确认枚举期 `sof_handler` 未干扰普通 IN 完成；以及 ISO 延迟逻辑是否被误触发。

## 快速验证
```bash
cd examples/host/audio_host
export PATH="$HOME/work/ch32/ch32-docker/wlink-linux-x64:$PATH"
export CROSS_COMPILE="$HOME/work/ch32/ch32-docker/Toolchain/RISC-V-Embedded-GCC12/bin/riscv-wch-elf-"
export OPENOCD_WCH="$HOME/work/ch32/ch32-docker/OpenOCD/OpenOCD/bin/openocd"

make BOARD=nanoch32v203 LOG=1 clean all   # 构建
make BOARD=nanoch32v203 LOG=3 clean all   # 调试（USBH 日志）
make BOARD=nanoch32v203 flash             # 烧录+复位
stty -F /dev/ttyACM0 115200 cs8 -cstopb -parenb raw -echo
(sleep 1; wlink reset >/dev/null 2>&1) &
timeout 15 cat /dev/ttyACM0
```

## 验收判据
```
MIC CB=~1000 SPK CB=~1000 ERR CB=0
```
（连续抓 5+ 秒，取每秒打印值）
