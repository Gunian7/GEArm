#!/usr/bin/env python3
"""Read the GEArm diagnostic counters + FDCAN state straight off the board.

Purpose: localise a "motor does not move" report in one shot. Every link in the
chain has its own counter, so the first counter that stops incrementing *is*
the break.

Counters live in .bss of the running image; addresses come from
GEArm-H723-Firmware/Debug/symbols.txt and must be re-read after every rebuild
(this script does that automatically).

Usage:
    py -3 read_diag.py            # one snapshot
    py -3 read_diag.py --watch    # keep reading every second

Compare two snapshots around one `rel_rotate` from the Web console:
    python read_diag.py > before.txt
    (click / send rel_rotate 0 1)
    python read_diag.py > after.txt
then diff the counters.
"""
import argparse
import pathlib
import re
import struct
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent     # .../GEArm-H723-Firmware
FW = HERE
SYMBOLS = FW / "Debug" / "symbols.txt"
JLINK = pathlib.Path("F:/MCU/STM32/Jlink/JLink_V914a/JLink.exe")
JLINK_SN = "69405446"

# FDCAN1 = D2_APB1PERIPH_BASE (0x40000000) + 0xA000
FDCAN1 = 0x4000A000
FDCAN_REGS = [
    ("CCCR",  0x018, "时钟控制/自动重发/模式"),
    ("NBTP",  0x01C, "标称位时序（prescaler/tseg1/tseg2/sjw）"),
    ("ECR",   0x040, "TEC/REC 错误计数"),
    ("PSR",   0x044, "LEC/ACT/BO 协议状态"),
    ("TXBC",  0x0C0, "TX FIFO 配置（TFQS = FIFO 深度）"),
    ("TXFQS", 0x0C4, "TFFL = FIFO 剩余空位（满=0，从未用=深度）"),
    ("TXBRP", 0x0CC, "待发送请求位图"),
    ("TXBTO", 0x0D8, "已成功发出的缓冲位图"),
    ("RXF0S", 0x0A4, "RX FIFO0 填充等级"),
]

COUNTERS = [
    ("g_dbg_scanf_float_ok", "1 = sscanf 支持 %f（0 = 链接漏了 -u _scanf_float）"),
    ("g_dbg_last_angle",     "上次算出的 rel_angle（按 float 解读）"),
    ("g_dbg_last_steps",     "上次算出的脉冲数（0 = 电机收到\"移动 0 步\"）"),
    ("g_dbg_last_vel",       "上次算出的转速"),
    ("g_dbg_can_tx_ok",      "CAN 帧成功入队/发出次数 ← 这里不涨说明没发到总线"),
    ("g_dbg_can_tx_busy",    "TX FIFO 满被拒次数"),
    ("g_dbg_can_tx_err",     "其它 HAL 错误次数"),
    ("g_dbg_cmd_queued",     "USB 收到并进 cmd_queue 的命令数"),
    ("g_dbg_cmd_parsed",     "cmd_service 解析的命令数"),
    ("g_dbg_event_queued",   "成功投递到 event_queue 的事件数"),
    ("g_dbg_rotate_called",  "进入 robot_joint_rotate_to 次数 ← 急停会把它挡住"),
    ("g_dbg_rotate_reject",  "被关节号/负速度拒绝的次数"),
]

# 这些符号的低 32 位要按 IEEE-754 单精度浮点解读
FLOAT_SYMS = {"g_dbg_last_angle"}


def symtab():
    """name -> address from nm output."""
    out = {}
    if not SYMBOLS.exists():
        return out
    for line in SYMBOLS.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                out[parts[-1]] = int(parts[0], 16)
            except ValueError:
                pass
    return out


def run_jlink(cmds):
    script = HERE / "_read_diag.jlink"
    script.write_text("\n".join(cmds + ["q"]) + "\n", encoding="utf-8")
    cmd = [str(JLINK), "-Device", "STM32H723VG", "-If", "SWD", "-Speed", "100",
           "-SelectEmuBySN", JLINK_SN, "-AutoConnect", "1", "-ExitOnError", "1",
           "-CommandFile", str(script)]
    p = subprocess.run(cmd, capture_output=True, text=True, errors="replace",
                       timeout=40)
    return p.stdout + p.stderr


def parse_mem32(text):
    """'4000A0C4 = 00000008' -> {'4000A0C4': 8}"""
    vals = {}
    for m in re.finditer(r"^\s*([0-9A-F]{8})\s*=\s*([0-9A-F]{8})\b", text, re.M):
        vals[m.group(1).upper()] = int(m.group(2), 16)
    return vals


def snapshot():
    syms = symtab()
    cmds = ["si SWD", "speed 100", "h"]
    want = {}
    for name, _desc in COUNTERS:
        if name in syms:
            want[name] = syms[name]
    want["g_estop"] = syms.get("g_estop", 0x24000446)

    for addr in sorted(set(want.values())):
        cmds.append("mem32 0x%08X, 1" % addr)
    for _n, off, _d in FDCAN_REGS:
        cmds.append("mem32 0x%08X, 1" % (FDCAN1 + off))
    cmds.append("g")          # let it run again
    text = run_jlink(cmds)
    vals = parse_mem32(text)
    if not vals:
        print("!! 没有从 J-Link 读到任何值。输出尾部：")
        print("\n".join(text.strip().splitlines()[-15:]))
        return None
    return syms, want, vals


def show(snap):
    syms, want, vals = snap
    print("=" * 70)
    print("GEArm 诊断快照")
    print("=" * 70)

    estop = vals.get("%08X" % (want["g_estop"] & 0xFFFFFFFF))
    if estop is not None:
        flag = estop & 0xFF
        print("  g_estop = %d  %s" % (flag, "★ 急停锁存中！所有运动事件会被丢弃" if flag else "未锁存"))
    print()

    print("-- 命令链路计数（累加，需两次对比）--")
    for name, desc in COUNTERS:
        a = want.get(name)
        v = vals.get("%08X" % a) if a is not None else None
        if v is None:
            shown = "?"
        elif name in FLOAT_SYMS:
            shown = "%.3f" % struct.unpack("<f", struct.pack("<I", v))[0]
        else:
            shown = str(v)
        print("  %-22s %-12s  %s" % (name, shown, desc))
    print()

    print("-- FDCAN1 寄存器 --")
    for name, off, desc in FDCAN_REGS:
        v = vals.get("%08X" % (FDCAN1 + off))
        if v is None:
            continue
        extra = ""
        if name == "NBTP":
            nsjw = (v >> 25) & 0x7F
            nbrp = (v >> 16) & 0x1FF
            ntseg1 = (v >> 8) & 0xFF
            ntseg2 = v & 0x7F
            extra = ("-> prescaler=%d tseg1=%d tseg2=%d sjw=%d  (%d tq)"
                     % (nbrp + 1, ntseg1 + 1, ntseg2 + 1, nsjw + 1,
                        1 + ntseg1 + 1 + ntseg2 + 1))
        elif name == "ECR":
            extra = "-> TEC=%d REC=%d" % (v & 0xFF, (v >> 8) & 0x7F)
        elif name == "PSR":
            lec = v & 0x7
            act = (v >> 3) & 0x3
            bo = (v >> 7) & 0x1
            lec_s = ["NoError", "Stuff", "Form", "Ack", "Bit1", "Bit0",
                     "CRC", "NoChange"][lec]
            act_s = ["Sync", "Idle", "Receiver", "Transmitter"][act]
            extra = "-> LEC=%s ACT=%s BusOff=%d" % (lec_s, act_s, bo)
        elif name == "TXBC":
            extra = "-> TFQS=%d (FIFO 深度) NDTB=%d" % (v & 0x3F, (v >> 16) & 0x3F)
        elif name == "TXFQS":
            tffl = v & 0x3F
            extra = "-> TFFL=%d 空位%s" % (tffl, "（=深度，从未发送过）" if tffl >= 8 else "")
        elif name == "TXBTO":
            extra = "-> 已发出缓冲位图 0x%X" % v
        elif name == "RXF0S":
            extra = "-> 填充 %d" % (v & 0x7F)
        print("  %-6s 0x%08X = 0x%08X  %s" % (name, FDCAN1 + off, v, extra))
    print()
    print("判读：g_dbg_rotate_called 不涨 → 事件没到运动函数（看 g_estop）")
    print("      rotate_called 涨但 can_tx_ok 不涨 → CAN 层没发出")
    print("      can_tx_ok 涨但电机不动 → 驱动菜单/使能/机械侧")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true", help="每秒重复读取")
    args = ap.parse_args()
    import time
    while True:
        snap = snapshot()
        if snap is None:
            return 1
        show(snap)
        if not args.watch:
            return 0
        time.sleep(1.0)


if __name__ == "__main__":
    sys.exit(main())
