#!/usr/bin/env python3
"""Drive one limit-switch MCU pin HIGH so you can find it on the header with a
multimeter -- no firmware change, no motor motion, no need to probe the chip.

Why this exists
---------------
The pin choice (PC10/PC11/PC12/PD14/PD15/PE01 on the extension header) was
derived from the official schematic. Before wiring 6 limit switches we want to
confirm the header really carries those pins, and in which order. Probing a
0.5 mm-pitch LQFP100 with a multimeter probe is unreliable, so instead we let
the MCU tell us: force one pin to a GPIO output driving HIGH, then sweep the
multimeter across the header. Whichever pin reads ~3.3V is that MCU pin.

Only the requested pin is driven HIGH; the other five are driven LOW (they
become outputs so BSRR can hold them at 0). So exactly one pin on the header
reads 3.3V at a time, and it moves when you ask for the next pin -- that rules
out static 3V3 power pins, which never change.

SAFETY -- read-modify-write, never a whole-register write:
  GPIOD PD0/PD1 are FDCAN1 (motor CAN bus) configured by fdcan.c. Writing the
  whole GPIOD MODER register would destroy CAN1 and silently kill motor
  communication. So MODER is read first, only our pin's 2 bits are changed,
  then written back. The same is done for GPIOC/GPIOE even though nothing else
  configures them today.

The MCU keeps running after J-Link disconnects, so the pin stays driven.
Run `python pin_drive.py off` (or just reset the board) when finished.

Usage
-----
    python pin_drive.py PC10      # PC10 high, the other five low
    python pin_drive.py PD15
    python pin_drive.py off       # all six back to input

Safety: confirm the extension header has nothing plugged in BEFORE running this.
"""
import pathlib
import re
import subprocess
import sys

JLINK = "F:/MCU/STM32/Jlink/JLink_V914a/JLink.exe"
HERE = pathlib.Path(__file__).resolve().parent

# GPIO port base addresses, stm32h723xx.h: D3_AHB1PERIPH_BASE = 0x58020000
PORT_BASE = {"A": 0x58020000, "B": 0x58020400, "C": 0x58020800,
             "D": 0x58020C00, "E": 0x58021000, "F": 0x58021400,
             "G": 0x58021800, "H": 0x58021C00}
MODER, OTYPER, PUPDR, IDR, BSRR = 0x00, 0x04, 0x0C, 0x10, 0x18

PINS = {"PC10": ("C", 10), "PC11": ("C", 11), "PC12": ("C", 12),
        "PD14": ("D", 14), "PD15": ("D", 15), "PE01": ("E", 1)}
PORTS = sorted({p for p, _ in PINS.values()})


def jlink(lines, what):
    script = HERE / "pin_drive.jlink"
    script.write_text("\n".join(["device STM32H723VG", "si SWD", "speed 4000",
                                 "Halt"] + lines + ["q"]) + "\n", encoding="utf8")
    p = subprocess.run([JLINK, "-CommanderScript", str(script),
                        "-AutoConnect", "1", "-ExitOnError", "1"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors="replace", timeout=300)
    out = p.stdout
    (HERE / "pin_drive.log").write_text(out, encoding="utf8")
    if p.returncode != 0 or re.search(r"Illegal|Unknown command|ERROR", out):
        raise RuntimeError("%s 失败:\n%s" % (what, out[-1500:]))
    return out


def read_moder():
    """Read MODER for every port that owns a candidate pin."""
    lines = ["mem32 0x%08X, 1" % (PORT_BASE[p] + MODER) for p in PORTS]
    out = jlink(lines, "读取 MODER")
    got = {int(a, 16): int(v, 16)
           for a, v in re.findall(r"^([0-9A-F]{8})\s*=\s*([0-9A-F]{8})", out, re.M)}
    moder = {}
    for p in PORTS:
        v = got.get(PORT_BASE[p] + MODER)
        if v is None:
            raise RuntimeError("没读到 GPIO%s 的 MODER" % p)
        moder[p] = v
    return moder


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    if arg is None:
        print(__doc__)
        print("\n可选: %s | off" % " ".join(sorted(PINS)))
        return 2

    off = arg.lower() == "off"
    target = None
    if not off:
        m = re.fullmatch(r"P([A-H])(\d{1,2})", arg.upper())
        if not m:
            raise SystemExit("引脚名格式应为 PA0..PH15，收到: %r" % arg)
        target = (m.group(1), int(m.group(2)))
        if target not in PINS.values():
            print("注意: %s 不在 6 个限位候选脚里，仍可驱动。" % arg.upper())

    moder = read_moder()

    # read-modify-write: change ONLY our pin's 2 MODER bits
    new_moder = dict(moder)
    if off:
        for name, (port, num) in PINS.items():
            new_moder[port] = moder[port] & ~(0x3 << (2 * num))
    else:
        port, num = target
        new_moder[port] = (moder[port] & ~(0x3 << (2 * num))) | (0x1 << (2 * num))

    lines = ["W4 0x%08X, 0x%08X" % (PORT_BASE[p] + MODER, new_moder[p])
             for p in PORTS if new_moder[p] != moder[p]]
    # every candidate low first (BSRR high half resets), then the target high
    for name, (port, num) in PINS.items():
        lines.append("W4 0x%08X, 0x%08X" % (PORT_BASE[port] + BSRR, 1 << (num + 16)))
    if target is not None:
        port, num = target
        lines.append("W4 0x%08X, 0x%08X" % (PORT_BASE[port] + BSRR, 1 << num))
        lines.append("mem32 0x%08X, 1" % (PORT_BASE[port] + IDR))
    jlink(lines, "驱动引脚")

    print("已执行: %s" % ("全部恢复输入" if off else "%s 拉高，其余 5 脚拉低" % arg.upper()))
    for p in PORTS:
        if new_moder[p] != moder[p]:
            print("  GPIO%s MODER: 0x%08X -> 0x%08X" % (p, moder[p], new_moder[p]))
    print("\nMCU 继续运行，J-Link 断开后电平保持。")
    print("万用表直流电压档：黑表笔接板上 GND，红表笔扫扩展接口排针，")
    print("找到读 ~3.3V 的那一针——它就是 %s。" % (arg.upper() if not off else "(无)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
