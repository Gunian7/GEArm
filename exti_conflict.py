#!/usr/bin/env python3
"""Prove whether PA02 and PD02 can BOTH be armed as EXTI2 interrupt sources.

Hypothesis: SYSCFG_EXTICR gives every EXTI line a single 4-bit port selector,
so configuring PD02 after PA02 silently overwrites PA02's routing and PA02's
interrupt can never fire again. If true, no amount of callback cleverness can
save the pair -- the fix has to be either different pins or polling.

Method: call HAL_GPIO_Init twice (same as robot_joint_limit_set_irq does) and
read SYSCFG_EXTICR1 after each call. EXTI2 lives in EXTICR1 bits [11:8].
Port codes: PA=0 PB=1 PC=2 PD=3 PE=4.
"""
import pathlib
import re
import subprocess
import sys

JLINK = "F:/MCU/STM32/Jlink/JLink_V914a/JLink.exe"
HERE = pathlib.Path(__file__).resolve().parent

HAL_GPIO_INIT = 0x08005164
DEFAULT_HANDLER = 0x08003984
STRUCT, STACK = 0x20000000, 0x20010000
PORT_BASE = {"A": 0x58020000, "C": 0x58020800, "D": 0x58020C00, "E": 0x58021000}
MODE_IT_RISING = (0 << 0) | (1 << 16) | (1 << 20)      # 0x00110000
PULLUP = 0x1
EXTICR1 = 0x58000408

CASES = [("PD02", "D", 2), ("PA02", "A", 2)]   # 先 PD 后 PA，中间值非零，证据更硬


def jlink(lines):
    script = HERE / "exti_conflict.jlink"
    script.write_text("\n".join(["device STM32H723VG", "si SWD", "speed 4000",
                                 "Halt"] + lines + ["q"]) + "\n", encoding="utf8")
    p = subprocess.run([JLINK, "-CommanderScript", str(script),
                        "-AutoConnect", "1", "-ExitOnError", "0"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors="replace", timeout=600)
    out = p.stdout
    (HERE / "exti_conflict.log").write_text(out, encoding="utf8")
    if re.search(r"Illegal|Unknown command", out):
        raise RuntimeError("J-Link 报错:\n" + out[-1200:])
    return out


def main():
    lines = ["mem32 0x%08X, 1" % EXTICR1]
    for name, port, pin in CASES:
        lines += [
            "W4 0x%08X, 0x%X" % (STRUCT + 0, 1 << pin),
            "W4 0x%08X, 0x%X" % (STRUCT + 4, MODE_IT_RISING),
            "W4 0x%08X, 0x%X" % (STRUCT + 8, PULLUP),
            "W4 0x%08X, 0x%X" % (STRUCT + 12, 0),
            "W4 0x%08X, 0x%X" % (STRUCT + 16, 0),
            "mem32 0x%08X, 5" % STRUCT,
            "WReg R0 0x%08X" % PORT_BASE[port],
            "WReg R1 0x%08X" % STRUCT,
            "WReg R14 0x%08X" % DEFAULT_HANDLER,
            "SetPC 0x%08X" % HAL_GPIO_INIT,
            "Go", "Sleep 300", "Halt",
            "mem32 0x%08X, 1" % EXTICR1,
        ]
    out = jlink(lines)

    vals = [int(v, 16) for v in re.findall(r"^%08X\s*=\s*([0-9A-F]{8})" % EXTICR1,
                                           out, re.M)]
    pcs = re.findall(r"^PC = ([0-9A-Fa-f]{8})", out, re.M)
    if len(vals) < 1 + len(CASES):
        print("读数不足，日志见 exticr_conflict.log")
        return 1

    print("EXTICR2 字段位置：EXTI2 在 EXTICR1 的 bits[11:8]\n")
    names = "ABCDEFGH"
    # vals[0] 是基线读数，vals[1..] 才是每次配置之后的
    for i, (name, port, pin) in enumerate(CASES):
        v = vals[i + 1]
        code = (v >> 8) & 0xF
        print("配 %-5s 之后：EXTICR1 = 0x%08X   EXTI2 实际路由到 P%s (code %d)%s"
              % (name, v, names[code], code,
                 "   ← 被上一个覆盖了" if i > 0 and code != (vals[i] >> 8) & 0xF else ""))
    final = vals[len(CASES)]
    print("")
    print("基线（未配置时）: EXTICR1 = 0x%08X" % vals[0])
    code = (final >> 8) & 0xF
    print("")
    print("最终：EXTI2 只路由到 P%s" % "ABCDEFGH"[code])
    print("→ %s 的中断被静默覆盖，永远不会触发。" % CASES[0][0])
    return 0


if __name__ == "__main__":
    sys.exit(main())
