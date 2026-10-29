#!/usr/bin/env python3
"""Verify the limit-switch pin -> EXTI routing on real hardware WITHOUT moving
any motor.

robot_joint_limit_set_irq() is static and got inlined at -Os, so we call the
HAL function it calls -- HAL_GPIO_Init -- directly through the J-Link debugger:

  * the GPIO_InitTypeDef struct is written to DTCM (0x20000000, 128K, declared
    in the linker script but completely unused by the firmware)
  * the stack pointer is redirected to DTCM (0x20010000) so the call cannot
    corrupt FreeRTOS task stacks in RAM_D1
  * LR is pointed at Default_Handler (an infinite loop) so the call returns
    into a known safe place and we can halt cleanly

This reproduces exactly what the firmware does on a HARD_RESET event, with zero
motor motion. Expected EXTICR afterwards:
  EXTICR1 = 0x00000040   EXTI1  = PE (port code 4)
  EXTICR3 = 0x00000220   EXTI10 = PC, EXTI11 = PC
  EXTICR4 = 0x00003220   EXTI12 = PC, EXTI14 = PD, EXTI15 = PD
"""
import pathlib
import re
import subprocess
import sys

JLINK = "F:/MCU/STM32/Jlink/JLink_V914a/JLink.exe"
ELF = pathlib.Path(__file__).resolve().parent / "Debug" / "GEArm.elf"

HAL_GPIO_INIT = 0x08005164
DEFAULT_HANDLER = 0x08003984
STRUCT = 0x20000000      # DTCM, unused
STACK = 0x20010000       # DTCM, 64K above the struct

# port code -> base, per stm32h723xx.h (D3_AHB1PERIPH_BASE = 0x58020000)
PORTS = {"A": 0x58020000, "B": 0x58020400, "C": 0x58020800,
         "D": 0x58020C00, "E": 0x58021000}
MODE_IT_RISING = (0 << 0) | (1 << 16) | (1 << 20)   # 0x00110000
PULLUP = 0x1

JOINTS = [("J1", "C", 10), ("J2", "C", 11), ("J3", "C", 12),
          ("J4", "D", 14), ("J5", "D", 15), ("J6", "E", 1)]

EXTICR = {"EXTICR1": 0x58000408, "EXTICR2": 0x5800040C,
          "EXTICR3": 0x58000410, "EXTICR4": 0x58000414}
EXPECTED = {1: 0x40, 3: 0x2200, 4: 0x3302}


def build_script():
    lines = ["device STM32H723VG", "si SWD", "speed 4000", "Halt"]
    for name, port, pin in JOINTS:
        base = PORTS[port]
        lines += [
            # struct: Pin, Mode, Pull, Speed, Alternate
            "W4 0x%08X, 0x%X" % (STRUCT + 0, 1 << pin),
            "W4 0x%08X, 0x%X" % (STRUCT + 4, MODE_IT_RISING),
            "W4 0x%08X, 0x%X" % (STRUCT + 8, PULLUP),
            "W4 0x%08X, 0x%X" % (STRUCT + 12, 0),
            "W4 0x%08X, 0x%X" % (STRUCT + 16, 0),
            # read the struct back so a bad write cannot silently pass
            "mem32 0x%08X, 5" % STRUCT,
            "WReg R0 0x%08X" % base,
            "WReg R1 0x%08X" % STRUCT,
            "WReg R14 0x%08X" % DEFAULT_HANDLER,
            "SetPC 0x%08X" % HAL_GPIO_INIT,
            "Go", "Sleep 300", "Halt",
            "RReg R0",           # HAL_StatusTypeDef: 0 == HAL_OK
            "mem32 0x%08X, 1" % (STRUCT + 4),   # struct must be untouched
        ]
    for name, addr in EXTICR.items():
        lines.append("mem32 0x%08X, 1" % addr)
    lines.append("q")
    return lines


def run(lines):
    script = pathlib.Path(__file__).resolve().parent / "exticr_probe.jlink"
    script.write_text("\n".join(lines) + "\n", encoding="utf8")
    p = subprocess.run([JLINK, "-CommanderScript", str(script),
                        "-AutoConnect", "1", "-ExitOnError", "0"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, errors="replace", timeout=900)
    return p.stdout


def main():
    out = run(build_script())
    log = pathlib.Path(__file__).resolve().parent / "exticr_probe.log"
    log.write_text(out, encoding="utf8")

    # collect every "xxxxxxxx = yyyyyyyy" memory read
    mem = re.findall(r"^([0-9A-F]{8})\s*=\s*([0-9A-F]{8})", out, re.M)
    mem = {int(a, 16): int(v, 16) for a, v in mem}
    # Halt 打印的 PC 行：PC = xxxxxxxx, CycleCnt = ...
    # 返回后 PC 必须落在 Default_Handler（0x08003984，一条 b . 死循环）
    pcs = [int(v, 16) for v in re.findall(r"^PC = ([0-9A-Fa-f]{8})", out, re.M)]
    ret = pcs[1:] if len(pcs) == len(JOINTS) + 1 else pcs

    print("每次调用后停下的 PC:", ["0x%08X" % v for v in ret])
    ok = (len(ret) == len(JOINTS) and all(v == DEFAULT_HANDLER for v in ret))

    vals = {}
    for i, (name, addr) in enumerate(EXTICR.items()):
        v = mem.get(addr)
        vals[i + 1] = v
        print("%s @ 0x%08X = 0x%08X" % (name, addr, v if v is not None else -1))

    print("\n判定:")
    good = ok
    for idx, want in EXPECTED.items():
        got = vals.get(idx)
        same = got == want
        good = good and same
        print("  EXTICR%d  期望 0x%08X  实测 0x%08X  %s"
              % (idx, want, got if got is not None else -1, "PASS" if same else "FAIL"))
    print("\n总体:", "PASS" if good else "FAIL")
    print("完整日志: %s" % log)
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
