#!/usr/bin/env python3
"""Command-line build for GEArm-H723-Firmware, replicating the CubeIDE Debug
configuration so the project can be built/flashed without opening the IDE.

Reads the configuration that CubeIDE stores in .cproject:
  includes : ../Core/Inc, ../Drivers/STM32H7xx_HAL_Driver/Inc (+Legacy),
             ../Drivers/CMSIS/Device/ST/STM32H7xx/Include, ../Drivers/CMSIS/Include,
             ../Middlewares/Third_Party/FreeRTOS/Source/{include,CMSIS_RTOS_V2,
             portable/GCC/ARM_CM4F}
  defines  : USE_PWR_LDO_SUPPLY, USE_HAL_DRIVER, STM32H723xx
  target   : STM32H723VGTx, cortex-m7, fpv5-d16, hard float, -Os, -g3
  linker   : STM32H723VGTX_FLASH.ld, nano/nosys specs, --gc-sections
  sources  : every .c under Core/Src and Drivers/STM32H7xx_HAL_Driver/Src, the
             startup .s under Core/Startup, and the FreeRTOS files that the
             previous build's .map shows were compiled.

Outputs Debug/GEArm.{elf,hex,bin,map} next to the sources, plus size/nm summary.

Toolchain discovery, first hit wins:
  1. $ARM_GCC            explicit path to the arm-none-eabi-gcc executable
  2. STM32CubeIDE        $CUBEIDE_ROOT (Windows keeps F:/MCU/CubeIDE as default)
                         plus /opt/st/stm32cubeide_* on Linux
  3. $PATH               shutil.which("arm-none-eabi-gcc")
objcopy/size/nm are taken from the same bin/ directory as the compiler found,
with the executable suffix of the current platform.
"""
import glob
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
OUT = ROOT / "Debug"
OUT.mkdir(exist_ok=True)

# 工具链在 main() 里现找（find_toolchain），模块导入期不扫盘：这样"找不到
# 工具链"只会变成一条可读的错误信息，而不是导入期的 StopIteration 堆栈。
EXE = ".exe" if os.name == "nt" else ""
GCC_NAMES = ("arm-none-eabi-gcc.exe", "arm-none-eabi-gcc")
CUBEIDE_GNU_TOOLS = "com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*"

TOOLCHAIN_HINT = (
    "请任选一种方式提供工具链后重试：\n"
    "  1) export ARM_GCC=/path/to/arm-none-eabi-gcc\n"
    "  2) export CUBEIDE_ROOT=<STM32CubeIDE 安装根目录>，其下需有\n"
    "       plugins/" + CUBEIDE_GNU_TOOLS + "/tools/bin/arm-none-eabi-gcc\n"
    "       （Linux 通常是 /opt/st/stm32cubeide_1.19.0）\n"
    "  3) 把 arm-none-eabi-gcc 放进 PATH（Ubuntu: sudo apt install gcc-arm-none-eabi）\n"
    "详见 README.md 的「Ubuntu 24.04」小节。")


class ToolchainNotFound(RuntimeError):
    pass


def _cubeide_roots():
    """STM32CubeIDE 安装根目录候选。CUBEIDE_ROOT 一旦设置就以它为准（Windows
    不再回落到 F:/MCU/CubeIDE）；未设置时 Windows 保留原来的默认路径，并叠加
    Linux 的 /opt/st/stm32cubeide_*。"""
    roots = []
    env = os.environ.get("CUBEIDE_ROOT", "").strip()
    if env:
        roots.append(pathlib.Path(env).expanduser())
    elif os.name == "nt":
        roots.append(pathlib.Path("F:/MCU/CubeIDE"))
    roots += [pathlib.Path(p) for p in sorted(glob.glob("/opt/st/stm32cubeide_*"))]
    return roots


def _cubeide_compilers(root):
    """找出某安装根目录下自带 GNU 工具链里的 gcc。

    Windows 版把 plugins/ 放在 <root>/STM32CubeIDE/ 下、Linux 版直接放在
    <root>/ 下，所以 plugins 这一层用 ** 递归，不写死层级。
    """
    found = []
    for name in GCC_NAMES:
        pattern = str(root / "**" / "plugins" / CUBEIDE_GNU_TOOLS / "tools" / "bin" / name)
        found += [pathlib.Path(p) for p in sorted(glob.glob(pattern, recursive=True))]
    return [p for p in found if p.is_file()]


def find_toolchain():
    """按 ARM_GCC -> CubeIDE 内置 -> PATH 的顺序返回 arm-none-eabi-gcc 的绝对路径。"""
    env = os.environ.get("ARM_GCC", "").strip()
    if env:
        p = pathlib.Path(env).expanduser()
        if p.is_file():
            return p.resolve()
        # 显式指定却指错了就报错：静默换成另一套工具链的话，版本与浮点 ABI
        # 的差异在构建阶段完全看不出来，只会在上板后变成怪问题。
        raise ToolchainNotFound(
            "ARM_GCC 指向的不是一个文件：%s\n%s" % (env, TOOLCHAIN_HINT))

    roots = _cubeide_roots()
    for root in roots:
        for p in _cubeide_compilers(root):
            return p.resolve()

    found = shutil.which("arm-none-eabi-gcc")
    if found:
        return pathlib.Path(found).resolve()

    raise ToolchainNotFound(
        "找不到 arm-none-eabi-gcc。已查找：ARM_GCC（未设置）；CubeIDE 根目录 %s；PATH。\n%s"
        % (", ".join(str(r) for r in roots) or "（无）", TOOLCHAIN_HINT))


def tool_path(bindir, name):
    """objcopy/size/nm 与发现到的 gcc 同目录，后缀跟随平台。"""
    return bindir / ("arm-none-eabi-" + name + EXE)

INCLUDES = [
    "Core/Inc",
    "Drivers/STM32H7xx_HAL_Driver/Inc",
    "Drivers/STM32H7xx_HAL_Driver/Inc/Legacy",
    "Drivers/CMSIS/Device/ST/STM32H7xx/Include",
    "Drivers/CMSIS/Include",
    "Middlewares/Third_Party/FreeRTOS/Source/include",
    "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
    "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F",
    # USB CDC 栈（2026-10-28 从达妙官方 CtrBoard-H7_USB 例程搬入，经 serial-test 移植）
    "usb/App",
    "usb/Target",
    "usb/hal",                                        # stm32h7xx_hal_pcd.h / ll_usb.h
    "usb/Middlewares/ST/STM32_USB_Device_Library/Core/Inc",
    "usb/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc",
]
DEFINES = ["USE_PWR_LDO_SUPPLY", "USE_HAL_DRIVER", "STM32H723xx",
           # HSE_VALUE 不在这里覆盖：单一来源是 Core/Inc/stm32h7xx_hal_conf.h，
           # 那里已经是达妙 MC02 的真实晶振 24MHz（原来误写 25MHz，导致 CAN 实际
           # 跑 480k 而不是 500k，电机完全收不到）。fdcan.c 里有 _Static_assert
           # 把 HSE_VALUE 和 CAN 位时序锁在一起，配错会直接编译失败。
           "USE_USB_HS", "HAL_PCD_MODULE_ENABLED"]
CPU = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard"]
CFLAGS = [*CPU, "-Os", "-g3", "-std=gnu11", "-ffunction-sections", "-fdata-sections",
          "-Wall", "-c", *[f"-I{p}" for p in INCLUDES], *[f"-D{d}" for d in DEFINES]]
LDFLAGS = [*CPU, "-T", str(ROOT / "STM32H723VGTX_FLASH.ld"),
           "-Wl,--gc-sections", "-Wl,-Map=" + str(OUT / "GEArm.map"),
           "--specs=nano.specs", "--specs=nosys.specs", "-static",
           # ★ newlib-nano 默认不带浮点的 scanf/printf。必须显式用 -u 把这两个
           #   桩符号拉进来，否则：
           #     sscanf("rel_rotate 0 5", "%19s %f %f") -> 静默只转换 1 项，
           #     param[] 全保持 0 -> steps=0 -> 电机收到"移动 0 个脉冲"，
           #     回 FD 02（成功）但一动不动。这条曾经把整个排查带偏过一次。
           #   命令解析全靠 %f，LOG 里也大量 %f，两个都要。
           "-Wl,-u,_scanf_float", "-Wl,-u,_printf_float",
           "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group",
           "-Wl,--start-group", "-lgcc", "-Wl,--end-group"]

FREERTOS = [
    "Source/croutine.c", "Source/event_groups.c", "Source/list.c",
    "Source/queue.c", "Source/stream_buffer.c", "Source/tasks.c",
    "Source/timers.c", "Source/CMSIS_RTOS_V2/cmsis_os2.c",
    "Source/portable/GCC/ARM_CM4F/port.c", "Source/portable/MemMang/heap_4.c",
]
# USB CDC 栈。stm32h7xx_hal_spi*.c 不搬 —— 那是 serial-test 给 WS2812(SPI6) 用的，
# GEArm 没有 SPI6 外设活动，加了只会白白增大固件。
USB = [
    "usb/hal/stm32h7xx_hal_pcd.c",
    "usb/hal/stm32h7xx_hal_pcd_ex.c",
    "usb/hal/stm32h7xx_ll_usb.c",
    "usb/App/usb_device.c",
    "usb/App/usbd_desc.c",
    "usb/App/usbd_cdc_if.c",
    "usb/Target/usbd_conf.c",
    "usb/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c",
    "usb/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c",
    "usb/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c",
    "usb/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c",
]


def sources():
    out = []
    out += sorted((ROOT / "Core/Src").glob("*.c"))
    out += sorted((ROOT / "Core/Startup").glob("*.s"))
    out += sorted((ROOT / "Drivers/STM32H7xx_HAL_Driver/Src").glob("*.c"))
    frt = ROOT / "Middlewares/Third_Party/FreeRTOS"
    out += [frt / p for p in FREERTOS]
    out += [ROOT / p for p in USB]
    return [s for s in out if s.exists()]


def run(cmd, log):
    log.write("$ " + subprocess.list2cmdline([str(c) for c in cmd]) + "\n")
    log.flush()
    p = subprocess.run([str(c) for c in cmd], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, errors="replace")
    log.write(p.stdout)
    log.flush()
    if p.returncode:
        raise RuntimeError("exit %d: %s\n%s" % (p.returncode, cmd[0], p.stdout))
    return p.stdout


def main() -> int:
    objs = OUT / "obj"
    objs.mkdir(exist_ok=True)
    log = (OUT / "build.log").open("w", encoding="utf8")
    result = {"sources": 0, "objects": 0}
    try:
        TOOLCHAIN = find_toolchain()
        BINDIR = TOOLCHAIN.parent
        result["compiler"] = str(TOOLCHAIN)
        elf = OUT / "GEArm.elf"
        srcs = sources()
        result["sources"] = len(srcs)
        obuilt = []
        for s in srcs:
            obj = objs / (s.stem + ".o")
            run([TOOLCHAIN, *CFLAGS, s, "-o", obj], log)
            obuilt.append(obj)
        result["objects"] = len(obuilt)
        run([TOOLCHAIN, *LDFLAGS, *obuilt, "-o", elf], log)
        # 构建期断言：newlib-nano 的浮点 scanf/printf 必须真的被链接进来。
        # 少了它们不会报错，只会让 sscanf 静默丢弃 %f、参数保持 0 —— 曾经
        # 表现为"电机收到 steps=0 的命令，回成功但不动"。宁可这里构建失败。
        map_text = (OUT / "GEArm.map").read_text(encoding="utf-8", errors="replace")
        for hook in ("_scanf_float", "_printf_float"):
            if hook not in map_text:
                raise RuntimeError(
                    "link is missing %s: float scanf/printf are unavailable, "
                    "command parameters will silently parse as 0. "
                    "Keep -Wl,-u,%s in LDFLAGS." % (hook, hook))
        for fmt, ext in [("ihex", "hex"), ("binary", "bin")]:
            run([tool_path(BINDIR, "objcopy"), "-O", fmt, elf, OUT / ("GEArm." + ext)], log)
        size = run([tool_path(BINDIR, "size"), elf], log)
        nm = run([tool_path(BINDIR, "nm"), "-S", "-n", elf], log)
        (OUT / "size.txt").write_text(size, encoding="utf8")
        (OUT / "symbols.txt").write_text(nm, encoding="utf8")
        result["size"] = size.strip()
        wanted = ["EXTI0_IRQHandler", "EXTI1_IRQHandler", "EXTI2_IRQHandler",
                  "EXTI3_IRQHandler", "EXTI4_IRQHandler", "EXTI9_5_IRQHandler",
                  "EXTI15_10_IRQHandler", "LimitSwitch_Init", "exti"]
        result["symbols"] = [l for l in nm.splitlines() if l.split()[-1] in wanted]
        for name in ["GEArm.elf", "GEArm.hex", "GEArm.bin", "GEArm.map"]:
            p = OUT / name
            result[name] = {"sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
                            "bytes": p.stat().st_size}
        result["status"] = "passed"
    except Exception as exc:
        result["status"] = "failed"
        result["error"] = str(exc)
        print(str(exc), file=sys.stderr)
    finally:
        log.close()
        (OUT / "result.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf8")
    print(json.dumps({k: result.get(k, "") for k in ["status", "sources", "objects", "size"]},
                     indent=2, ensure_ascii=False))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
