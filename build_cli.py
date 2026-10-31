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
"""
import hashlib
import json
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
TOOLCHAIN = next(pathlib.Path("F:/MCU/CubeIDE").rglob("arm-none-eabi-gcc.exe"))
BINDIR = TOOLCHAIN.parent
OUT = ROOT / "Debug"
OUT.mkdir(exist_ok=True)

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
    result = {"compiler": str(TOOLCHAIN), "sources": 0, "objects": 0}
    try:
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
        for fmt, ext in [("ihex", "hex"), ("binary", "bin")]:
            run([BINDIR / "arm-none-eabi-objcopy.exe", "-O", fmt, elf, OUT / ("GEArm." + ext)], log)
        size = run([BINDIR / "arm-none-eabi-size.exe", elf], log)
        nm = run([BINDIR / "arm-none-eabi-nm.exe", "-S", "-n", elf], log)
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
