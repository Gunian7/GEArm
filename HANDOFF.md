# GEArm H723 固件 — 交接说明

> 项目总览与文档分类入口见 [`../README.md`](../README.md)。

> 用途：把当前状态、已排除的误区、待修问题一次性交代清楚，避免重复踩坑。
> 所有结论都有 J-Link 寄存器级或 CAN 字节级证据，不是推断。

---

## 1. 环境

| 项 | 值 |
|---|---|
| 板子 | 达妙 DM-MC02，主控 STM32H723VGT6 |
| **板载晶振** | **24 MHz**（说明书 V1.1 第 3 页 + 附录） |
| 电机 | 张大头 Emm42_V5.0 + CAN 模块，CAN 地址 1，16 细分（3200 脉冲/圈） |
| 关节减速比 | `g_joints_init[0].reduction_ratio = 50` |
| 工具链 | Windows：CubeIDE 1.19.0 自带 GCC 13.3.1（`F:\MCU\CubeIDE\...`）；Linux：CubeIDE for Linux 或 apt 的 `gcc-arm-none-eabi`，用 `ARM_GCC` / `CUBEIDE_ROOT` 指定（见 §2.1） |
| 仓库 | `github.com/Gunian7/GEArm`，分支 `main` |

**最容易踩的坑：晶振是 24 MHz，不是 CubeMX 默认的 25 MHz。**
按 25 MHz 编译会让 CAN 实际跑 480 kbps（偏 4%），电机一帧都解不出来。

---

## 2. 构建 / 烧录 / 调试

```bash
# 构建（不需要打开 IDE）
cd F:\Output\Project_Launch\GEArm\GEArm-H723-Firmware
py -3 build_cli.py         # 本机 python 不在 PATH 上；Linux 上换成 python3 build_cli.py
                           # 产出 Debug/GEArm.{elf,hex,bin,map} + size/nm 摘要

# 烧录（J-Link SWD）——.jlink 里的 loadfile 是相对路径，CWD 必须是仓库根
"F:\MCU\STM32\Jlink\JLink_V914a\JLink.exe" -CommanderScript flash_cli.jlink -AutoConnect 1 -ExitOnError 1
"F:\MCU\STM32\Jlink\JLink_V914a\JLink.exe" -CommanderScript verify_cli.jlink -AutoConnect 1 -ExitOnError 1

# Linux（Ubuntu 24.04）：脚本自己 cd 到仓库根，并自动定位 JLinkExe
./flash.sh                 # 等价于 JLinkExe -CommanderScript flash_cli.jlink ...
./verify.sh                # 只读读取运行态寄存器，不写 Flash
```

`.jlink` 脚本里一律用相对路径（`loadfile Debug/GEArm.elf`）：J-Link Commander 按
**当前工作目录**解释脚本里的相对路径，所以烧录前必须先 `cd` 到仓库根——
`flash.sh` / `verify.sh` 已经替你做了这一步。

J-Link Commander 两个必知语法（今天在这上面浪费了很多次）：

1. **命令行上传脚本路径要用原生 Windows 路径** `C:\tmp\x.jlink`。写 `/c/tmp/x.jlink` 会报
   `Could not open J-Link Command File`——J-Link 是原生 Windows 程序，不认 MSYS 路径。
   这条只影响手动传参；脚本内部的相对路径不受影响。
2. **写内存用逗号**：`w4 24000608, 1`。空格写法静默失败。

### 2.1 Ubuntu 24.04

**待实机确认**：以下 Linux 步骤尚未在 Ubuntu 机器上实跑。已在本机（Windows）验证的是
`build_cli.py` 的工具链发现与全量构建，以及 `flash.sh` / `verify.sh` 的查找与参数拼装
（用一个假的 JLinkExe 顶替，本机没装 J-Link）。

工具链（二选一；`build_cli.py` 按 `ARM_GCC` → CubeIDE 内置 → `PATH` 的顺序找）：

```bash
# A. STM32CubeIDE for Linux 自带工具链（与 Windows 侧同为 13.3.rel1）
export CUBEIDE_ROOT=/opt/st/stm32cubeide_1.19.0     # 路径不标准时才需要
arm-none-eabi-gcc --version                          # 自检

# B. 发行版包
sudo apt update && sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi
arm-none-eabi-gcc --version
```

构建 / 烧录：

```bash
cd GEArm/GEArm-H723-Firmware
chmod +x flash.sh verify.sh      # 正常 git clone 已带 755，从 zip 解包时才需要
python3 build_cli.py             # status: passed 即成功
./flash.sh                       # 需要 SEGGER J-Link 软件，见 README「Ubuntu 24.04」
```

- Linux 上的 J-Link 可执行文件叫 `JLinkExe`（无 `.exe`），默认装在
  `/opt/SEGGER/JLink_<版本>/`；Windows 的 `JLink.exe` 在 Linux 上不存在。
- `flash.sh` 找不到 JLinkExe 时打印安装提示并以非 0 退出；`.jlink` 里的
  `device` / `si` / `speed` / `h` / `go` / `q` 等命令两个平台通用。

读变量示例：

```
device STM32H723VG
si SWD
speed 4000
h
mem32 2400061C 1
go
q
```

---

## 3. 已验证正确 — 不要改

这些都是寄存器级 / 字节级实测，动之前先想清楚。

### 3.1 CAN 位时序（`Core/Src/fdcan.c`）

`HSE=24MHz → VCO 528MHz → CAN 内核 132MHz`

```
NBTP = 0x060A1203
  -> prescaler=11, tseg1=19, tseg2=4, sjw=4   （寄存器存的是 n-1）
  -> 位时间 24 tq，波特率 132MHz/(24×11) = 500,000 bps，采样点 83.3%
```

`fdcan.c` 里有 `_Static_assert` 把 HSE_VALUE / PLLM / PLLN / PLLQ / CAN 位时序锁在一起，
改一个漏一个会直接编译失败——这是故意的，别绕过。

### 3.2 分包（`Core/Src/robot.c` 的 `can_send_emm`）

厂商手册 6.4 节的位置命令样例与我们的输出**逐字节一致**：

```
厂商要求  包0: ID=0x0100  8字节  FD 01 0F A0 00 00 01 FA
          包1: ID=0x0101  5字节  FD 00 00 00 6B
我们输出  包0: ID=0x0100  8字节  FD 01 0F A0 00 00 01 FA   ✅
          包1: ID=0x0101  5字节  FD 00 00 00 6B            ✅
```

规则：剥掉地址字节 → 功能码作为每包首字节 → 其余按 7 字节一组 → `ID = (addr << 8) | pkt`。

### 3.3 其它

- `TxFifoQueueElmtsNbr = 8`（原来是 0，导致 `CAN_Send_Msg` 里
  `HAL_FDCAN_GetTxFifoFreeLevel() > 0` 恒为假，**所有帧都在入队前被丢弃**——
  这是"命令链路全通但电机毫无反应"的真凶）
- 6 条扩展帧过滤器 + 显式 `ConfigGlobalFilter`
- **`0x02` 是"成功"不是失败**。手册原文：`正确返回 01 46 02 6B，错误命令返回 01 00 EE 6B`。
  今天有人（我）据此误判过"电机拒绝命令"，实际电机一直在成功接受命令。

---

## 4. 问题 A — USB CDC 枚举成功，但 PC→板子写入失败

### 现象

- Windows 里 COM10 `Status: OK`、`Problem: CM_PROB_NONE`，设备枚举完全正常
- 板子 CPU 活着（J-Link 能读，`g_dbg_can_tx_ok = 12` 说明开机序列跑完）
- 但 PC 侧 `WriteFile` 失败，板子 `g_cdc_rx_lines` 恒为 **0**，一个字节都收不到
- bridge 日志：`device link lost: usb write failed on COM10: WriteFile failed`

### 已排除

| 怀疑对象 | 结论 |
|---|---|
| bridge 的 writer/reader 线程、`inflight` 清理、超时逻辑 | 逻辑正确，`_check_timeout` 确实清 `inflight` |
| `CDC_Receive_HS` 没有重新 arm OUT 端点 | 有，末尾是 `USBD_CDC_SetRxBuffer` + `USBD_CDC_ReceivePacket` |
| `USBD_CDC_Init` 没武装 OUT 端点 | 有，末尾 `USBD_LL_PrepareReceive(pdev, CDCOutEpAdd, hcdc->RxBuffer, 64)` |
| RX 缓冲区太小 | `APP_RX_DATA_SIZE = 2048`，够 |
| trace 开关忘了关，把 USB 冲垮 | `g_dbg_trace` 确认是 0 |
| PA11/PA12 没配成 AF10 | **确实没按正确顺序配**（见下），已修，但**不是本问题原因** |
| 设备挂起 / 软断开 | `DSTS` bit0=0（未挂起）、`DCTL`=0（未软断开） |
| USB 线 / USB 口 | 用户换线无效 |

### 附带修掉的一个真 bug（顺序反了）

`Core/Src/main.c` 里原来是：

```c
MX_USB_DEVICE_Init();          // USB 先起来
{ ... HAL_GPIO_Init(GPIOA, &usb_gpio); }   // PA11/PA12 后才配成 AF10
```

注释里自己写着"不配的后果：PC 永远枚举不出设备"，但代码还是把 GPIO 配置放在了
USB 初始化之后。已调整为 GPIO 先、`MX_USB_DEVICE_Init()` 后。
**这个 bug 真实存在，但修复后问题 A 依旧**，所以还有别的原因。

### 建议的下一步（需要物理层证据）

用逻辑分析仪或示波器抓 D+/D-：
- **完全静默** → 固件 PHY 没驱动，查 `HAL_PCD` 时钟 / `USBD_LL_Init`
- **有信号但设备回 NAK** → 端点状态问题，读 `DOEPCTL` / `DOEPINT`
- **有信号且设备 ACK 但 `cdc_rx_lines` 不涨** → `CDC_Receive_HS` 没被调用，查中断

---

## 5. 问题 B — `rel_angle` 实际是 0（**已修复**，根因是 newlib-nano 缺浮点 scanf）

### 曾经的现象

用 trace 打出实际上线字节，`rel_rotate 0 5` 和 `rel_rotate 0 7` 产生的**字节完全相同**：

```
TX id=0x100 ln=08 d=FD 01 03 41 C8 00 00 00
```

三条独立证据：

1. 5° 和 7° 字节一样 → 角度没进命令
2. `clk` 高 3 字节全是 0 → `steps < 256`，而 5°×50×3200/360 本该是 **2222**
3. `g_robot.joints[0].current_angle` 发完美对口命令仍是 **90.0** → 相对分支的
   `joint->current_angle += angle` 没生效

### 根因（已由 commit `61c7ff7` 修复）

`robot_uart1_handle()` 用 `sscanf(cmd, "%19s %f %f ...", ...)` 解析参数。
newlib-nano 默认不链接浮点 scanf，`%f` 静默失败——`param[1]` 永远是 0。

修复方式是在 `build_cli.py` 的 LDFLAGS 里加
`-Wl,-u,_scanf_float` 和 `-Wl,-u,_printf_float`（现在两个都有，
构建脚本还会自检这两个符号是否存在）。

**这也解释了"电机不动"**：位置命令带的是 `clk = 0`，电机收到"移动 0 个脉冲"，
它当然返回成功并且不动。之前所有关于电机侧配置的怀疑（P_Serial、ID_Addr、
堵转保护）都是不必要的。

### 调试变量（仍可用于回归验证）

| 变量 | 地址 | 含义 |
|---|---|---|
| `g_dbg_last_angle` | `0x2400061C` | `robot_joint_rotate_to` 实际算出的 `rel_angle` |
| `g_dbg_last_steps` | `0x24000618` | 实际算出的 `steps` |
| `g_dbg_last_vel` | `0x24000614` | 实际算出的 `_velocity` |
| `g_dbg_can_tx_ok` | `0x24000620` | 成功入队的 CAN 帧数 |
| `g_cdc_rx_lines` | `0x24005A40` | USB 收到的命令行数 |
| `g_robot` | `0x2400048C` | `joints` 在 +80，`joints[0].current_angle` 在 +80 |

判读方法：发一条 `rel_rotate 0 5`，读 `g_dbg_last_angle`：
- **读出 0** → 问题在事件队列 / 参数传递
- **读出 5 但 steps 是 0** → 问题在 `reduction_ratio` 或浮点计算
- **读出 5 且 steps=2222 但 trace 里 clk 高位是 0** → 问题在 `can_send_emm` 分包

### 帧 trace 开关

`g_dbg_trace` @ `0x24000608`，置 1 后每条上线的 CAN 帧都会打到 USB：

```
w4 24000608, 1     # 开
w4 24000608, 0     # 关
```

输出格式：`TX id=0x100 ln=08 d=FD 01 03 41 C8 00 00 00`
注意 `can_send_emm` 也会被 EXTI 中断调用，trace 开着时不要触发限位/急停。

---

## 6. 当前进度（2026-10-31）

**已完成**

| 项 | 状态 |
|---|---|
| CAN 四个阻断（TX FIFO / 分包 / 波特率 / 过滤器） | ✅ 寄存器级验证 |
| `rel_angle = 0`（newlib-nano 缺浮点 scanf） | ✅ commit `61c7ff7` |
| **开机同步电机实际位置** | ✅ `robot_init()` 里对 6 轴调 `robot_update_current_angle()` |
| **软件限位** | ✅ `robot_clamp_joint_angle()`，绝对/相对两条路径都夹到 `[min_angle, max_angle]` |
| 诊断可观测性 | ✅ `jstat` 命令 + 限位/同步结果送 USB |
| USB PHY GPIO 顺序（`MX_USB_DEVICE_Init` 早于 PA11/PA12 配 AF10） | ✅ 已调整顺序 |

**未完成**

| 项 | 状态 |
|---|---|
| 问题 A：USB CDC 枚举成功但 PC→板子写入失败 | ❌ 待示波器看 D+/D- |
| 限位开关接线验证 + 开机启用 | ⏸ 用户尚未焊接限位开关 |
| 电机方向标定（`MOTOR_DIR_CW/CCW`） | ⏸ 需上板 |

**为什么先做位置同步再做软件限位**：限位判断靠 `current_angle`，
如果基准是 `g_joints_init` 的硬编码值而和电机实际位置差很多，限位就是假的。

### 新增命令 `jstat`

只读，把 6 个轴的当前角度、软件限位范围、减速比、限位开关使能状态打到 USB：

```
joint0 cur=  90.00  limit=   0.00.. 360.00  ratio= 50.00  limit_off
joint1 cur=  90.00  limit=  90.00.. 180.00  ratio= 50.89  limit_off
...
```

用法与验证方法：

- **验证开机同步**：上电后第一次 `jstat`，`cur` 应该是电机实际角度，
  而不是 `g_joints_init` 的硬编码值。同时启动时会打 6 行 `sync jN -> xx.xx`，
  没接电机的轴打 `!sync jN timeout, keep init xx.xx`。
- **验证软件限位**：发一条越界指令（如 joint1 范围 90..180，发 `rel_rotate 1 500`），
  板子回 `!limit j1 rel -> 180.00 (range 90.00..180.00)`，
  再 `jstat` 确认 `cur` 停在 180.00。
  注意是**夹到限位**而不是拒绝命令，这样 Web 上按 +/− 越界时是"走到头就停"。

### 下一步建议顺序

1. **焊接限位开关**，然后验证接线 → 开机启用 `ROBOT_STATUS_LIMIT_ENABLE`
   （目前只在 `soft_reset`/`hard_reset` 里设置，开机后是失效的）
2. 电机方向标定：`jstat` 看同步角度，对比实际机械姿态，定 `MOTOR_DIR_CW/CCW`
3. 问题 A（USB 写入失败）需要物理层证据

---

## 7. 电机侧观察记录（供参考，可能是假象）

以下都是在 `steps=0` 的前提下测的，**修完问题 B 需要重新测**：

- 开机回包 `F3 02 6B` → 使能命令成功
- `46 02 6B` → 控制模式切换成功
- `FD 02 6B` → 位置命令被接受（成功）
- 读命令全部正常：`flag=3A 03 6B`（使能+到位）、`tpos`、`pos`、`perr`、`vel`、`ver`
- **电机对任何地址都应答**（`qstat 1` / `qstat 2` / `qstat 3` 返回同一组数据）
  → CAN 模块没做地址过滤，所以"地址不对"这个可能性可以排除
- 总线干净：`TEC=0 REC=0 LEC=0`

---

## 8. 工作区状态与提交历史

源码**没有未提交改动**（本节旧版写的那份清单已经过期）：

- 迁移前的板级调试改动已进 `main` 的存档提交 `5a10876`；
- Ubuntu 24.04 迁移改动在 `ubuntu-24.04-migration` 分支上。

工作区只剩下面这些**未跟踪**的一次性诊断文件，故意没有纳入版本库 —— 它们是
排查过程的中间产物，结论已经固化进源码和本文档：

```
?? J1_LIMIT_SWITCH_TEST.md
?? USB_CDC_FIX_VERIFICATION.md
?? build_output.txt
?? check_cdc_counters.jlink
?? read_gpio_byte.jlink
?? read_gpio_clock.jlink
?? read_j1_gpio.jlink
?? read_j1_gpio_simple.jlink
?? test_jlink_connection.jlink
```

分支与最近提交：

```
ubuntu-24.04-migration  426c04c chore(build): 工具链自动发现 + J-Link 脚本改相对路径
                        5a10876 wip(firmware): 存档 Ubuntu 24.04 迁移前的板级调试改动
main                    5a10876 ← 迁移前的存档点
```

更早的历史（仅作背景；其中 `33b2e58` 的方向已被 `f420f93` 推翻）：

```
b56d754 fix(clock): 修正 HSE 为 24MHz 并重算 CAN 位时序，波特率从 480k 回到 500k
f420f93 fix(can): 按厂商协议分包发送经典 CAN 帧，修复电机无响应
33b2e58 fix(can): 开启 CAN FD，修复 13 字节位置指令被截断   ← 方向错了，已被 f420f93 推翻
```
