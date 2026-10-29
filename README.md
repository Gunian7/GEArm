# GEArm

桌面级 6 轴机械臂嵌入式控制系统。运行于达妙 DM-MC02 运动控制板（STM32H723VGT6，Cortex-M7），
通过 CANFD 总线驱动 6 路闭环步进电机，提供逆运动学求解、轨迹插值、限位保护与急停等完整能力。

---

## 1. 系统概述

GEArm 是一套面向桌面级 6 轴机械臂的完整控制方案，由三部分组成：

| 组成部分 | 位置 | 职责 |
| :--- | :--- | :--- |
| **嵌入式固件** | `GEArm-H723-Firmware/` | 实时运动控制、逆运动学、CANFD 总线驱动、安全保护 |
| **上位机桥接服务** | `serial-test/bridge/` | Web 服务、USB CDC 链路管理、命令白名单校验 |
| **工具链** | `serial-test/*.py` | 构建、单元测试、端到端链路测试、数据录制与导出 |

控制链路：

```
Web 控制界面
    │  HTTP / WebSocket
    ▼
桥接服务（Python，端口 8765）
    │  USB CDC（VID 0483 / PID 5740）
    ▼
GEArm 固件 ──── CANFD 500 kbps ──── 6 × 闭环步进电机
    │
    └─ 限位开关（6 路 GPIO，1 kHz 轮询）
```

---

## 2. 主要功能

### 2.1 运动控制

- **6 轴协同控制**：基于 DH 参数的逆运动学求解，支持末端位姿到关节角的解析
- **直线轨迹插值**：路径按 100 ms 分辨率离散，逐点做逆运动学，配合关节最优解选择
- **PID 闭环跟踪**：插值过程中逐周期修正各关节误差，抑制累积偏移
- **多模式运动指令**：

  | 指令 | 功能 |
  | :--- | :--- |
  | `rel_rotate <关节> <角度>` | 单关节相对转动 |
  | `auto <x> <y> <z>` | 末端空间坐标直线运动（自动逆解） |
  | `hard_reset` | 逐轴寻找限位并归零 |
  | `soft_reset` | 软复位 |
  | `zero` | 位置清零 |
  | `remote_enable` / `remote_disable` | 远程控制模式开关 |

### 2.2 安全保护

机械臂持续运动时若指令异常，可能撞坏机械结构。系统提供两层保护：

- **限位开关保护**：每关节一个常闭（NC）微动开关，COM 接 GND、NC 接 GPIO 配内部上拉。
  未触碰时读到低电平，触碰时 NC 断开、上拉电阻拉高即触发。触发立即停转并置零该轴。
  采用常闭接法，断线或脱焊时引脚悬空被拉高，同样判为触发——**安全相关开关一律 NC，常开接法在断线时会静默失效**。
- **急停**：`estop` 指令一次性向 6 轴下发停止命令并置位急停标志，阻断后续所有动作；
  必须显式 `estop_clear` 才恢复，不会自动解除。

### 2.3 通信架构

- **主控链路：USB CDC**。枚举为虚拟串口，免去 CH340 模块与 TTL 电平转换，
  文本命令协议，与板载 USART1 完全同一条解析链路
- **辅助链路：USART1**（PA9/PA10，115200 8N1），保留为调试与备用入口
- **电机总线：CANFD**。配置为经典 CAN 帧、500 kbps，兼容闭环步进电机协议
- **可选扩展：USART3 + ESP8266**，编译期通过 `ROBOT_MQTT_ENABLE` 开关控制，
  关闭时系统可完全脱机运行

---

## 3. 硬件平台

**达妙 DM-MC02 运动控制板**，主控 STM32H723VGT6（Cortex-M7 @ 275 MHz，1 MB Flash / 564 KB RAM）。

板载 3 路 CANFD 收发器、2 路 RS485、BMI088 IMU、QSPI Flash、DCMI 摄像头接口等。
本项目使用其中第 1 路 CANFD 驱动电机。

### 引脚分配

| 功能 | 引脚 | 说明 |
| :--- | :--- | :--- |
| FDCAN1_RX / TX | PD0 / PD1 | 电机总线，500 kbps |
| USART1_TX / RX | PA9 / PA10 | 文本命令链路 |
| USART3_TX / RX | PB10 / PB11 | ESP8266 MQTT（可选） |
| USB DM / DP | PA11 / PA12 | USB CDC 主控链路 |
| SWD | PA13 / PA14 | J-Link / ST-Link 调试 |
| 限位开关 J1–J6 | PA00 / PA02 / PE09 / PE13 / PD02 / PE14 | 均布于舵机排针，杜邦线直插 |

> 限位开关引脚选型经过完整电气冲突审查：板载原理图上全部已占用引脚（CAN、RS485、
> DCMI、QSPI、SPI、USB、可控电源等）均已排除，所选 6 脚在固件中无任何外设活动。
> 其中 PA02 与 PD02 引脚编号相同，因采用轮询检测而非 EXTI 中断，可正确区分；
> 若改回中断方案，该组合会因 SYSCFG EXTICR 单端口路由限制而静默失效。

---

## 4. 软件架构

### 4.1 固件

基于 FreeRTOS V10.3.1（CMSIS-RTOS V2 封装，`ARM_CM4F` 移植层），任务划分：

| 任务 | 职责 |
| :--- | :--- |
| `robot_control_task` | 事件队列消费，执行运动指令 |
| `robot_cmd_service` | 文本命令解析与分发 |

中断体系：

| 中断源 | 优先级 | 用途 |
| :--- | :--- | :--- |
| FDCAN1_IT0 | 5 | 电机总线接收 |
| TIM6_DAC | 0 | 1 kHz 系统时基 + 限位开关轮询 |
| OTG_HS | 0 | USB CDC 数据收发 |
| USART1 / USART3 | — | 串口命令接收 |

安全相关的限位检测挂在 TIM6 时基中断上而非独立任务，理由是 `vTaskDelay`
的周期不受保证——高优先级任务长时间运行会无限推迟轮询，导致保护响应不可预期。

### 4.2 上位机

桥接服务提供 Web 控制界面（实时遥测、摇杆、参数面板）与命令转发。
命令下发前经过白名单与参数校验，未登记的命令不会到达设备。

### 4.3 目录结构

```text
GEArm-H723-Firmware/
├── Core/
│   ├── Inc/              # robot.h、fdcan.h、FreeRTOSConfig.h 等核心头文件
│   ├── Src/              # robot.c（控制内核）、robot_kinematics.c（运动学）、
│   │                     # robot_cmd.c（命令解析）、fdcan.c（CANFD）、usb/ 等
│   └── Startup/          # startup_stm32h723vgtx.s（Cortex-M7 启动汇编）
├── Drivers/              # STM32H7xx HAL Driver 与 CMSIS 核心库
├── Middlewares/          # FreeRTOS 内核源码与 CMSIS_RTOS_V2 封装层
├── usb/                  # USB Device Library（CDC 类）
├── Debug/                # 构建产物（.elf / .hex / .bin / .map）
├── GEArm.ioc             # STM32CubeMX 工程配置
├── STM32H723VGTX_FLASH.ld
├── build_cli.py          # 命令行完整构建脚本
└── README.md
```

---

## 5. 构建与烧录

### 方式一：命令行构建（推荐）

无需打开 IDE，直接复现 CubeIDE Debug 配置：

```bash
python build_cli.py
```

产物输出至 `Debug/`：`GEArm.elf`、`GEArm.hex`、`GEArm.bin`、`GEArm.map`，
并附 `size.txt`、`symbols.txt`、`result.json` 供校验。

烧录与运行态验证：

```bash
JLink.exe -CommanderScript flash_cli.jlink -AutoConnect 1 -ExitOnError 1
JLink.exe -CommanderScript verify_cli.jlink -AutoConnect 1 -ExitOnError 1
```

### 方式二：STM32CubeIDE

导入工程后 `Build Project`（`Ctrl + B`）。
注意：`LimitSwitch_Init()` 与 `MX_USB_DEVICE_Init()` 均置于 USER CODE 区，
重新生成代码不会丢失。

### 环境要求

- STM32CubeIDE 1.19.0（或兼容版本）
- GNU Tools for STM32 13.3.rel1
- J-Link 或 ST-Link

---

## 6. 测试

| 套件 | 覆盖范围 |
| :--- | :--- |
| `bridge/test_bridge_api.py` | 桥接服务 API、命令白名单、文本与二进制双编码通路 |
| `integration_link_test.py` | 上位机到虚拟设备的端到端链路 |
| `e2e_link_test.py` | 完整链路场景回归 |
| `pc_service.py --selftest` | USB 端口发现与协议自检 |

固件侧另有一组基于 J-Link 的硬件在环验证工具（`exticr_probe.py`、
`exti_conflict.py`、`pin_drive.py`），用于在不驱动电机的前提下核对引脚路由与寄存器状态。

---

## 7. 开源致谢

本项目的逆运动学模型与轨迹规划算法借鉴自 **零一造物 ZERO 机械臂**
（gitee `dearxie/zero-robotic-arm`）。原项目面向 STM32F407 平台，
以其完整的机器人知识体系为桌面级机械臂的开放做出了贡献，特此致谢。

在此基础上，GEArm 针对 STM32H723 平台与 CANFD 电机总线完成了通信架构、
实时系统与安全机制的重新设计与实现。

---

## 8. 许可

沿用上游项目的开源许可。
