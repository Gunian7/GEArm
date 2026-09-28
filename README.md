# GEArm - 桌面 6 轴机械臂嵌入式控制固件 (STM32H723 / DM-MC02 适配版)

本项目基于开源项目 **零一造物 ZERO 机械臂**（gitee `dearxie/zero-robotic-arm`）的嵌入式控制工程深度重构与硬件适配而成。

原版基于 **STM32F407VET6** 开发板（bxCAN + FreeRTOS），本项目将其完整移植适配到了 **达妙科技 DM-MC02 运动控制板（主控芯片：STM32H723VGT6，Cortex-M7 550MHz）**，并对通信架构、实时操作系统和通信协议栈进行了全面升级与排错。

---

## 硬件适配说明：从 STM32F407 到 DM-MC02 (STM32H723VGT6)

达妙 DM-MC02 是一款高性能多轴运动控制板，板载 STM32H723VGT6 芯片，自带 3 路 CANFD 收发器与丰富外设接口。

### 1. 通信总线架构升级：bxCAN → FDCAN

* **原版 (STM32F407)**：使用传统经典 CAN（bxCAN），基于 `HAL_CAN_*` 接口，中断通过 `CAN1_RX0_IRQn` 接收。
* **本项目 (STM32H723)**：
  * 全面重构为 STM32H7 专用的 **FDCAN 架构**（基于 `HAL_FDCAN_*` 驱动接口）。
  * 重新设计了 `robot.c`、`fdcan.c` 与张大头步进闭环控制例程 `Emm_V5.c` 的底层收发逻辑，统一桥接至 `CAN_Send_Msg()`，适配了 H7 体系下独特的 DLC 数据长度宏定义（如 `FDCAN_DLC_BYTES_8 = 0x00080000U`）。
  * 配置并启用了 **FDCAN1 接收中断 (`FDCAN1_IT0_IRQn` / `FDCAN_IT_RX_FIFO0_NEW_MESSAGE`)** 与消息回调函数 `HAL_FDCAN_RxFifo0Callback()`，修复了电机角度读取与急停反馈超时问题。
  * 数据帧接收结构体由原版的 `can.CAN_RxMsg.ExtId` 字段对齐至 H7 的 `can.CAN_RxMsg.Identifier`。

### 2. 实时操作系统 (FreeRTOS) 重建与优先级协调

* 修复了移植过程中工程缺失 FreeRTOS 内核、配置头文件与链接脚本丢失导致大面积编译崩溃的问题。
* 引入了完整的 **FreeRTOS V10.3.1 (CMSIS-RTOS V2)** 协议栈，选用经官方验证与 H7 硬件兼容的 `ARM_CM4F` 移植层。
* 修复了 Cortex-M7 内核中断接管冲突：FreeRTOS 内核接管 `vPortSVCHandler` 与 `xPortPendSVHandler`，从中断向量表中移除重复的 `SVC_Handler` / `PendSV_Handler` / `SysTick_Handler`。
* 调整了全局 NVIC 优先级体系：
  * PendSV 中断优先级配置为 15（最低）。
  * DMA 与 FDCAN 接收中断抢占优先级调整至 5（符合 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 保护范围），彻底杜绝了 DMA/CAN 中断回调中调用 FreeRTOS API 导致的 hardfault 隐患。
* 在系统入口任务 `StartDefaultTask` 中注入了完整的 `robot_init()` 机械臂运动学控制内核初始化流程。

### 3. 引脚分配与板载外设适配 (Pinout Mapping)

| 功能模块 | 原版 F407 引脚 | 本项目 MC02 (STM32H723VGT6) 引脚 | 说明 |
| :--- | :--- | :--- | :--- |
| **FDCAN1_RX** | PB8 (AF9) | **PA11 (AF9)** | 连接板载 CAN1 收发器（注意：DM-MC02 板载 3 路 CAN，首尾需配 120Ω 终端电阻） |
| **FDCAN1_TX** | PB9 (AF9) | **PA12 (AF9)** | 连接板载 CAN1 收发器 |
| **DEBUG_UART1_TX** | PA9 | **PB14 (AF4)** | 用于上位机控制 / Shell 串口调试 (115200 8N1) |
| **DEBUG_UART1_RX** | PA10 | **PB15 (AF4)** | 串口命令接收与解析中断 |
| **USART3_TX** | PB10 | **PB10 (AF7)** | 预留通信 / 辅助串口 (115200 8N1) |
| **USART3_RX** | PB11 | **PB11 (AF7)** | 配合 DMA1_Stream0 接收 |
| **SWD 调试口** | PA13 / PA14 | **PA13 / PA14** | ST-Link / J-Link SWD 烧录与调试口 |
| **外部高速晶振** | 8MHz HSE | **HSE (PH0/PH1)** | 经 PLL 倍频至 550MHz 系统主频，启用 I-Cache 与 D-Cache |

*(注：6 个关节限位开关暂使用宏预留定义于 `Core/Inc/main.h`，可根据机械结构装配时的实际端子走线灵活重映射。)*

### 4. 其它修复与底层工程优化

* **剔除硬件强依赖**：清除了 `esp8266_mqtt.h` 中对 `stm32f4xx_hal.h` 的残留依赖，并将默认编译宏置为 `ROBOT_MQTT_ENABLE = 0U`。在无需外挂 ESP8266 模块的情况下，机械臂控制系统可完全独立脱机稳定运行。
* **重构日志打印系统**：补全了线程安全的带锁串口日志打印函数 `safe_printf`、`safe_printf_from_isr` 与 `LOG()` 系列宏定义，修复了 `usart.c` 内部的接收状态机解析逻辑。
* **多产物编译导出**：配置了自动化编译链路，单次构建可同步产出可执行 ELF、用于烧录的 HEX/BIN 文件以及详细的反汇编列表 (`.list`) 与符号内存分布图 (`.map`)。

---

## 目录结构

```text
GEArm-H723-Firmware/
├── Core/
│   ├── Inc/           # 包含 robot.h, fdcan.h, FreeRTOSConfig.h 等核心头文件
│   ├── Src/           # 包含 robot.c (机械臂内核), robot_kinematics.c (运动学), fdcan.c 等
│   └── Startup/       # startup_stm32h723vgtx.s (Cortex-M7 启动汇编)
├── Drivers/           # STM32H7xx_HAL_Driver 及 CMSIS 核心库
├── Middlewares/       # FreeRTOS V10.3.1 核心源码及 CMSIS_RTOS_V2 封装层
├── Debug/             # 预编译生成的完整固件 (GEArm.elf, GEArm.hex, GEArm.bin, GEArm.map)
├── GEArm.ioc          # STM32CubeMX 工程配置文件 (STM32H723VGT6)
├── STM32H723VGTX_FLASH.ld  # 针对 1MB Flash / 320KB RAM_D1 的链接脚本
└── README.md          # 本说明文档
```

---

## 编译与烧录指南

### 开发环境要求
* **IDE**：STM32CubeIDE 1.14.0 或更高版本（本项目在 STM32CubeIDE 1.19.0 下验证通过）
* **交叉编译器**：GNU Tools for STM32 (13.3.rel1 或兼容的 `arm-none-eabi-gcc`)
* **烧录工具**：ST-Link V2/V3 或 J-Link，配合 STM32CubeProgrammer

### 导入与编译
1. 打开 STM32CubeIDE，选择 `File -> Import... -> General -> Existing Projects into Workspace`。
2. 浏览并选择 `GEArm-H723-Firmware` 根目录，勾选后点击 `Finish`。
3. 右键点击工程名称，选择 `Build Project`（快捷键 `Ctrl + B`）。
4. 编译完成后，控制台将显示构建通过信息：
   ```text
   Finished building target: GEArm.elf
      text    data     bss     dec     hex filename
     93036     440   22336  115812   1c464 GEArm.elf
   Build Finished. 0 errors, 0 warnings.
   ```
5. 固件成品位于 `Debug/` 目录：
   * `GEArm.hex` / `GEArm.bin`：可以直接拖入烧录器烧写至开发板。

---

## 致谢与声明

本项目运动学逆解模型与轨迹规划算法参考自开源项目 **零一造物 ZERO 机械臂**（作者：零一造物）。原项目以其高性价比和完整的机器人知识体系为桌面级机械臂的开源普及做出了杰出贡献，特此致敬与鸣谢！
