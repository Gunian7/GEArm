/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
/* -------------------------------------------------------------------------
 * 限位开关引脚（沿用原项目 KW11-3Z 常闭 NC 接法）
 * 接法：COM→GND，NC→GPIO，NO 悬空。未按压低电平，按压被内部上拉拉高。
 * 判定：GPIO_PIN_SET = 已触发（robot.c robot_limit_poll）。
 *
 * ======================== 2026-10-28 轮询方案 ========================
 * 检测方式从 EXTI 中断改为 TIM6 1kHz 轮询（见 robot.c robot_limit_poll）。
 *
 * 为什么改：板子上唯一能直接用杜邦线接的排针是舵机/PWM 排针，上面只有
 *   PE13(TIM1_CH3) PE09(TIM1_CH1) PA02(TIM2_CH3) PA00(TIM2_CH1) PD02(UART5_RX/SBUS)
 * 5 个空闲脚，而 PA02 与 PD02 同为引脚号 2 → 同一条 EXTI 线。
 * 已实测证明同号线两个端口不能同时产生中断：SYSCFG_EXTICR 每条 EXTI 线只有
 * 4 bit 端口选择字段，后配的会把先配的路由静默覆盖（先配的那个中断永远不来）。
 * 这不是回调能修的，根因在硬件。
 *
 * 轮询读的是 HAL_GPIO_ReadPin(端口, 引脚)，端口+引脚组合唯一，
 * 所以 PA02 和 PD02 同号也分得清，5 个舵机排针脚全部可用，不必碰 24P 排母。
 *
 * 最终选脚（全部在舵机/PWM 排针，杜邦线直插）：
 *   J1=PA00  J2=PA02  J3=PE09  J4=PE13  J5=PD02
 *   J6=PE14（24P 排母上的空闲 GPIO，BOM 无此开关，不接线）
 * 代价：停电机延迟从 ~50µs 变成 ~1ms；10rpm 下过冲 0.06°，机械上可忽略。
 *
 * TIM1/TIM2 固件未使用（HAL timebase 用的是 TIM6），UART5 也未使用，
 * 故这 5 个脚电气上完全空闲，已核对固件无任何代码触碰。
 *
 * 原项目 BOM 只有 5 个开关（KW11-3Z-A×3 + KW11-3Z-B×2），第 6 轴无开关，
 * robot_joint_hard_reset 从 ROBOT_MAX_JOINT_NUM-2 开始循环即跳过 J6。
 * J6 的 PE14 悬空会被内部上拉拉高，但 robot_joint_limit_happend() 开头的
 * ROBOT_STATUS_LIMIT_ENABLE 检查会让它直接返回，不会误停电机。
 *
 * 已被占用、不可使用（依据官方原理图网络标签）：
 *   PD0/PD1(FDCAN1)、PA9/PA10(USART1)、PB10/PB11(USART3，原理图上是 IIC2_SCL/SDA)、
 *   PA13/PA14(SWD)、PH0/PH1(HSE)、PA7(WS2812 SPI6)、PA11/PA12(USB)、
 *   PC13/14/15(可控电源/5V EN)、PA5(ADC按键)、PA15(USER_KEY)、PA8(MCO1 24MHz)、
 *   PB0/PB2/PA1/PA3/PD11/PE11(QSPI 启动闪存，绝对不能碰)、
 *   PB5/PB6(CAN2)、PD12/PD13(CAN3)、PB3/PB4/PD7/PE15(SPI1)、PC0-PC3/PB13(SPI2)、
 *   PB14/PD8/PD9(RS485)、PE7/PE8(UART7)、PE2/PE3(UART10)、
 *   PB7/PB8/PB9/PB12/PC5-PC9/PD3/PE4-PE6(DCMI 摄像头)、
 *   PC10/PC11/PC12(SPI3)、PD14/PD15(UART9)、PE00/PE01(UART8)。
 *
 * ⚠ 注意：J1/J2 同在 GPIOA，J2/J5 同为 GPIO_PIN_2 —— 这是有意的，
 *   轮询方案下合法；若改回 EXTI 中断方案，这个组合会直接失效。
 * ------------------------------------------------------------------------- */
#ifndef JOINT_LIMIT_1_Pin
#define JOINT_LIMIT_1_Pin           GPIO_PIN_0
#define JOINT_LIMIT_1_GPIO_Port     GPIOA
#define JOINT_LIMIT_1_EXTI_IRQn     EXTI0_IRQn        /* 轮询方案下不再使用 */

#define JOINT_LIMIT_2_Pin           GPIO_PIN_2
#define JOINT_LIMIT_2_GPIO_Port     GPIOA
#define JOINT_LIMIT_2_EXTI_IRQn     EXTI2_IRQn

#define JOINT_LIMIT_3_Pin           GPIO_PIN_9
#define JOINT_LIMIT_3_GPIO_Port     GPIOE
#define JOINT_LIMIT_3_EXTI_IRQn     EXTI9_5_IRQn

#define JOINT_LIMIT_4_Pin           GPIO_PIN_13
#define JOINT_LIMIT_4_GPIO_Port     GPIOE
#define JOINT_LIMIT_4_EXTI_IRQn     EXTI15_10_IRQn

#define JOINT_LIMIT_5_Pin           GPIO_PIN_2
#define JOINT_LIMIT_5_GPIO_Port     GPIOD
#define JOINT_LIMIT_5_EXTI_IRQn     EXTI2_IRQn

#define JOINT_LIMIT_6_Pin           GPIO_PIN_14
#define JOINT_LIMIT_6_GPIO_Port     GPIOE
#define JOINT_LIMIT_6_EXTI_IRQn     EXTI15_10_IRQn
#endif /* JOINT_LIMIT_1_Pin */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
