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
 * 限位开关引脚（6 个关节）
 * !! 待确认 !! 以下引脚暂沿用上游 F4 工程（零一造物 ZERO 机械臂）的定义。
 *    MC02 (STM32H723VGT6) 板上的实际接线确认后，请改为在 CubeMX 的 .ioc 里
 *    正式配置这些 GPIO(EXTI)，届时 CubeMX 会在上面生成同名宏，删除本段即可。
 * ------------------------------------------------------------------------- */
#ifndef JOINT_LIMIT_1_Pin
#define JOINT_LIMIT_1_Pin           GPIO_PIN_0
#define JOINT_LIMIT_1_GPIO_Port     GPIOD
#define JOINT_LIMIT_1_EXTI_IRQn     EXTI0_IRQn

#define JOINT_LIMIT_2_Pin           GPIO_PIN_1
#define JOINT_LIMIT_2_GPIO_Port     GPIOD
#define JOINT_LIMIT_2_EXTI_IRQn     EXTI1_IRQn

#define JOINT_LIMIT_3_Pin           GPIO_PIN_2
#define JOINT_LIMIT_3_GPIO_Port     GPIOD
#define JOINT_LIMIT_3_EXTI_IRQn     EXTI2_IRQn

#define JOINT_LIMIT_4_Pin           GPIO_PIN_3
#define JOINT_LIMIT_4_GPIO_Port     GPIOD
#define JOINT_LIMIT_4_EXTI_IRQn     EXTI3_IRQn

#define JOINT_LIMIT_5_Pin           GPIO_PIN_4
#define JOINT_LIMIT_5_GPIO_Port     GPIOD
#define JOINT_LIMIT_5_EXTI_IRQn     EXTI4_IRQn

#define JOINT_LIMIT_6_Pin           GPIO_PIN_5
#define JOINT_LIMIT_6_GPIO_Port     GPIOD
#define JOINT_LIMIT_6_EXTI_IRQn     EXTI9_5_IRQn
#endif /* JOINT_LIMIT_1_Pin */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
