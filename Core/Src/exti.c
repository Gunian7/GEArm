/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    exti.c
  * @brief   EXTI IRQ handlers for the joint limit switches.
  *
  *          The startup file declares EXTI0/1/2/3/4/9_5/15_10_IRQHandler as
  *          weak symbols pointing to Default_Handler. Without a strong
  *          definition here, a limit-switch edge jumps to Default_Handler (a
  *          tight infinite loop) and HAL_GPIO_EXTI_Callback() in robot.c is
  *          never reached. This file provides the missing strong definitions.
  *
  *          HAL_GPIO_EXTI_IRQHandler() already clears the pending bit and then
  *          calls HAL_GPIO_EXTI_Callback(), so the handlers below must NOT
  *          clear the pending bit again -- doing so can swallow a legitimate
  *          edge that arrived between the two clears.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  EXTI Line0 interrupt (limit switch 5 / PE0).
  *
  *          常闭 NC 开关（沿用原项目 KW11-3Z 接法）：COM→GND，NC→GPIO。
  *          未按压时 NC 闭合 GPIO 被拉低；按压时 NC 断开被上拉拉高，
  *          上升沿触发中断，HAL_GPIO_EXTI_Callback() 里判定 GPIO_PIN_SET
  *          为"已触发"。
  */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
  * @brief  EXTI Line1 interrupt (limit switch 6 / PE1).
  */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
  * @brief  EXTI Line2 interrupt (limit switch 1 / PD2).
  */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
  * @brief  EXTI Line3 interrupt (limit switch 2 / PD3).
  */
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/**
  * @brief  EXTI Line4 interrupt (limit switch 3 / PD4).
  */
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
  * @brief  EXTI Lines [9:5] interrupt (limit switch 4 / PD5).
  */
void EXTI9_5_IRQHandler(void)
{
  /* Only PD5 is currently mapped into this group (JOINT_LIMIT_4).
   * If a future limit switch lands on PE5..PE9, add its PIN here. */
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  }
}

/**
  * @brief  EXTI Lines [15:10] interrupt (unused, kept for completeness).
  */
void EXTI15_10_IRQHandler(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  }
  if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != 0x00U)
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
