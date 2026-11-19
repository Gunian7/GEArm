/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "main.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 2 */
/**
  * @brief  限位开关初始化（必须放在 USER CODE 区，不能放 MX_GPIO_Init 生成区）
  *
  *         .ioc 里没有声明这 6 个限位 GPIO，若把 GPIO 时钟和 NVIC 使能写进
  *         MX_GPIO_Init() 函数体（CubeMX 生成区），点"生成代码"会被静默删掉
  *         → NVIC 不开 → 限位中断彻底失效，且不报错，只表现为"按了限位没反应"、
  *         robot_joint_reset 每次白等 12 秒超时。这种静默失效最难查，故独立成函数，
  *         由 main.c 的 USER CODE 区显式调用。
  *
  *         限位引脚映射见 main.h JOINT_LIMIT_x 宏（PD2-5 + PE0/PE1，J6 不接）。
  *         EXTI 中断优先级必须 >=5（FreeRTOSConfig.h
  *         configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5），robot_joint_limit_happend()
  *         里调用 xQueueSendToBackFromISR，优先级 <5 会 hardfault。统一用 6。
  */
void LimitSwitch_Init(void)
{
  /* 限位 GPIO 时钟：GPIOA=PA00/PA02，GPIOE=PE09/PE13/PE14，GPIOD=PD02。
     GPIOC 不再需要（轮询方案改用了舵机排针的脚）。 */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* 6 个限位脚全部配成上拉输入。
   *
   * 为什么必须在这里配：轮询方案下 TIM6 1kHz 中断从 HAL_Init() 起就在跑，
   * 而 g_robot.joints[] 要等 MX_FREERTOS_Init() -> robot_init() 才 memcpy 填充。
   * robot_limit_poll() 只挡了"端口指针还是 NULL"那段窗口，之后就全是真读了 ——
   * 如果引脚没配过，读的是复位默认的模拟浮空输入，值随机；一旦
   * ROBOT_STATUS_LIMIT_ENABLE 被置上（hard_reset / soft_reset 之后），
   * 随机上升沿就会让 robot_joint_limit_happend() 误停电机。
   *
   * 原来只靠 robot_joint_reset() 里的 robot_joint_limit_set_input() 现配现用，
   * 那只覆盖"已经复位过的轴"；没复位过的轴在限位使能后读到的就是悬空电平。
   *
   * 用 JOINT_LIMIT_x 宏而不是 g_robot.joints[]：本函数在 main() 里调用，
   * 早于 robot_init() 的 memcpy，那时候 joints 还是全零、端口指针是 NULL。 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;

  GPIO_InitStruct.Pin = JOINT_LIMIT_1_Pin;
  HAL_GPIO_Init(JOINT_LIMIT_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = JOINT_LIMIT_2_Pin;
  HAL_GPIO_Init(JOINT_LIMIT_2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = JOINT_LIMIT_3_Pin;
  HAL_GPIO_Init(JOINT_LIMIT_3_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = JOINT_LIMIT_4_Pin;
  HAL_GPIO_Init(JOINT_LIMIT_4_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = JOINT_LIMIT_5_Pin;
  HAL_GPIO_Init(JOINT_LIMIT_5_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = JOINT_LIMIT_6_Pin;
  HAL_GPIO_Init(JOINT_LIMIT_6_GPIO_Port, &GPIO_InitStruct);
}
/* USER CODE END 2 */
