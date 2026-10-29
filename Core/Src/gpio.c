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
  /* 限位 GPIO 时钟：GPIOA=PA00/PA02（MX_GPIO_Init 已开过），
     GPIOE=PE09/PE13/PE14，GPIOD=PD02。
     GPIOC 不再需要（轮询方案改用了舵机排针的脚）。 */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* 2026-10-28 改为 TIM6 1kHz 轮询检测（robot.c robot_limit_poll），
     不再使用 EXTI 中断，因此这里删掉全部 NVIC 使能。
     原来那 6 组 HAL_NVIC_SetPriority/EnableIRQ 一并移除：
     - PA02 与 PD02 同为引脚号 2，同一条 EXTI 线，SYSCFG_EXTICR 只能路由一个端口，
       两个都配会互相覆盖（已实测），中断方案下这个引脚组合根本不成立；
     - 不使能 NVIC 后 exti.c 里的 handler 不会被触发，成为死代码，留着不碍事。 */
}
/* USER CODE END 2 */
