/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
/* -------------------------------------------------------------------------
 * FDCAN 收发状态 (自上游 F4 工程的 can.h 移植, 类型改为 H7 的 FDCAN_*)
 * robot.c 依赖 can.rxFrameFlag / can.CAN_RxMsg / can.rxData
 * ------------------------------------------------------------------------- */
typedef struct {
	__IO FDCAN_RxHeaderTypeDef CAN_RxMsg;
	__IO uint8_t rxData[32];

	__IO FDCAN_TxHeaderTypeDef CAN_TxMsg;
	__IO uint8_t txData[32];

	__IO bool rxFrameFlag;
} CAN_t;

extern __IO CAN_t can;

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

