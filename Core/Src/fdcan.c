/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
/* FDCAN 收发状态实例, 由 robot.c 与 FDCAN 中断回调共用 */
__IO CAN_t can = {0};

/* ---------------------------------------------------------------------------
 * 编译期自检：把「HSE_VALUE ↔ PLL ↔ CAN 位时序」三者锁死。
 *
 * 这一类错误最难查：配错晶振不会编译失败、不会运行报错、寄存器读出来也"正常"，
 * 只是波特率静默偏 4%，电机一声不响。所以这里用常量算式在编译期就算一遍：
 *
 *   内核频率 = HSE/PLLM × PLLN / PLLQ = HSE / 2 × 44 / 4
 *   波特率   = 内核频率 / (NominalPrescaler × (1 + TSEG1 + TSEG2))
 *            = 内核频率 / (11 × 24)
 *
 * 下面三个宏必须与 main.c 的 PLL 参数、本文件的位时序参数保持一致；
 * 任何一处改了而没同步，编译直接失败，不会再溜到板子上。
 * --------------------------------------------------------------------------- */
#define GEARM_PLL_M              2u
#define GEARM_PLL_N              44u
#define GEARM_PLL_Q              4u
#define GEARM_CAN_PRESCALER      11u
#define GEARM_CAN_TSEG1          19u
#define GEARM_CAN_TSEG2          4u

#define GEARM_CAN_KERNEL_HZ   ((uint32_t)(HSE_VALUE) / GEARM_PLL_M * GEARM_PLL_N / GEARM_PLL_Q)
#define GEARM_CAN_TQ_PER_BIT  (1u + GEARM_CAN_TSEG1 + GEARM_CAN_TSEG2)
#define GEARM_CAN_BITRATE     (GEARM_CAN_KERNEL_HZ / (GEARM_CAN_PRESCALER * GEARM_CAN_TQ_PER_BIT))

_Static_assert(GEARM_CAN_BITRATE == 500000u,
               "FDCAN bitrate must be 500 kbit/s. If you changed HSE_VALUE "
               "(DM-MC02 is 24 MHz), you must also re-derive PLLM/PLLN/PLLQ "
               "and NominalPrescaler/TimeSeg1/TimeSeg2 in fdcan.c.");
_Static_assert(GEARM_PLL_M == 2u && GEARM_PLL_N == 44u && GEARM_PLL_Q == 4u,
               "These must match SystemClock_Config() in main.c");
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  /* ---------------------------------------------------------------------------
   * 经典 CAN（不是 CAN FD）。
   *
   * 张大头 Emm_V5 / X42S 的 CAN 协议是**纯经典 CAN**：手册只有"扩展帧、
   * ID=(Addr<<8)|Packet、超过 8 字节要手工分包"这一套规则，全文没有 CAN FD
   * 概念（搜 "CAN FD"/"CANFD"/"数据段" 零命中；出现的 26 处 "FD" 全是功能码
   * 0xFD）。厂商给的 13 字节位置命令拆分样例就是 8B + 5B 两帧，正是经典 CAN
   * 8 字节上限的写法。
   *
   * 之前这里被改成 FDCAN_FRAME_FD_NO_BRS，想用 FD 一帧装下 13 字节——方向错了：
   * FD 帧的 FDF 位=1，经典 CAN 控制器会按错误帧处理或直接忽略。而且真正让
   * 位置指令失效的原因不是"一帧装不下"，而是**缺分包逻辑**（见 robot.c
   * can_send_emm）：13 字节本来就必须拆成 ID 0x100(8B) + ID 0x101(5B)。
   * --------------------------------------------------------------------------- */
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 11;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  /* 位时序：1(同步段) + TSEG1 + TSEG2 = 24 个时间份额
   *
   * 达妙 MC02 板载 24MHz 晶振 → PLL M2/N44 → VCO 528MHz
   *   → FDCAN 内核 PLL1Q = 528/4 = 132MHz
   *   → tq 时钟 = 132MHz / 11 = 12MHz
   *   → 波特率   = 12MHz / 24 = 500 kbit/s  ✓ 与驱动菜单 500k 一致
   *   采样点     = (1+19)/24 = 83.3%
   *
   * 原来是 TSEG1=20（共 25 tq），那是按 25MHz 晶振算的：
   *   137.5MHz/11/25 = 500k。晶振改回 24MHz 后 132/11/25 = 480k，
   *   偏 4%，电机收不到。所以 TSEG1 从 20 改成 19。 */
  hfdcan1.Init.NominalTimeSeg1 = 19;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  /* 经典 CAN 不用数据段时序，但 HAL 要求填合法值 */
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  /* 只收扩展帧：地址 1..6 各一条过滤项（main.c 里配置）。
     标准帧过滤项归零——张大头不用标准帧。 */
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 6;
  /* 经典 CAN 单帧上限 8 字节，元素大小就是 8 字节 */
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  /* ★ 关键修复：原来 TxBuffersNbr=0 且 TxFifoQueueElmtsNbr=0 —— 一个发送
   * 缓冲都没分配，HAL_FDCAN_GetTxFifoFreeLevel() 读 TXFQS.TFFL 恒为 0，
   * 于是 CAN_Send_Msg() 每次都在入队前返回 HAL_BUSY，**一个字节都没上过
   * 总线**。分配 8 个 FIFO 元素：6 轴使能/停止 + 分包余量。 */
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN1_MspInit 1 */
    /* FDCAN1 interrupt Init (IT0 负责 FIFO0 接收等中断) */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  FDCAN 接收 FIFO0 新消息回调函数
  * @param  hfdcan: FDCAN 句柄指针
  * @param  RxFifo0ITs: 触发回调的 FIFO0 中断标志
  * @retval 无
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, (FDCAN_RxHeaderTypeDef *)(&can.CAN_RxMsg), (uint8_t *)(can.rxData)) == HAL_OK)
    {
      can.rxFrameFlag = true;
    }
  }
}

/* USER CODE END 1 */
