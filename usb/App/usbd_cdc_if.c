/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usbd_cdc_if.h"
#include "robot.h"      /* GEArm 新增：文本命令入口 robot_cmd_send_from_isr / CMD_TYPE_UART1 */

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceHS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_HS(void);
static int8_t CDC_DeInit_HS(void);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_HS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

/* USB CDC 文本命令接收缓冲（GEArm 固件新增）
 * 和 usart.c 里 uart1_rx_buff 同样的角色：按字节攒，遇到 '\n' 算一行。
 * 128 字节足够放最长命令（robot_uart1_handle 的 sscanf 只取前 19 字符的命令名）。 */
#define USB_CMD_RX_BUF_SIZE  128U
static uint8_t usb_cmd_rx_buf[USB_CMD_RX_BUF_SIZE];
static uint32_t usb_cmd_rx_pos = 0U;

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS =
{
  CDC_Init_HS,
  CDC_DeInit_HS,
  CDC_Control_HS,
  CDC_Receive_HS,
  CDC_TransmitCplt_HS
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CDC media low layer over the USB HS IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_HS(void)
{
  /* USER CODE BEGIN 8 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS);
  return (USBD_OK);
  /* USER CODE END 8 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @param  None
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_HS(void)
{
  /* USER CODE BEGIN 9 */
  return (USBD_OK);
  /* USER CODE END 9 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 10 */
  switch(cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:
    /* Windows CDC 驱动打开 COM 口时会发送此命令设置 DTR/RTS。
     * pbuf[0] bit0=DTR, bit1=RTS。虚拟串口不需要实际控制硬件信号线，
     * 但必须响应此命令，否则 Windows 可能拒绝启动数据传输。
     * 表现为：设备枚举正常、COM 口出现，但 WriteFile 失败。
     *
     * 修复方案来自 usb_cdc_fix.md（2024-11-13），对应 HANDOFF.md 问题 A。 */
    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 10 */
}

/* 命令回显：把收到的命令原样回给 PC，让 Web/桥接能确认"板子真的收到了"。
 *
 * 为什么必须有：命令通道原来是单向的（收但不回），用户点了急停按钮后
 * 界面上只能看到"发送成功"，无法区分"命令没发出去"和"发了但板子没反应"。
 * 机械臂不动的时候这两种情况看起来一模一样，安全相关的操作不该靠猜。
 *
 * 格式："+OK <命令>\r\n"。前缀 '+' 是为了和二进制帧协议区分开 ——
 * bridge 的解析器认 0xA5 帧头，文本行会被单独路由，不会混进坏帧统计。
 *
 * 在 ISR 里调用，所以：
 *  - 不用 snprintf（不可重入），手工拼字符串
 *  - CDC_Transmit_HS 可能返回 USBD_BUSY（上一次发送还没完成），
 *    此时直接放弃本次回显。回显只是反馈，丢了不影响命令本身，
 *    绝不能在这里阻塞或重试。
 */
static uint8_t usb_cmd_tx_buf[USB_CMD_RX_BUF_SIZE + 8U];
static volatile uint32_t g_cdc_rx_lines = 0U;   /* 收到的命令数，J-Link 可读 */
static volatile uint32_t g_cdc_tx_lines = 0U;   /* 成功回显的命令数，J-Link 可读 */
static volatile uint32_t g_cdc_rx_calls = 0U;   /* CDC_Receive_HS 被调用次数，用于诊断接收问题（2024-11-13） */

static void usb_cmd_echo_line(const char *cmd)
{
  uint32_t n = 0U;
  g_cdc_rx_lines++;

  usb_cmd_tx_buf[n++] = (uint8_t)'+';
  usb_cmd_tx_buf[n++] = (uint8_t)'O';
  usb_cmd_tx_buf[n++] = (uint8_t)'K';
  usb_cmd_tx_buf[n++] = (uint8_t)' ';

  /* 原样回显命令内容，超长截断（缓冲区是静态的，绝不能写越界） */
  uint32_t space = (uint32_t)sizeof(usb_cmd_tx_buf) - n - 2U; /* 留 \r\n */
  for (uint32_t i = 0U; cmd[i] != 0x00U && i < space; i++) {
    usb_cmd_tx_buf[n++] = (uint8_t)cmd[i];
  }
  usb_cmd_tx_buf[n++] = 0x0DU;
  usb_cmd_tx_buf[n++] = 0x0AU;

  if (CDC_Transmit_HS(usb_cmd_tx_buf, (uint16_t)n) == USBD_OK) {
    g_cdc_tx_lines++;
  }
}

/* 诊断输出的通用文本通道：从任务上下文调用，把一行文本送到 PC。
 * 与上面的 +OK 回显区别：那个在 ISR 里跑、只回显命令；这个给
 * "查询驱动器状态" 这类需要打印十六进制回包的地方用。
 * 同样不做重试：USB 忙就丢，诊断输出不值得阻塞控制链。 */
void usb_cdc_send_text(const char *s)
{
  static uint8_t buf[160];
  uint16_t n = 0U;

  if (s == NULL) { return; }

  /* 拒绝从异常上下文（中断/ISR）调用。
   *
   * CDC_Transmit_HS 不可重入：它改 hcdc->TxState / PMA 缓冲。如果 EXTI 里
   * （限位、急停）的 CAN trace 和任务里的输出同时发，USB 栈会被写坏，
   * 之后的典型症状就是"枚举正常、TX 还偶尔能出，但 PC 再也写不进来"——
   * 和交接里问题 A（WriteFile failed、g_cdc_rx_lines 恒 0）完全对得上。
   * 诊断输出丢一行毫无影响，USB 被写坏的代价太大。 */
  if (__get_IPSR() != 0U) { return; }

  while (s[n] != '\0' && n < (uint16_t)(sizeof(buf) - 1U)) {
    buf[n] = (uint8_t)s[n];
    n++;
  }
  if (n == 0U) { return; }

  /* USB 忙就等一小会儿再发。早先直接丢弃，导致 qstat 这种连续输出只剩最后
   * 一两行 —— 丢一行诊断输出就等于读数不可信，比慢一点糟糕得多。
   * 本函数只在任务上下文调用（上面已挡掉 ISR），阻塞几十毫秒无害。 */
  for (uint32_t i = 0U; i < 50U; i++) {
    if (CDC_Transmit_HS(buf, n) == USBD_OK) { return; }
    HAL_Delay(2U);
  }
}

/**
  * @brief Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAILL
  */
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 11 */
  g_cdc_rx_calls++;  /* 诊断计数器：与 g_cdc_rx_lines 对比可判断是否调用了但没解析到完整命令 */

  /* GEArm 固件：USB CDC 收到的数据按"文本命令"处理，直接进 USART1 那条命令链路。
     终止条件是 0x0A（换行），与 usart.c 里 USART1 的接收逻辑保持一致；0x0D（回车）忽略。
     攒满一行就补 0x00 交给 robot_cmd_send_from_isr()，由 robot_cmd_service 任务解析。

     这样 Web 端发来的命令（rel_rotate / auto / hard_reset / estop ...）和
     从 USART1 发过来的是同一条路，行为完全一致。

     缓冲区溢出时丢弃当前这一行重新开始 —— 宁可丢一条命令，也不要写越界。
     这里用十六进制字面量而不是字符常量，避免补丁工具把转义序列吃成真换行。 */
  for (uint32_t i = 0; i < *Len; i++) {
    if (usb_cmd_rx_pos >= (USB_CMD_RX_BUF_SIZE - 1U)) {
      usb_cmd_rx_pos = 0U;    /* 溢出，丢弃这一行 */
    }
    if (Buf[i] == 0x0AU) {          /* 换行：一行结束 */
      usb_cmd_rx_buf[usb_cmd_rx_pos] = 0x00U;
      robot_cmd_send_from_isr((char *)usb_cmd_rx_buf, CMD_TYPE_UART1);
      usb_cmd_echo_line((char *)usb_cmd_rx_buf);
      usb_cmd_rx_pos = 0U;
    } else if (Buf[i] != 0x0DU) {   /* 回车：忽略 */
      usb_cmd_rx_buf[usb_cmd_rx_pos++] = Buf[i];
    }
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceHS);
  return (USBD_OK);
  /* USER CODE END 11 */
}

/**
  * @brief  Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 12 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceHS);
  /* USER CODE END 12 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_HS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 14 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  /* USER CODE END 14 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
