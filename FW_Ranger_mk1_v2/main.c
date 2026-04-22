/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACE_NODE_ID                 0x01U

#define ACE_CANID_HEARTBEAT         (0x700U + ACE_NODE_ID)   /* 0x701 */
#define ACE_CANID_COMMAND           (0x600U + ACE_NODE_ID)   /* 0x601 */
#define ACE_CANID_RESPONSE          (0x580U + ACE_NODE_ID)   /* 0x581 */

/* Command IDs from page 16 */
#define ACE_CMD_WRITE               0x01U
#define ACE_CMD_READ                0x02U

/* Parameter IDs - bring-up subset */
#define ACE_PARAM_LED_PA1           0x01U

/* Response status codes from page 17 */
#define ACE_STATUS_EXECUTING        0x00U   /* Accepted, command currently executing */
#define ACE_STATUS_QUEUED           0x01U   /* Accepted, command queued */
#define ACE_STATUS_DATA_FOLLOWS     0x02U   /* Accepted, data follows in payload */
#define ACE_STATUS_UNKNOWN_COMMAND  0x10U
#define ACE_STATUS_INVALID_PARAM    0x11U

/* Heartbeat fields from page 15/16 */
#define ACE_PROTOCOL_VERSION        0x01U
#define ACE_STATE_READY             0x01U
#define ACE_STATE_FAULT             0x02U
#define ACE_STATE_MOVING            0x03U
#define ACE_STATE_BOOTLOADER        0x04U
#define ACE_STATE_DISABLED          0x05U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t HeartbeatData[8];
uint32_t uptime_s = 0;

static uint8_t led_pa1_state = 0U;
static uint32_t last_heartbeat_ms = 0U;
static uint32_t last_uptime_ms = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);

/* USER CODE BEGIN PFP */
static void FDCAN_Heartbeat_Init(void);
static void FDCAN_SendHeartbeat(void);

static void ACE_SetLedPA1(uint8_t state);
static void ACE_SendResponse(uint8_t command_id, uint8_t status_code, uint8_t parameter_id, const uint8_t *payload, uint8_t payload_len);
static void ACE_ProcessCommand(const FDCAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ACE_SetLedPA1(uint8_t state)
{
  if (state)
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    led_pa1_state = 1U;
  }
  else
  {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    led_pa1_state = 0U;
  }
}

static void ACE_SendResponse(uint8_t command_id, uint8_t status_code, uint8_t parameter_id, const uint8_t *payload, uint8_t payload_len)
{
  FDCAN_TxHeaderTypeDef ResponseHeader;
  uint8_t ResponseData[8] = {0};

  ResponseHeader.Identifier          = ACE_CANID_RESPONSE;
  ResponseHeader.IdType              = FDCAN_STANDARD_ID;
  ResponseHeader.TxFrameType         = FDCAN_DATA_FRAME;
  ResponseHeader.DataLength          = FDCAN_DLC_BYTES_8;
  ResponseHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  ResponseHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  ResponseHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  ResponseHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  ResponseHeader.MessageMarker       = 0;

  ResponseData[0] = command_id;
  ResponseData[1] = status_code;

  /* Page 17 example uses Byte 2 as parameter_id */
  ResponseData[2] = parameter_id;

  if ((payload != NULL) && (payload_len > 0U))
  {
    if (payload_len > 5U)
    {
      payload_len = 5U;
    }
    memcpy(&ResponseData[3], payload, payload_len);
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &ResponseHeader, ResponseData) != HAL_OK)
  {
    Error_Handler();
  }
}

static void ACE_ProcessCommand(const FDCAN_RxHeaderTypeDef *rxHeader, const uint8_t *rxData)
{
  uint8_t command_id;
  uint8_t parameter_id;
  uint8_t response_payload[5] = {0};

  if (rxHeader->Identifier != ACE_CANID_COMMAND)
  {
    return;
  }

  command_id   = rxData[0];
  parameter_id = rxData[1];

  switch (command_id)
  {
    case ACE_CMD_WRITE:
      switch (parameter_id)
      {
        case ACE_PARAM_LED_PA1:
          /* Byte 2 = payload0 = LED state */
          if (rxData[2] == 0x00U)
          {
            ACE_SetLedPA1(0U);
            ACE_SendResponse(ACE_CMD_WRITE, ACE_STATUS_EXECUTING, ACE_PARAM_LED_PA1, NULL, 0U);
          }
          else if (rxData[2] == 0x01U)
          {
            ACE_SetLedPA1(1U);
            ACE_SendResponse(ACE_CMD_WRITE, ACE_STATUS_EXECUTING, ACE_PARAM_LED_PA1, NULL, 0U);
          }
          else
          {
            ACE_SendResponse(ACE_CMD_WRITE, ACE_STATUS_INVALID_PARAM, ACE_PARAM_LED_PA1, NULL, 0U);
          }
          break;

        default:
          ACE_SendResponse(ACE_CMD_WRITE, ACE_STATUS_INVALID_PARAM, parameter_id, NULL, 0U);
          break;
      }
      break;

    case ACE_CMD_READ:
      switch (parameter_id)
      {
        case ACE_PARAM_LED_PA1:
          response_payload[0] = led_pa1_state;
          ACE_SendResponse(ACE_CMD_READ, ACE_STATUS_DATA_FOLLOWS, ACE_PARAM_LED_PA1, response_payload, 1U);
          break;

        default:
          ACE_SendResponse(ACE_CMD_READ, ACE_STATUS_INVALID_PARAM, parameter_id, NULL, 0U);
          break;
      }
      break;

    default:
      ACE_SendResponse(command_id, ACE_STATUS_UNKNOWN_COMMAND, parameter_id, NULL, 0U);
      break;
  }
}

/* FDCAN RX callback */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if (hfdcan != &hfdcan1)
  {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      ACE_ProcessCommand(&RxHeader, RxData);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_FDCAN1_Init();

  /* USER CODE BEGIN 2 */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  FDCAN_Heartbeat_Init();

  /* Initial output levels */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  led_pa1_state = 0U;
  last_heartbeat_ms = HAL_GetTick();
  last_uptime_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* uptime in seconds */
    if ((now - last_uptime_ms) >= 1000U)
    {
      last_uptime_ms += 1000U;
      uptime_s++;
    }

    /* heartbeat every 5 seconds per page 15 */
    if ((now - last_heartbeat_ms) >= 5000U)
    {
      last_heartbeat_ms += 5000U;
      FDCAN_SendHeartbeat();
    }

    /* Optional alive indicator on PA0 only */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{
  /* USER CODE BEGIN FDCAN1_Init 0 */
  FDCAN_FilterTypeDef sFilterConfig = {0};
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */

  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;

  /* 1 Mbit/s nominal CAN timing assumption:
     FDCAN kernel clock = 80 MHz
     tq = Prescaler / 80 MHz = 8 / 80 MHz = 100 ns
     total tq per bit = 1 + TimeSeg1 + TimeSeg2 = 1 + 7 + 2 = 10
     bit time = 10 * 100 ns = 1 us => 1 Mbit/s
     sample point = (1 + 7) / 10 = 80%
  */
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;

  /* Data phase values are not used in classical CAN, but must be set */
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;

  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Accept only standard command frame for this node: 0x600 + node_id */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = ACE_CANID_COMMAND;
  sFilterConfig.FilterID2 = 0x7FFU;   /* exact match mask */

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Reject everything else not matching configured filters */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void FDCAN_Heartbeat_Init(void)
{
  TxHeader.Identifier = ACE_CANID_HEARTBEAT;     /* 0x700 + node_id */
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

static void FDCAN_SendHeartbeat(void)
{
  /* Per page 15/16:
     Byte 0: protocol_version
     Byte 1: system_state
     Byte 2: module temperature
     Byte 3: error_flags_lsb
     Byte 4: error_flags_msb
     Byte 5: uptime_lsb
     Byte 6: uptime_mid
     Byte 7: uptime_msb
  */
  HeartbeatData[0] = ACE_PROTOCOL_VERSION;
  HeartbeatData[1] = ACE_STATE_READY;
  HeartbeatData[2] = 0x00U; /* no temperature data yet */
  HeartbeatData[3] = 0x00U;
  HeartbeatData[4] = 0x00U;
  HeartbeatData[5] = (uint8_t)(uptime_s & 0xFFU);
  HeartbeatData[6] = (uint8_t)((uptime_s >> 8) & 0xFFU);
  HeartbeatData[7] = (uint8_t)((uptime_s >> 16) & 0xFFU);

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, HeartbeatData) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */