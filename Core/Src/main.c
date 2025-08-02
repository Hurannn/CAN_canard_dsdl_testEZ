/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "canard.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.equipment.power.BatteryInfo.h"
#include <stdarg.h> // for va_list, va_start, va_end
#include <string.h> // for strlen()
#include <stdio.h> // for vsnprintf()
#include <stdlib.h> // for rand()
#include <math.h>  // for using NAN
//#include <random>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NODE_ID            20
#define TARGET_NODE_ID     10
#define STATUS_BROADCAST_INTERVAL_MS 1000
#define PMU_BROADCAST_INTERVAL_MS     500
#define CANARD_MTU         8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static CanardInstance canard;
static uint8_t canard_memory_pool[1024];
static uint8_t transfer_id_node_status = 0;
static uint8_t transfer_id_battery_info = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void canard_init(void);
void send_node_status(void);
void send_battery_info(void);
void handle_can_rx(void);
void handle_transfer_received(CanardInstance* ins, CanardRxTransfer* transfer);
bool should_accept_transfer(const CanardInstance* ins,
                            uint64_t* out_data_type_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debug_uart(const char* fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

void canard_init(void) {
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool),
               handle_transfer_received, should_accept_transfer, NULL);
    canard.node_id = NODE_ID;
}

bool should_accept_transfer(const CanardInstance* ins,
                            uint64_t* out_data_type_signature,
                            uint16_t data_type_id,
                            CanardTransferType transfer_type,
                            uint8_t source_node_id) {
    if (data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {
        *out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
        return true;
    }
    return false;
}


void handle_transfer_received(CanardInstance* ins, CanardRxTransfer* transfer) {
    if (transfer->transfer_type == CanardTransferTypeBroadcast &&
        transfer->source_node_id == TARGET_NODE_ID &&
        transfer->data_type_id == UAVCAN_PROTOCOL_NODESTATUS_ID) {

        struct uavcan_protocol_NodeStatus status;
        int res = uavcan_protocol_NodeStatus_decode(transfer, &status);
        if (res >= 0) {
            debug_uart("[Node 10] Uptime: %lus, Health: %u, Mode: %u\r\n",
                       (unsigned long)status.uptime_sec,
                       status.health,
                       status.mode);
        }
    }
}

void send_node_status(void) {
    struct uavcan_protocol_NodeStatus msg = {
        .uptime_sec = HAL_GetTick() / 1000,
        .health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK,
        .mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL,
        .vendor_specific_status_code = 0
    };

    uint8_t payload[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    int16_t payload_len = uavcan_protocol_NodeStatus_encode(&msg, payload);

    if (payload_len > 0) {
        canardBroadcast(&canard,
                        UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                        UAVCAN_PROTOCOL_NODESTATUS_ID,
                        &transfer_id_node_status,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        payload,
                        (uint16_t)payload_len);
    }
}

void send_battery_info(void) {
    struct uavcan_equipment_power_BatteryInfo msg = {0};

		msg.temperature = 317.0000f;
    msg.voltage = 20.3594f;
    msg.current = 1.0176f;
		
		msg.average_power_10sec = 0.0f;
		msg.remaining_capacity_wh = 0.0f;
		msg.full_charge_capacity_wh = 0.0f;
		msg.hours_to_full_charge = 0.0f;

    msg.status_flags = 1;
		msg.state_of_health_pct = 0;
		msg.state_of_charge_pct = 0;
		msg.state_of_charge_pct_stdev = 0;
		msg.battery_id = 0;
		
    msg.model_instance_id = 0;

    // Fill model_name (e.g., "stm32_pmu")
    const char* name = "stm32_pmu_123456v";
    msg.model_name.len = strlen(name);
    memcpy(msg.model_name.data, name, msg.model_name.len);

    uint8_t payload[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
    int16_t payload_len = uavcan_equipment_power_BatteryInfo_encode(&msg, payload);

    if (payload_len > 0) {
        canardBroadcast(&canard,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        &transfer_id_battery_info,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        payload,
                        (uint16_t)payload_len);
    }
}



void handle_can_tx(void) {
	int count = 0;
    while (1) {
        const CanardCANFrame* txf = canardPeekTxQueue(&canard);
        if (txf == NULL) break;  // No more frames

        CAN_TxHeaderTypeDef tx_header;
        tx_header.StdId = txf->id;
        tx_header.ExtId = txf->id;
        tx_header.IDE = CAN_ID_EXT;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = txf->data_len;
        tx_header.TransmitGlobalTime = DISABLE;

        uint32_t tx_mailbox;
        HAL_StatusTypeDef res = HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t*)txf->data, &tx_mailbox);
        if (res == HAL_OK) {
            canardPopTxQueue(&canard);
					count++;
        } else {
            // Can't send now — hardware busy, exit and retry later
					debug_uart("CAN TX mailbox full, retry later %d \r\n",HAL_GetTick());
            break;
        }
    }
		if (count > 0) {
        debug_uart("CAN TX frames sent: %d  ", count);
			debug_uart("ms: %d\r\n", HAL_GetTick());
    }
}


void handle_can_rx(void) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
            CanardCANFrame frame = {
                .id = rx_header.ExtId,
                .data_len = rx_header.DLC
            };
            memcpy(frame.data, data, 8);
            canardHandleRxFrame(&canard, &frame, HAL_GetTick() * 1000);
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	//MX_TIM2_Init();

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	canard_init();
	debug_uart("DroneCAN node started (ID %d)\r\n", NODE_ID);

	//uint32_t last_broadcast = HAL_GetTick();
	uint32_t last_status = 0;
	uint32_t last_pmu = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		handle_can_rx();
		while (canardPeekTxQueue(&canard)) {
        handle_can_tx();
    }
		
		uint32_t now = HAL_GetTick();
		
		
		if (now - last_status >= STATUS_BROADCAST_INTERVAL_MS) {
			send_node_status();
			last_status = now;
			HAL_GPIO_TogglePin(Bluepill_LED_GPIO_Port,Bluepill_LED_Pin);
        }

		if (now - last_pmu >= PMU_BROADCAST_INTERVAL_MS) {
				send_battery_info();
				last_pmu = now;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Bluepill_LED_GPIO_Port, Bluepill_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Bluepill_LED_Pin */
  GPIO_InitStruct.Pin = Bluepill_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Bluepill_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
