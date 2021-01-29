/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __enc_values{
	uint8_t state;
	uint8_t pre_state;
	int32_t count;
}enc_values;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_PPR 11
#define I2C_SLAVE_ADDR 0x34
#define WHO_AM_I_REGISTER 0x75
#define MOTOR_PWM_SPEED 40
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile enc_values enc1, enc2, enc3;
volatile uint8_t isRecieved;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void state2count(enc_values*);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// Motor1 initialize
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	enc1.pre_state = HAL_GPIO_ReadPin(ENCODER_1A_GPIO_Port, ENCODER_1A_Pin)
			+ 2 * HAL_GPIO_ReadPin(ENCODER_1B_GPIO_Port, ENCODER_1B_Pin);
	enc1.state = HAL_GPIO_ReadPin(ENCODER_1A_GPIO_Port, ENCODER_1A_Pin)
			+ 2 * HAL_GPIO_ReadPin(ENCODER_1B_GPIO_Port, ENCODER_1B_Pin);
	enc1.count = 0;

	// Motor2 initialize
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	enc2.pre_state = HAL_GPIO_ReadPin(ENCODER_2A_GPIO_Port, ENCODER_2A_Pin)
			+ 2 * HAL_GPIO_ReadPin(ENCODER_2B_GPIO_Port, ENCODER_2B_Pin);
	enc2.state = HAL_GPIO_ReadPin(ENCODER_2A_GPIO_Port, ENCODER_2A_Pin)
			+ 2 * HAL_GPIO_ReadPin(ENCODER_2B_GPIO_Port, ENCODER_2B_Pin);
	enc2.count = 0;

	//Motor3 initialize
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	enc3.pre_state = HAL_GPIO_ReadPin(ENCODER_3A_GPIO_Port, ENCODER_2A_Pin)
			+ 2 * HAL_GPIO_ReadPin(ENCODER_3B_GPIO_Port, ENCODER_3B_Pin);
	enc3.state = HAL_GPIO_ReadPin(ENCODER_3A_GPIO_Port, ENCODER_3A_Pin)
			+ 2 * HAL_GPIO_ReadPin(ENCODER_3B_GPIO_Port, ENCODER_3B_Pin);
	enc3.count = 0;

	//i2cのバッファは、uint8_tでデータ数＋１のサイズが望ましい。
	uint8_t txBuf[3] = { 0 };
	uint8_t rxBuf[3] = { 0 };
	isRecieved = 0;
	HAL_Delay(600);
	while (1) {
		HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)rxBuf, 1);
		while (!isRecieved) {
			//motor1 pid
			if (enc1.count > ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_PWM_SPEED);
			} else if (enc1.count < ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_PWM_SPEED);
			}else if (enc1.count == ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
			}

			//motor2 pid
			if (enc2.count > ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MOTOR_PWM_SPEED);
			} else if (enc2.count < ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MOTOR_PWM_SPEED);
			}else if (enc2.count == ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			}

			//motor3 pid
			if (enc3.count > ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, MOTOR_PWM_SPEED);
			} else if (enc3.count < ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, MOTOR_PWM_SPEED);
			}else if (enc3.count == ENCODER_PPR * 4 * 577) {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
			}
//
//			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
//			HAL_Delay(50);
		}
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
		isRecieved = 0;
		switch (rxBuf[0]) {
		case WHO_AM_I_REGISTER:
			txBuf[0] = I2C_SLAVE_ADDR;
			break;
		default:
			txBuf[0] = 0xFF;
			break;
		}
		while(HAL_I2C_GetState(&hi2c1)!=HAL_I2C_STATE_READY);
		HAL_I2C_Slave_Transmit(&hi2c1, (uint8_t*) txBuf, 1, 100);
		isRecieved = 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.OwnAddress1 = 52;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_1B_Pin ENCODER_1A_Pin ENCODER_2A_Pin */
  GPIO_InitStruct.Pin = ENCODER_1B_Pin|ENCODER_1A_Pin|ENCODER_2A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_2B_Pin ENCODER_3B_Pin ENCODER_3A_Pin */
  GPIO_InitStruct.Pin = ENCODER_2B_Pin|ENCODER_3B_Pin|ENCODER_3A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_R_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_R_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_W_Pin SW_B_Pin */
  GPIO_InitStruct.Pin = SW_W_Pin|SW_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	isRecieved = 1;
	UNUSED(hi2c);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//エンコーダ用の割り込みの有効化
	switch (GPIO_Pin) {
	case ENCODER_1A_Pin:
		enc1.pre_state = enc1.state;
		enc1.state = HAL_GPIO_ReadPin(ENCODER_1A_GPIO_Port, ENCODER_1A_Pin)
				+ 2 * HAL_GPIO_ReadPin(ENCODER_1B_GPIO_Port, ENCODER_1B_Pin);
		state2count(&enc1);
		break;
	case ENCODER_1B_Pin:
		enc1.pre_state = enc1.state;
		enc1.state = HAL_GPIO_ReadPin(ENCODER_1A_GPIO_Port, ENCODER_1A_Pin)
				+ 2 * HAL_GPIO_ReadPin(ENCODER_1B_GPIO_Port, ENCODER_1B_Pin);
		state2count(&enc1);
	case ENCODER_2A_Pin:
		enc2.pre_state = enc2.state;
		enc2.state = HAL_GPIO_ReadPin(ENCODER_2A_GPIO_Port, ENCODER_2A_Pin)
				+ 2 * HAL_GPIO_ReadPin(ENCODER_2B_GPIO_Port, ENCODER_2B_Pin);
		state2count(&enc2);
		break;
	case ENCODER_2B_Pin:
		enc2.pre_state = enc2.state;
		enc2.state = HAL_GPIO_ReadPin(ENCODER_2A_GPIO_Port, ENCODER_2A_Pin)
				+ 2 * HAL_GPIO_ReadPin(ENCODER_2B_GPIO_Port, ENCODER_2B_Pin);
		state2count(&enc2);
		break;
	case ENCODER_3A_Pin:
		enc3.pre_state = enc3.state;
		enc3.state = HAL_GPIO_ReadPin(ENCODER_3A_GPIO_Port, ENCODER_3A_Pin)
				+ 2 * HAL_GPIO_ReadPin(ENCODER_3B_GPIO_Port, ENCODER_3B_Pin);
		state2count(&enc3);
		break;
	case ENCODER_3B_Pin:
		enc3.pre_state = enc3.state;
		enc3.state = HAL_GPIO_ReadPin(ENCODER_3A_GPIO_Port, ENCODER_3A_Pin)
				+ 2 * HAL_GPIO_ReadPin(ENCODER_3B_GPIO_Port, ENCODER_3B_Pin);
		state2count(&enc3);
		break;
	default:
		break;
	}
}

void state2count(enc_values *ev) {
	switch (ev->state) {
	case 0:
		if (ev->pre_state == 1)
			(ev->count)--;
		if (ev->pre_state == 2)
			(ev->count)++;
		break;
	case 1:
		if (ev->pre_state == 3)
			(ev->count)--;
		if (ev->pre_state == 0)
			(ev->count)++;
		break;
	case 2:
		if (ev->pre_state == 0)
			(ev->count)--;
		if (ev->pre_state == 3)
			(ev->count)++;
		break;
	case 3:
		if (ev->pre_state == 2)
			(ev->count)--;
		if (ev->pre_state == 1)
			(ev->count)++;
		break;
	default:
		break;
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
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
		HAL_Delay(200);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
