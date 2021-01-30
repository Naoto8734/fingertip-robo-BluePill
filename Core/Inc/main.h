/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_BUILTIN_Pin GPIO_PIN_13
#define LED_BUILTIN_GPIO_Port GPIOC
#define MOTOR_PWM_2A_Pin GPIO_PIN_0
#define MOTOR_PWM_2A_GPIO_Port GPIOA
#define MOTOR_PWM_2B_Pin GPIO_PIN_1
#define MOTOR_PWM_2B_GPIO_Port GPIOA
#define MOTOR_PWM_3B_Pin GPIO_PIN_2
#define MOTOR_PWM_3B_GPIO_Port GPIOA
#define MOTOR_PWM_3A_Pin GPIO_PIN_3
#define MOTOR_PWM_3A_GPIO_Port GPIOA
#define ENCODER_1B_Pin GPIO_PIN_5
#define ENCODER_1B_GPIO_Port GPIOA
#define ENCODER_1B_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_1A_Pin GPIO_PIN_6
#define ENCODER_1A_GPIO_Port GPIOA
#define ENCODER_1A_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_2A_Pin GPIO_PIN_7
#define ENCODER_2A_GPIO_Port GPIOA
#define ENCODER_2A_EXTI_IRQn EXTI9_5_IRQn
#define ENCODER_2B_Pin GPIO_PIN_0
#define ENCODER_2B_GPIO_Port GPIOB
#define ENCODER_2B_EXTI_IRQn EXTI0_IRQn
#define ENCODER_3B_Pin GPIO_PIN_10
#define ENCODER_3B_GPIO_Port GPIOB
#define ENCODER_3B_EXTI_IRQn EXTI15_10_IRQn
#define ENCODER_3A_Pin GPIO_PIN_11
#define ENCODER_3A_GPIO_Port GPIOB
#define ENCODER_3A_EXTI_IRQn EXTI15_10_IRQn
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOB
#define SW_W_Pin GPIO_PIN_13
#define SW_W_GPIO_Port GPIOB
#define SW_B_Pin GPIO_PIN_14
#define SW_B_GPIO_Port GPIOB
#define MOTOR_PWM_1B_Pin GPIO_PIN_8
#define MOTOR_PWM_1B_GPIO_Port GPIOB
#define MOTOR_PWM_1A_Pin GPIO_PIN_9
#define MOTOR_PWM_1A_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
