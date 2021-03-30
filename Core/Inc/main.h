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
#include "stm32l4xx_hal.h"

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
#define Button_Blue_Pin GPIO_PIN_13
#define Button_Blue_GPIO_Port GPIOC
#define Button_Blue_EXTI_IRQn EXTI15_10_IRQn
#define Motor_A_Direction_2_Pin GPIO_PIN_0
#define Motor_A_Direction_2_GPIO_Port GPIOA
#define Motor_A_Direction_1_Pin GPIO_PIN_1
#define Motor_A_Direction_1_GPIO_Port GPIOA
#define Motor_B_Direction_2_Pin GPIO_PIN_3
#define Motor_B_Direction_2_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOA
#define Motor_A_Speed_PWM_Pin GPIO_PIN_0
#define Motor_A_Speed_PWM_GPIO_Port GPIOB
#define Motor_B_Speed_PWM_Pin GPIO_PIN_1
#define Motor_B_Speed_PWM_GPIO_Port GPIOB
#define LSM6DSL_INT_1_Pin GPIO_PIN_11
#define LSM6DSL_INT_1_GPIO_Port GPIOD
#define LSM6DSL_INT_1_EXTI_IRQn EXTI15_10_IRQn
#define Motor_B_Direction_1_Pin GPIO_PIN_14
#define Motor_B_Direction_1_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
