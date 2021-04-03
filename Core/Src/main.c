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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>

#include "serial.h"
#include "lsm6dsl.h"
#include "l298n.h"
#include "gpio_pin.h"
#include "std.h"
#include "verbose.h"
#include "math.h"

//#define ARM_MATH_CM4
//#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VERBOSE_FMT(fmt) "{MAIN} " fmt

#define LSM6DSL_EVENT_INTERRUPT			BIT(0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for primaryTask */
osThreadId_t primaryTaskHandle;
uint32_t primaryTaskBuffer[ 256 ];
osStaticThreadDef_t primaryTaskControlBlock;
const osThreadAttr_t primaryTask_attributes = {
  .name = "primaryTask",
  .cb_mem = &primaryTaskControlBlock,
  .cb_size = sizeof(primaryTaskControlBlock),
  .stack_mem = &primaryTaskBuffer[0],
  .stack_size = sizeof(primaryTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for secondaryTask */
osThreadId_t secondaryTaskHandle;
uint32_t secondaryTaskBuffer[ 256 ];
osStaticThreadDef_t secondaryTaskControlBlock;
const osThreadAttr_t secondaryTask_attributes = {
  .name = "secondaryTask",
  .cb_mem = &secondaryTaskControlBlock,
  .cb_size = sizeof(secondaryTaskControlBlock),
  .stack_mem = &secondaryTaskBuffer[0],
  .stack_size = sizeof(secondaryTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for robotStateMutex */
osMutexId_t robotStateMutexHandle;
osStaticMutexDef_t robotStateMutexControlBlock;
const osMutexAttr_t robotStateMutex_attributes = {
  .name = "robotStateMutex",
  .cb_mem = &robotStateMutexControlBlock,
  .cb_size = sizeof(robotStateMutexControlBlock),
};
/* Definitions for lsm6dslEvent */
osEventFlagsId_t lsm6dslEventHandle;
osStaticEventGroupDef_t lsm6dslEventControlBlock;
const osEventFlagsAttr_t lsm6dslEvent_attributes = {
  .name = "lsm6dslEvent",
  .cb_mem = &lsm6dslEventControlBlock,
  .cb_size = sizeof(lsm6dslEventControlBlock),
};
/* USER CODE BEGIN PV */
static float roll;
static GPIO_Pin led1 = { .port = LED_1_GPIO_Port, .pin = LED_1_Pin };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
void StartPrimaryTask(void *argument);
void StartSecondaryTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Serial_Init(&huart1);

  verboseln("=============================");
  verboseln("Peripherals initialization...");

  LSM6DSL_Init(&hi2c2);

  L298N_Config l298n_config = {
	  .motor_a = {
		  .enabled = true,
		  .direction_1 = {
			  .port = Motor_A_Direction_1_GPIO_Port,
			  .pin = Motor_A_Direction_1_Pin
		  },
		  .direction_2 = {
			  .port = Motor_A_Direction_2_GPIO_Port,
			  .pin = Motor_A_Direction_2_Pin
		  },
		  .speed_type = L298N_MOTOR_SPEED_TYPE_ANALOG,
		  .speed = {
			  .analog = {
				 .tim = &htim3,
				 .channel = TIM_CHANNEL_3,
			  }
		  }
	  },
	  .motor_b = {
		  .enabled = true,
		  .inverted = true,
		  .direction_1 = {
			  .port = Motor_B_Direction_1_GPIO_Port,
			  .pin = Motor_B_Direction_1_Pin
		  },
		  .direction_2 = {
			  .port = Motor_B_Direction_2_GPIO_Port,
			  .pin = Motor_B_Direction_2_Pin
		  },
		  .speed_type = L298N_MOTOR_SPEED_TYPE_ANALOG,
		  .speed = {
			  .analog = {
				 .tim = &htim3,
				 .channel = TIM_CHANNEL_4,
			  }
		  }
	  },
  };

  L298N_Init(l298n_config);

  verboseln("Starting FreeRTOS kernel...");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of robotStateMutex */
  robotStateMutexHandle = osMutexNew(&robotStateMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of primaryTask */
  primaryTaskHandle = osThreadNew(StartPrimaryTask, NULL, &primaryTask_attributes);

  /* creation of secondaryTask */
  secondaryTaskHandle = osThreadNew(StartSecondaryTask, NULL, &secondaryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of lsm6dslEvent */
  lsm6dslEventHandle = osEventFlagsNew(&lsm6dslEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65305;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
//  HAL_TIM_Base_MspInit(&htim3);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_A_Direction_2_Pin|Motor_A_Direction_1_Pin|Motor_B_Direction_2_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_B_Direction_1_GPIO_Port, Motor_B_Direction_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Blue_Pin */
  GPIO_InitStruct.Pin = Button_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_A_Direction_2_Pin Motor_A_Direction_1_Pin Motor_B_Direction_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = Motor_A_Direction_2_Pin|Motor_A_Direction_1_Pin|Motor_B_Direction_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_INT_1_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_B_Direction_1_Pin */
  GPIO_InitStruct.Pin = Motor_B_Direction_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_B_Direction_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_GPIO_EXTI_Callback(uint16_t pin)
{

#if VERBOSE
	GPIO_Pin gpio_pin = {
		.pin = pin
	};
	char s_gpio_pin[8];
	GPIO_Pin_ToString(gpio_pin, s_gpio_pin, 8);
	verboseln("HAL_GPIO_EXTI_Callback on pin %s", s_gpio_pin);
#endif // DEBUG


	if (pin == LSM6DSL_INT_1_Pin) {
		verboseln("Received LSM6DSL data-ready signal");
		osEventFlagsSet(lsm6dslEventHandle, LSM6DSL_EVENT_INTERRUPT);
	}
	if (pin == Button_Blue_Pin) {
		verboseln("Button has been pressed");
		GPIO_Pin_Toggle(led1);
		L298N_Stop();
		LSM6DSL_DisableAccelerometer();
		LSM6DSL_DisableGyroscope();
	}
}

static void setRoll(float roll_deg)
{
	static const float ACTION_THRESHOLD_DEG = 1;
	static const float ACTION_CRITICAL_DEG = 30;
	static const float DUTY_CYCLE_MIN = 40;
	static const float DUTY_CYCLE_MAX = 100;

	verboseln("Roll  (x): %f", roll_deg);

	float roll_abs = ABS(roll_deg);

	uint8_t duty_cycle = 0;
	if (roll_abs > ACTION_THRESHOLD_DEG) {
		if (roll_abs > ACTION_CRITICAL_DEG)
			duty_cycle = DUTY_CYCLE_MAX;
		else
			duty_cycle = mapf(roll_abs,
					ACTION_THRESHOLD_DEG, ACTION_CRITICAL_DEG,
					DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
		verboseln("Duty%%: %u", duty_cycle);
	}


	osMutexAcquire(robotStateMutexHandle, osWaitForever);
	roll = roll_deg;

	if (roll_abs < ACTION_THRESHOLD_DEG) {
		L298N_Stop();
	} else {
		if (roll < 0) {
			L298N_Backward(duty_cycle);
		} else {
			L298N_Forward(duty_cycle);
		}
	}

	osMutexRelease(robotStateMutexHandle);
}

static void handleAccelerometerData(float x, float y, float z)
{
	verboseln("Accelerometer: 	(x=%f, y=%f, z=%f)g", x, y, z);
	float roll = RAD_TO_DEG(atan2f(y, z));
	setRoll(roll);

//	float roll2 = RAD_TO_DEG(atanf(y / sqrt(xx+zz)));
//	float pitch = RAD_TO_DEG(atanf(x / sqrt(yy+zz)));
//
//	float xx = x * x;
//	float yy = y * y;
//	float zz = z * z;
//	float norm = sqrtf(xx + yy + zz);
//
//	verboseln("Norm: %f", norm);

//	float pitch = RAD_TO_DEG(atan2f(x, z));

//	verboseln("Pitch (y): %f", pitch);
}

static void demoReadSerial()
{
	char data[16];
	while (1) {
		Serial_ReadStringCR(data, 16);
		println(">> %s", data);
		if (strstarts(data, "f=")) {
			uint8_t duty = strtol(&data[strlen("f=")], NULL, 10);
			L298N_Forward(duty);
		} else if (strstarts(data, "b=")) {
			uint8_t duty = strtol(&data[strlen("b=")], NULL, 10);
			L298N_Backward(duty);
		} else if (streq(data, "stop")) {
			L298N_Stop();
		} else if (strstarts(data, "psc=")) {
			uint16_t psc = strtol(&data[strlen("psc=")], NULL, 10);
			verboseln("psc=%u", psc);
			__HAL_TIM_SET_PRESCALER(&htim3, psc);
		} else if (strstarts(data, "arr=")) {
			uint16_t arr = strtol(&data[strlen("arr=")], NULL, 10);
			verboseln("arr=%u", arr);
			__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPrimaryTask */
/**
  * @brief  Function implementing the primaryTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPrimaryTask */
void StartPrimaryTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	LSM6DSL_EnableAccelerometer(LSM6DSL_208_HZ, LSM6DSL_FS_XL_2_G);
//	LSM6DSL_EnableGyroscope(LSM6DSL_12_HZ, LSM6DSL_FS_G_250_DPS);
	LSM6DSL_SetAccelerometerInterrupt(LSM6DSL_INT_1);
//	LSM6DSL_SetGyroscopeInterrupt(LSM6DSL_INT_1);

	struct { float x, y, z; } xl;

	while (1) {
		verboseln("Waiting for LSM6DSL data...");
		osEventFlagsWait(
				lsm6dslEventHandle, LSM6DSL_EVENT_INTERRUPT,
				osFlagsWaitAll, osWaitForever);
		verboseln("Received LSM6DSL data ready signal");

		uint8_t status;
		if (LSM6DSL_ReadStatus(&status) == HAL_OK) {
			if (status & LSM6DSL_REG_STATUS_BIT_XLDA) {
				if (LSM6DSL_ReadAccelerometer_g(&xl.x, &xl.y, &xl.z) != HAL_OK)
					continue;
				handleAccelerometerData(xl.x, xl.y, xl.z);
			}
		}
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSecondaryTask */
/**
* @brief Function implementing the secondaryTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSecondaryTask */
void StartSecondaryTask(void *argument)
{
  /* USER CODE BEGIN StartSecondaryTask */
	while (1) {
		println("Roll  (x): %f", roll);
		osDelay(100);
	}

  /* USER CODE END StartSecondaryTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

	verboseln("ERROR");
	while (1) {

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
