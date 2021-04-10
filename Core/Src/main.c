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

#include "math.h"
#include "verbose.h"
#include "serial.h"
#include "std.h"
#include "lsm6dsl.h"
#include "l298n.h"
#include "pid.h"
#include "complementary_filter.h"
#include "madgwick_filter.h"
#include "gpio_pin.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

typedef enum {
	FILTER_TYPE_COMPLEMENTARY,
	FILTER_TYPE_MADGWICK
} FilterType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERBOSE_FMT(fmt) "{MAIN} " fmt

#define LSM6DSL_EVENT_DATA_READY	BIT(0)
#define ROBOT_EVENT_START			BIT(0)
#define ROBOT_EVENT_TUNING			BIT(1)

// === TUNING PARAMETERS ===

// These two must be consistent each other
#define LSM6DSL_FREQ LSM6DSL_104_HZ
#define SAMPLE_RATE          104.0f

#define GYRO_OFFSET_X  0.41530f
#define GYRO_OFFSET_Y -1.04329f
#define GYRO_OFFSET_Z  1.19337f

#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f

#define FILTER_TYPE FILTER_TYPE_COMPLEMENTARY

#define COMPLEMENTARY_FILTER_TAU 3.00f
#define MADGWICK_FILTER_BETA     0.05f

#define PID_KP 11.2f
#define PID_KI 38.0f
#define PID_KD  0.6f

#define MOTOR_DUTY_CYCLE_MIN   0.0f
#define MOTOR_DUTY_CYCLE_MAX 100.0f

#define MOTOR_A_SPEED_FACTOR 1.06f
#define MOTOR_B_SPEED_FACTOR 1.00f

#define MOTORS_FORWARD_SPEED_FACTOR  1.00f
#define MOTORS_BACKWARD_SPEED_FACTOR 1.00f

#define DEG_SETPOINT  0.0f
#define DEG_GIVEUP   40.0f


#define PRINT_STEP_RESPONSE 1
#define SIMULATION false

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COMPLEMENTARY_FILTER_ALPHA(tau, rate) \
	((float) tau / (tau + ((float) 1 / rate)))

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
/* Definitions for robotEvent */
osEventFlagsId_t robotEventHandle;
osStaticEventGroupDef_t robotEventControlBlock;
const osEventFlagsAttr_t robotEvent_attributes = {
  .name = "robotEvent",
  .cb_mem = &robotEventControlBlock,
  .cb_size = sizeof(robotEventControlBlock),
};
/* USER CODE BEGIN PV */
static GPIO_Pin led1 = { .port = LED_1_GPIO_Port, .pin = LED_1_Pin };
static GPIO_Pin led2 = { .port = LED_2_GPIO_Port, .pin = LED_2_Pin };

// Copy the default values into variables since these
// could be modified while in tuning mode over serial
static FilterType filter_type = FILTER_TYPE;
static float tau = COMPLEMENTARY_FILTER_TAU;
static float beta = MADGWICK_FILTER_BETA;
static float kp = PID_KP;
static float ki = PID_KI;
static float kd = PID_KD;
static float pwm_min = MOTOR_DUTY_CYCLE_MIN;
static float pwm_max = MOTOR_DUTY_CYCLE_MAX;
static float forward_factor = MOTORS_FORWARD_SPEED_FACTOR;
static float backward_factor = MOTORS_BACKWARD_SPEED_FACTOR;
static float a_factor = MOTOR_A_SPEED_FACTOR;
static float b_factor = MOTOR_B_SPEED_FACTOR;
static float setpoint = DEG_SETPOINT;
static float giveup = DEG_GIVEUP;
static bool simulation = SIMULATION;
static bool print_step_response = PRINT_STEP_RESPONSE;
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
static void printRobotParameters();
static void initializeRobot();
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
  println(
	  "======= SELF BALANCING ROBOT =======" SERIAL_ENDL
	  "Build date: " __DATE__ " " __TIME__ SERIAL_ENDL
	  "===================================="
  );

  verboseln("Starting FreeRTOS kernel...");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of robotStateMutex */
  robotStateMutexHandle = osMutexNew(&robotStateMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
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

  /* creation of robotEvent */
  robotEventHandle = osEventFlagsNew(&robotEvent_attributes);

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
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
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
  HAL_GPIO_WritePin(GPIO_High_GPIO_Port, GPIO_High_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_Low_GPIO_Port, GPIO_Low_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_A_Direction_2_Pin|Motor_A_Direction_1_Pin|Motor_B_Direction_2_Pin|Motor_B_Direction_1_Pin
                          |LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Blue_Pin */
  GPIO_InitStruct.Pin = Button_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_High_Pin GPIO_Low_Pin */
  GPIO_InitStruct.Pin = GPIO_High_Pin|GPIO_Low_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_A_Direction_2_Pin Motor_A_Direction_1_Pin Motor_B_Direction_2_Pin Motor_B_Direction_1_Pin
                           LED_1_Pin */
  GPIO_InitStruct.Pin = Motor_A_Direction_2_Pin|Motor_A_Direction_1_Pin|Motor_B_Direction_2_Pin|Motor_B_Direction_1_Pin
                          |LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_2_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_INT_1_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_External_1_Pin */
  GPIO_InitStruct.Pin = Button_External_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Button_External_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
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
#endif

	if (pin == LSM6DSL_INT_1_Pin) {
		verboseln("Received LSM6DSL data-ready signal");
		osEventFlagsSet(lsm6dslEventHandle, LSM6DSL_EVENT_DATA_READY);
	} else if (pin == Button_Blue_Pin) {
		verboseln("Button has been pressed (internal)");
		osEventFlagsSet(robotEventHandle, ROBOT_EVENT_START);
	} else if (pin == Button_External_1_Pin) {
		verboseln("Button has been pressed (external)");
		osEventFlagsSet(robotEventHandle, ROBOT_EVENT_TUNING);
	}
}

static void initializeSensors()
{
	LSM6DSL_Init(&hi2c2);

	LSM6DSL_EnableAccelerometer(LSM6DSL_FREQ, LSM6DSL_FS_XL_2_G);
	LSM6DSL_EnableGyroscope(LSM6DSL_FREQ, LSM6DSL_FS_G_250_DPS);
	LSM6DSL_SetGyroscopeInterrupt(LSM6DSL_INT_1);
}

static void initializeMotors()
{
	L298N_Config l298n_config = {
		.motor_a = {
			.inverted = true, // depends on the wiring
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
			},
			.speed_factor = a_factor
		},
		.motor_b = {
			.inverted = false, // depends on the wiring
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
			},
			.speed_factor = b_factor
		},
	};

	L298N_Init(l298n_config);
}

static void initializeFilter()
{
	if (filter_type == FILTER_TYPE_COMPLEMENTARY) {
		ComplementaryFilter_Config compl_filter_config = {
			.alpha = COMPLEMENTARY_FILTER_ALPHA(tau, SAMPLE_RATE),
			.sample_rate = SAMPLE_RATE
		};

		ComplementaryFilter_Init(compl_filter_config);
	} else {
		MadgwickFilter_Config madgwick_filter_config = {
			.beta = beta,
			.sample_rate = SAMPLE_RATE
		};

		MadgwickFilter_Init(madgwick_filter_config);
	}
}

static void initializePID()
{
	PID_Config pid_config = {
		.Kp = kp,
		.Ki = ki,
		.Kd = kd,
		.setpoint = setpoint,
		.sample_rate = SAMPLE_RATE,
		.limit_output = false,
		.direction = PID_DIRECTION_DIRECT
	};

	PID_Init(pid_config);
}

static void initializeRobot()
{
	verboseln("=============================");
	verboseln("Robot initialization...");
	initializeSensors();
	initializeMotors();
	initializeFilter();
	initializePID();
	verboseln("=============================");
}

static void tuningOverSerial()
{
	println("========== TUNING ==========");
	println("Type h for help...");

	static const char *HELP =
		"<ENTER> : print parameters; twice to end tuning" SERIAL_ENDL
		"h       : help" SERIAL_ENDL
		"+<VALUE>: manually forward motor" SERIAL_ENDL
		"-<VALUE>: manually backward motor" SERIAL_ENDL
		"0       : manually stop motors" SERIAL_ENDL
		".       : FILTER_TYPE (toggle)" SERIAL_ENDL
		"t<VALUE>: COMPLEMENTARY_FILTER_TAU" SERIAL_ENDL
		"b<VALUE>: MADGWICK_FILTER_BETA" SERIAL_ENDL
		"p<VALUE>: PID_KP" SERIAL_ENDL
		"d<VALUE>: PID_KD" SERIAL_ENDL
		"i<VALUE>: PID_KI" SERIAL_ENDL
		"o<VALUE>: MOTOR_DUTY_CYCLE_MIN" SERIAL_ENDL
		"O<VALUE>: MOTOR_DUTY_CYCLE_MAX" SERIAL_ENDL
		"><VALUE>: MOTORS_FORWARD_SPEED_FACTOR" SERIAL_ENDL
		"<<VALUE>: MOTORS_BACKWARD_SPEED_FACTOR" SERIAL_ENDL
		"A<VALUE>: MOTOR_A_SPEED_FACTOR" SERIAL_ENDL
		"B<VALUE>: MOTOR_B_SPEED_FACTOR" SERIAL_ENDL
		"c<VALUE>: SETPOINT" SERIAL_ENDL
		"z       : SIMULATION_MODE (toggle)" SERIAL_ENDL
		"s       : PRINT_STEP_RESPONSE (toggle)"
	;

	bool already_printed = false;
	bool exit = false;
	char data[16];
	while (!exit) {
		printf("$ ");
		Serial_ReadStringCR(data, 16);

		char cmd = '\0';
		float arg = 0;
		size_t len = strlen(data);
		if (len > 0) {
			cmd = data[0];
			if (len > 1)
				arg = strtof(&data[1], NULL);
		}

		// Tuning options
		switch (cmd) {
		case '+':
			initializeMotors();
			L298N_Forward(L298N_MOTOR_BOTH, arg);
			break;
		case '-':
			initializeMotors();
			L298N_Backward(L298N_MOTOR_BOTH, arg);
			break;
		case '0':
			L298N_Stop(L298N_MOTOR_BOTH);
			break;
		case 'f': filter_type = filter_type == FILTER_TYPE_COMPLEMENTARY ?
					FILTER_TYPE_MADGWICK : FILTER_TYPE_COMPLEMENTARY; break;
		case 't': tau = arg; break;
		case 'b': beta = arg; break;
		case 'p': kp = arg; break;
		case 'd': kd = arg; break;
		case 'i': ki = arg; break;
		case 'm': pwm_min = arg; break;
		case 'M': pwm_max = arg; break;
		case '>': forward_factor = arg; break;
		case '<': backward_factor = arg; break;
		case 'A': a_factor = arg; break;
		case 'B': b_factor = arg; break;
		case 'c': setpoint = arg; break;
		case 'z': simulation = !simulation; break;
		case 's': print_step_response = !print_step_response; break;
		case 'h': println("%s", HELP); break;
		case '\0':
			if (!already_printed)
				printRobotParameters();
			else
				exit = true;
			break;
		default:
			println("ERROR: %s", data);
		}

		already_printed = cmd == '\0';
	}

	println("===== TUNING FINISHED =====");
}

static void printRobotParameters() {
	println(
		"# SAMPLE_RATE                  = %f Hz" SERIAL_ENDL
		"# (SAMPLE_TIME)                = %f s" SERIAL_ENDL
		"# FILTER_TYPE                  = %s"SERIAL_ENDL
		"# COMPLEMENTARY_FILTER_TAU     = %f"SERIAL_ENDL
		"# (COMPLEMENTARY_FILTER_ALPHA) = %f"SERIAL_ENDL
		"# MADGWICK_FILTER_BETA         = %f"SERIAL_ENDL
		"# PID_KP                       = %f"SERIAL_ENDL
		"# PID_KI                       = %f"SERIAL_ENDL
		"# PID_KD                       = %f"SERIAL_ENDL
		"# MOTOR_DUTY_CYCLE_MIN         = %f %%" SERIAL_ENDL
		"# MOTOR_DUTY_CYCLE_MAX         = %f %%" SERIAL_ENDL
		"# MOTORS_FORWARD_SPEED_FACTOR  = %f"SERIAL_ENDL
		"# MOTORS_BACKWARD_SPEED_FACTOR = %f"SERIAL_ENDL
		"# MOTOR_A_SPEED_FACTOR         = %f"SERIAL_ENDL
		"# MOTOR_B_SPEED_FACTOR         = %f"SERIAL_ENDL
		"# DEG_SETPOINT                 = %f deg"SERIAL_ENDL
		"# DEG_GIVUP                    = %f deg"SERIAL_ENDL
		"# SIMULATION                   = %s"SERIAL_ENDL
		"# PRINT_STEP_RESPONSE          = %s",
		SAMPLE_RATE,
		1 / SAMPLE_RATE,
		filter_type == FILTER_TYPE_COMPLEMENTARY ? "complementary" :  "madgwick",
		tau,
		COMPLEMENTARY_FILTER_ALPHA(tau, SAMPLE_RATE),
		beta,
		kp,
		ki,
		kd,
		pwm_min,
		pwm_max,
		forward_factor,
		backward_factor,
		a_factor,
		b_factor,
		setpoint,
		giveup,
		BOOL_TO_STR(simulation),
		BOOL_TO_STR(print_step_response)
	);
}

static void setLeds(bool l1, bool l2)
{
	verboseln("LED1: %s | LED2: %s", l1 ? "on" : "off", l2 ? "on" : "off");
	GPIO_Pin_Write(led1, l1);
	GPIO_Pin_Write(led2, l2);
}


static void handleSensorsMeasurement(dim3_f xl, dim3_f g)
{
	verboseln("Accelerometer: (x=%f, y=%f, z=%f)g", xl.x, xl.y, xl.z);
	verboseln("Gyroscope:     (x=%f, y=%f, z=%f)dps", g.x, g.y, g.z);

	// Apply adjustments to sensors measurements
	xl.x -= ACCEL_OFFSET_X;
	xl.y -= ACCEL_OFFSET_Y;
	xl.z -= ACCEL_OFFSET_Z;
	g.x -= GYRO_OFFSET_X;
	g.y -= GYRO_OFFSET_Y;
	g.z -= GYRO_OFFSET_Z;

	// Estimate the angle using measures from both sensors with a filter
	float input;
	if (filter_type == FILTER_TYPE_COMPLEMENTARY)
		ComplementaryFilter_Compute(xl, g, &input, NULL, NULL);
	else
		MadgwickFilter_Compute(xl, g, &input, NULL, NULL);

	// Do not even try to balance if the robot is too much sloped
	if (ABS(input) > giveup) {
		L298N_Stop(L298N_MOTOR_BOTH);
		return;
	}

	// Compute the motor's output using a PID
	float output = PID_Compute(input);

	// Eventually increase the speed depending on the direction
	bool go_forward = (output < 0);
	output *= go_forward ? forward_factor : backward_factor;

	// Eventually remap output to the range [pwm_min, pwm_max]
	float duty_cycle = mapf(
		rangef(ABS(output), 0.0f, 100.0f),
		0.0f, 100.0f,
		pwm_min, pwm_max
	);

	// Actually control the motors
	if (!simulation) {
		if (go_forward)
			L298N_Forward(L298N_MOTOR_BOTH, duty_cycle);
		else
			L298N_Backward(L298N_MOTOR_BOTH, duty_cycle);
	}

	if (print_step_response)
		println("%u %f %f", HAL_GetTick(), input, copysignf(duty_cycle, output));
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

	uint32_t flags;

	// Do not start immediately.
	// Wait for either the start or the tuning event.
	do {
		verboseln("Waiting for event...");
		setLeds(true, false);

		flags = osEventFlagsWait(
			robotEventHandle,
			ROBOT_EVENT_START | ROBOT_EVENT_TUNING,
			osFlagsWaitAny, osWaitForever
		);
		if (flags & ROBOT_EVENT_TUNING) {
			setLeds(true, true);
			tuningOverSerial();
		}
	} while (!(flags & ROBOT_EVENT_START));

	setLeds(false, false);

	// Actually start the robot main loop

	initializeRobot();

	static const uint32_t DISCARD_INITIAL_SAMPLES = 5;
	uint64_t samples = 0;
	uint8_t status;
	dim3_f xl, g;
	bool xl_ok, g_ok;

	verboseln("==== ROBOT MAIN LOOP ====");
	while (true) {
		verboseln("Waiting for LSM6DSL data (time = %u)...", HAL_GetTick());
		osEventFlagsWait(lsm6dslEventHandle, LSM6DSL_EVENT_DATA_READY,
						 osFlagsWaitAll, osWaitForever);
		verboseln("Received LSM6DSL data ready signal (time = %u)", HAL_GetTick());


		if (!LSM6DSL_ReadStatus(&status) == HAL_OK) {
			verboseln("Failed to read LSM6DSL status");
			continue;
		}

		verboseln("LSM6DSL status = 0x%02X", status);

		// Read the sensors
		if (status & LSM6DSL_REG_STATUS_BIT_XLDA)
			xl_ok = LSM6DSL_ReadAccelerometer_g(&xl) == HAL_OK;

		if (status & LSM6DSL_REG_STATUS_BIT_GDA)
			g_ok = LSM6DSL_ReadGyroscope_dps(&g) == HAL_OK;

		if (!(xl_ok && g_ok))
			continue;

		if (samples++ < DISCARD_INITIAL_SAMPLES)
			// the first samples usually have some noise; discard them
			continue;

		// Handle the sensors measurement
		handleSensorsMeasurement(xl, g);
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
//		println("Roll  (x): %f", roll);
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
