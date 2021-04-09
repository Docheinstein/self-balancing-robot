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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERBOSE_FMT(fmt) "{MAIN} " fmt

#define LSM6DSL_EVENT_DATA_READY	BIT(0)
#define ROBOT_EVENT_START			BIT(0)

// === MODALITIES ===

#define TUNING_OVER_SERIAL 1
#define PRINT_STEP_RESPONSE 1
#define PRINT_GYRO_X 0
#define PRINT_GYRO_Y 0
#define PRINT_GYRO_Z 0
#define SIMULATION 0

// === TUNING PARAMETERS ===

// These two must be consistent each other
#define LSM6DSL_FREQ LSM6DSL_104_HZ
#define SAMPLE_RATE 104.0f

#define GYRO_OFFSET_X  0.41530f
#define GYRO_OFFSET_Y -1.04329f
#define GYRO_OFFSET_Z  1.19337f

#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f

#define COMPLEMENTARY_FILTER_TIME_CONSTANT 3.0f
#define MADGWICK_FILTER_BETA 0.1f

#define PID_KP  10.0f
#define PID_KI 22.0f
#define PID_KD  0.6f

#define MOTOR_DUTY_CYCLE_MIN   0.0f
#define MOTOR_DUTY_CYCLE_MAX 100.0f

#define SETPOINT_DEG  0.0f
#define GIVE_UP_DEG  40.0f

#define FORWARD_SPEED_FACTOR  1.00f
//#define BACKWARD_SPEED_FACTOR 0.96f
#define BACKWARD_SPEED_FACTOR 1.00f

//#define MOTOR_A_SPEED_FACTOR 1.12f
#define MOTOR_A_SPEED_FACTOR 1.06f
#define MOTOR_B_SPEED_FACTOR 1.00f

#define USE_MADGWICK_FILTER false
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

static float kp = PID_KP;
static float ki = PID_KI;
static float kd = PID_KD;
static float tau = COMPLEMENTARY_FILTER_TIME_CONSTANT;
static float beta = MADGWICK_FILTER_BETA;
static float pwm_min = MOTOR_DUTY_CYCLE_MIN;
static float pwm_max = MOTOR_DUTY_CYCLE_MAX;
static float setpoint = SETPOINT_DEG;
static float giveup = GIVE_UP_DEG;
static float forward_factor = FORWARD_SPEED_FACTOR;
static float backward_factor = BACKWARD_SPEED_FACTOR;
static bool use_madgwick = USE_MADGWICK_FILTER;
static bool simulation = SIMULATION;
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
  initializeRobot();
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
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 53332;
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
  HAL_GPIO_WritePin(GPIOA, Motor_A_Direction_2_Pin|Motor_A_Direction_1_Pin|Motor_B_Direction_2_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_B_Direction_1_GPIO_Port, Motor_B_Direction_1_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : Motor_A_Direction_2_Pin Motor_A_Direction_1_Pin Motor_B_Direction_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = Motor_A_Direction_2_Pin|Motor_A_Direction_1_Pin|Motor_B_Direction_2_Pin|LED_1_Pin;
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

  /*Configure GPIO pin : Motor_B_Direction_1_Pin */
  GPIO_InitStruct.Pin = Motor_B_Direction_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_B_Direction_1_GPIO_Port, &GPIO_InitStruct);

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
	}
	if (pin == Button_Blue_Pin) {
		verboseln("Button has been pressed");
		osEventFlagsSet(robotEventHandle, ROBOT_EVENT_START);
	}

}

static void handleSensorsData(uint32_t time, dim3_f xl, dim3_f g)
{
	verboseln("Accelerometer: (x=%f, y=%f, z=%f)g", xl.x, xl.y, xl.z);
	verboseln("Gyroscope:     (x=%f, y=%f, z=%f)dps", g.x, g.y, g.z);

#if PRINT_GYRO_X
	println("%f", g.x);
#endif

#if PRINT_GYRO_Y
	println("%f", g.y);
#endif

#if PRINT_GYRO_Z
	println("%f", g.z);
#endif

	// Apply adjustments to sensors measurements
	xl.x -= ACCEL_OFFSET_X;
	xl.y -= ACCEL_OFFSET_Y;
	xl.z -= ACCEL_OFFSET_Z;
	g.x -= GYRO_OFFSET_X;
	g.y -= GYRO_OFFSET_Y;
	g.z -= GYRO_OFFSET_Z;

	// Estimate the angle using measures from both sensors with a filter
	float input;
	if (use_madgwick)
		MadgwickFilter_Compute(xl, g, &input, NULL, NULL);
	else
		ComplementaryFilter_Compute(xl, g, &input, NULL, NULL);

	if (ABS(input) > giveup) {
		L298N_Stop(L298N_MOTOR_BOTH);
		return; // no change of balance
	}

	// Compute the motor's output using a PID
	float output = PID_Compute(time, input);

	// Eventually increase the speed depending on the direction
	bool go_forward = output < 0;
	float direction_speed_factor = go_forward ? forward_factor : backward_factor;
	output = output * direction_speed_factor;

	// Eventually remap output to the range [pwm_min, pwm_max]
	float duty_cycle = mapf(
		rangef(ABS(output), 0.0f, 100.0f),
		0.0f, 100.0f,
		pwm_min, pwm_max
	);

	if (!simulation) {
		if (go_forward)
			L298N_Forward(L298N_MOTOR_BOTH, duty_cycle);
		else
			L298N_Backward(L298N_MOTOR_BOTH, duty_cycle);
	}


#if PRINT_STEP_RESPONSE
	println("%u %f %f", time, input, copysignf(duty_cycle, output));
#endif
}


static void tuningOverSerial()
{
	println("==== TUNING ON SERIAL ====");
	println("Type h for help...");

	static const char *HELP =
		"<ENTER> : print parameters; twice to end tuning" SERIAL_ENDL
		"h       : help" SERIAL_ENDL
		"z       : simulation mode" SERIAL_ENDL
		".       : toggle filter" SERIAL_ENDL
		"p<VALUE>: PID_KP" SERIAL_ENDL
		"d<VALUE>: PID_KD" SERIAL_ENDL
		"i<VALUE>: PID_KI" SERIAL_ENDL
		"t<VALUE>: COMPLEMENTARY_FILTER_TAU" SERIAL_ENDL
		"b<VALUE>: MADGWICK_FILTER_BETA" SERIAL_ENDL
		"o<VALUE>: PWM_MIN" SERIAL_ENDL
		"c<VALUE>: SETPOINT" SERIAL_ENDL
		"F<VALUE>: MOTOR_FORWARD_FACTOR" SERIAL_ENDL
		"B<VALUE>: MOTOR_BACKWARD_FACTOR" SERIAL_ENDL
		"+<VALUE>: manually forward motor" SERIAL_ENDL
		"-<VALUE>: manually backward motor" SERIAL_ENDL
		"0       : manually stop motors"
	;

	bool just_print = false;
	bool exit = false;
	char data[16];
	while (!exit) {
		printf("$ ");
		Serial_ReadStringCR(data, 16);

		char cmd = '\0';
		char *args = NULL;
		size_t len = strlen(data);
		if (len > 0) {
			cmd = data[0];
			if (len > 1)
				args = &data[1];
		}

		// Tuning options
		switch (cmd) {
		case 'p': kp = strtof(args, NULL); break;
		case 'd': kd = strtof(args, NULL); break;
		case 'i': ki = strtof(args, NULL); break;
		case 't': tau = strtof(args, NULL); break;
		case 'b': beta = strtof(args, NULL); break;
		case 'o': pwm_min = strtof(args, NULL); break;
		case 'c': setpoint = strtof(args, NULL); break;
		case 'F': forward_factor = strtof(args, NULL); break;
		case 'B': backward_factor = strtof(args, NULL); break;
		case '+': L298N_Forward(L298N_MOTOR_BOTH, strtof(args, NULL)); break;
		case '-': L298N_Backward(L298N_MOTOR_BOTH, strtof(args, NULL)); break;
		case '0': L298N_Stop(L298N_MOTOR_BOTH); break;
		case '.': use_madgwick = !use_madgwick; break;
		case 'z': simulation = true; break;
		case 'h': println("%s", HELP); break;
		case '\0':
			if (just_print)
				exit = true;
			else
				printRobotParameters();
			break;
		default:
			println("ERROR: %s", data);
		}

		just_print = cmd == '\0';
	}

	println("==== TUNING ON SERIAL FINISHED ====");
}

static void printRobotParameters() {
	println(
		"# F         = %f Hz" SERIAL_ENDL
		"# tau       = %f s"SERIAL_ENDL
		"# alpha     = %f"SERIAL_ENDL
		"# beta      = %f"SERIAL_ENDL
		"# Kp        = %f"SERIAL_ENDL
		"# Ki        = %f" SERIAL_ENDL
		"# Kd        = %f" SERIAL_ENDL
		"# pwm_min   = %f %%" SERIAL_ENDL
		"# pwm_max   = %f %%" SERIAL_ENDL
		"# setpoint  = %f deg" SERIAL_ENDL
		"# giveup    = %f deg" SERIAL_ENDL
		"# x fwd     = %f" SERIAL_ENDL
		"# x bwd     = %f " SERIAL_ENDL
		"# x A       = %f " SERIAL_ENDL
		"# x B       = %f " SERIAL_ENDL
		"# madgwick  = %s " SERIAL_ENDL
		"# simul.    = %s",
		SAMPLE_RATE,
		tau,
		COMPLEMENTARY_FILTER_ALPHA(tau, SAMPLE_RATE),
		beta,
		kp, ki, kd,
		pwm_min, pwm_max,
		setpoint,
		giveup,
		forward_factor,
		backward_factor,
		MOTOR_A_SPEED_FACTOR,
		MOTOR_B_SPEED_FACTOR,
		BOOL_TO_STR(use_madgwick),
		BOOL_TO_STR(simulation)
	);
}

static void initializeRobot()
{
	verboseln("=============================");
	verboseln("Peripherals initialization...");

	// LSM6DSL (accelerometer + gyroscope)
	LSM6DSL_Init(&hi2c2);

	LSM6DSL_EnableAccelerometer(LSM6DSL_FREQ, LSM6DSL_FS_XL_2_G);
	LSM6DSL_EnableGyroscope(LSM6DSL_FREQ, LSM6DSL_FS_G_250_DPS);
	LSM6DSL_SetGyroscopeInterrupt(LSM6DSL_INT_1);

	// L298N (motors driver)
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
			.speed_factor = MOTOR_A_SPEED_FACTOR
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
			.speed_factor = MOTOR_B_SPEED_FACTOR
		},
	};

	L298N_Init(l298n_config);

	// Filters
	ComplementaryFilter_Config compl_filter_config = {
		.alpha = COMPLEMENTARY_FILTER_ALPHA(tau, SAMPLE_RATE),
		.sample_rate = SAMPLE_RATE
	};

	ComplementaryFilter_Init(compl_filter_config);

	MadgwickFilter_Config madgwick_filter_config = {
		.beta = beta,
		.sample_rate = SAMPLE_RATE
	};

	MadgwickFilter_Init(madgwick_filter_config);

	// PID
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

	GPIO_Pin_High(led1);

#if TUNING_OVER_SERIAL
	tuningOverSerial();
#endif

	GPIO_Pin_High(led2);

	// Do not start immediately: wait for the button to be pressed
	osEventFlagsWait(robotEventHandle, ROBOT_EVENT_START,
					 osFlagsWaitAll, osWaitForever);

	GPIO_Pin_Low(led1);
	GPIO_Pin_Low(led2);

	printRobotParameters();
	initializeRobot();

	static const uint32_t DISCARD_FIRST_SAMPLES = 5;
	uint32_t samples = 0;

	while (1) {
		verboseln("Waiting for LSM6DSL data (time = %u)...", HAL_GetTick());
		osEventFlagsWait(lsm6dslEventHandle, LSM6DSL_EVENT_DATA_READY,
						 osFlagsWaitAll, osWaitForever);
		verboseln("Received LSM6DSL data ready signal (time = %u)", HAL_GetTick());

		uint32_t tick = HAL_GetTick();

		uint8_t status;
		dim3_f xl, g;
		bool xl_ok, g_ok;

		if (LSM6DSL_ReadStatus(&status) == HAL_OK) {
			verboseln("status = 0x%02X", status);

			if (status & LSM6DSL_REG_STATUS_BIT_XLDA)
				xl_ok = LSM6DSL_ReadAccelerometer_g(&xl) == HAL_OK;

			if (status & LSM6DSL_REG_STATUS_BIT_GDA)
				g_ok = LSM6DSL_ReadGyroscope_dps(&g) == HAL_OK;

			verboseln("XL = %s | G = %s", BOOL_TO_STR(xl_ok), BOOL_TO_STR(g_ok));
			if (xl_ok && g_ok) {
				samples++;
				if (samples >= DISCARD_FIRST_SAMPLES) {
					handleSensorsData(tick, xl, g);
				}
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
