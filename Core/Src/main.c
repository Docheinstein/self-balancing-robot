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
#include "pid.h"
#include "complementary_filter.h"
#include "gpio_pin.h"
#include "a4988.h"
#include "utime.h"
#include "taskutils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct SerialCommand {
	char prefix;
	const char *arg_help;
	const char *help;
	void (*fn)(const char *);
} SerialCommand;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VERBOSE_FMT(fmt) fmt

/* ----- serial commands ----- */

#define CMD_PRINT_CONFIG_PREFIX '\0'
#define CMD_HELP_PREFIX 'h'
#define CMD_DEFAULT_SAMPLE_RATE_PREFIX 'r'
#define CMD_TAU_PREFIX 't'
#define CMD_KP_PREFIX 'p'
#define CMD_KI_PREFIX 'i'
#define CMD_KD_PREFIX 'd'
#define CMD_DEFAULT_SETPOINT_DEG_PREFIX 'c'
#define CMD_GYRO_X_OFFSET_PREFIX 'o'
#define CMD_DEFAULT_SIMULATION_MODE_PREFIX 'x'
#define CMD_DEFAULT_DISPLAY_STEP_RESPONSE_PREFIX 's'
#define CMD_DEFAULT_DISPLAY_MOTOR_RESPONSIVENESS_PREFIX 'm'
#define CMD_DEFAULT_ECHO_COMMANDS_PREFIX 'e'
#define CMD_STEP_MOTOR_A_PREFIX 'a'
#define CMD_STEP_MOTOR_B_PREFIX 'b'
#define CMD_SPIN_MOTORS_PREFIX 'g'
#define CMD_GYRO_AUTOTUNING_PREFIX 'u'
#define CMD_START_PREFIX '1'
#define CMD_STOP_PREFIX '0'


#if VERBOSE
#define CMD_VERBOSE_PREFIX 'v'
#endif

/* ----- fixed parameters ----- */

/*
 * Gyroscope adjustments.
 * The gyro offsets are computed as the mean
 * of the value read from the gyro while steady
 * (which ideally should be 0).
 * Actually only the X one is computed since the other are not used.
 */
#define GYRO_OFFSET_X  0.30f

/*
 * Accelerometer adjustments.
 * The acceleration offsets are 0 because we haven't
 * a more accurate accelerometer to compare
 * the obtained value; so we trust in what we read.
 */
#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f

/*
 * Granularity of the A4988s drivers.
 * (depends on the MSI configuration, which is fixed in hardware).
 */
#define MOTOR_MICROSTEPS_PER_STEP 2

/*
 * How many steps a revolution of the motor (NEMA17) contains.
 */
#define MOTOR_STEPS_PER_REV 200

/*
 * Degrees after which the robot is considered dead
 * (therefore won't be any output).
 */
#define GIVEUP_DEG 40.0f

/*
 * First sensors samples usually have some noise, discard some of them.
 */
#define LSM6DSL_INITIAL_SAMPLES_TO_DISCARD 30

/*
 * Software debounce to prevents too near consecutive presses.
 */
#define BUTTON_DEBOUNCE_MS 150

/*
 * Software debounce to prevents too near consecutive presses.
 */
#define BUTTON_LONG_PRESS_MS 2000


/*
 * After how many steps blink the LEDs.
 */
#define UI_BLINK_AFTER_MOTOR_STEPS 4


/*
 * After how many steps blink the LEDs.
 */
#define UI_BLINK_AFTER_AUTOTUNING_SENSOR_READ 4


/*
 * Command over serial max length.
 */
#define COMMAND_MAX_LEN 16


/* ----- runtime-configurable parameters ----- */

/*
 * Sampling sensor frequency.
 */
#define DEFAULT_SAMPLE_RATE 52
#define DEFAULT_LSM6DSL_FREQ LSM6DSL_52_HZ

/*
 * Complementary filter parameter.
 * Changing this value changes the threshold of when
 * we trust the gyroscope or we trust the accelerometer.
 */
#define DEFAULT_COMPLEMENTARY_FILTER_TAU 5.00f

/*
 * PID parameters.
 * P: proportional
 * I: integral
 * D: derivative
 */
#define DEFAULT_PID_KP  6.0f
#define DEFAULT_PID_KI  60.0f
#define DEFAULT_PID_KD  0.12f

/*
 * The desired angle;
 * can be adjusted if the hardware is not exactly balanced.
 */
#define DEFAULT_SETPOINT_DEG  0.0f

#define DEFAULT_SIMULATION_MODE false
#define DEFAULT_DISPLAY_STEP_RESPONSE false
#define DEFAULT_DISPLAY_MOTOR_RESPONSIVENESS false
#define DEFAULT_ECHO_COMMANDS true
#define DEFAULT_VERBOSE false

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define COMPLEMENTARY_FILTER_ALPHA(tau, rate) \
	((float) tau / (tau + ((float) 1 / rate)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId inputProcessorTaskHandle;
uint32_t inputProcessorTaskBuffer[ 256 ];
osStaticThreadDef_t inputProcessorTaskControlBlock;
osThreadId motorControllerTaskHandle;
uint32_t motorControllerTaskBuffer[ 256 ];
osStaticThreadDef_t motorControllerTaskControlBlock;
osThreadId uiTaskHandle;
uint32_t uiTaskBuffer[ 256 ];
osStaticThreadDef_t uiTaskControlBlock;
osThreadId serialTxFlushTaskHandle;
uint32_t serialTxFlushTaskBuffer[ 256 ];
osStaticThreadDef_t serialTxFlushTaskControlBlock;
osThreadId serialRxTaskHandle;
uint32_t serialRxTaskBuffer[ 256 ];
osStaticThreadDef_t serialRxTaskControlBlock;
osThreadId startStopTaskHandle;
uint32_t startStopTaskBuffer[ 256 ];
osStaticThreadDef_t startStopTaskControlBlock;
osThreadId autotuningTaskHandle;
uint32_t autotuningTaskBuffer[ 256 ];
osStaticThreadDef_t autotuningTaskControlBlock;
osMutexId sensorsMutexHandle;
osStaticMutexDef_t sensorsMutexControlBlock;
/* USER CODE BEGIN PV */

/* ----- conversion constants ----- */

static const float RPM_TO_FREQ = ((float) MOTOR_STEPS_PER_REV  * MOTOR_MICROSTEPS_PER_STEP) / 60.0f;
static const float FREQ_TO_RPM = 1 / RPM_TO_FREQ;

/* ----- signals ----- */

static const int START_STOP_TASK_SIGNAL_START = 					BIT(0);
static const int START_STOP_TASK_SIGNAL_STOP = 					BIT(1);
static const int START_STOP_TASK_SIGNAL_TOGGLE = 					BIT(2);

static const int INPUT_PROCESSOR_TASK_SIGNAL_INPUT_READY = 		BIT(0);
static const int INPUT_PROCESSOR_TASK_SIGNAL_RESET = 				BIT(1);

static const int UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_A =				BIT(0);
static const int UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_A =			BIT(1);
static const int UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_B =				BIT(2);
static const int UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_B =			BIT(3);
static const int UI_TASK_SIGNAL_AUTOTUNING =						BIT(4);

static const int MOTOR_CONTROLLER_TASK_SIGNAL_OUTPUT_READY = 		BIT(0);
static const int MOTOR_CONTROLLER_TASK_SIGNAL_RESET = 				BIT(1);

static const int AUTOTUNING_TASK_SIGNAL = 							BIT(0);

/* ----- peripherals ----- */

static GPIO_Pin led1 = { .port = LED_1_GPIO_Port, .pin = LED_1_Pin };
static GPIO_Pin led2 = { .port = LED_2_GPIO_Port, .pin = LED_2_Pin };
static GPIO_Pin button = { .port = Button_Blue_GPIO_Port, .pin = Button_Blue_Pin };

static A4988 a4988_a;
static A4988 a4988_b;
static PID pid;
static ComplementaryFilter filter;

/* ----- configurable parameters ----- */

// Some of these must be declared as volatile in order to be able to see the
// runtime updates that could eventually be done from the serial

bool verbose = DEFAULT_VERBOSE; // extern of verbose.h
static LSM6DSL_Frequency lsm6dsl_freq = DEFAULT_LSM6DSL_FREQ;
static int sample_rate = DEFAULT_SAMPLE_RATE;
static float tau = DEFAULT_COMPLEMENTARY_FILTER_TAU;
static float kp = DEFAULT_PID_KP;
static float ki = DEFAULT_PID_KI;
static float kd = DEFAULT_PID_KD;
static float setpoint_deg = DEFAULT_SETPOINT_DEG;
static bool simulation_mode = DEFAULT_SIMULATION_MODE;
static volatile bool display_step_response = DEFAULT_DISPLAY_STEP_RESPONSE;
static volatile bool display_motor_responsiveness = DEFAULT_DISPLAY_MOTOR_RESPONSIVENESS;
static volatile bool echo_commands = DEFAULT_ECHO_COMMANDS;
static volatile bool gyro_autotuning = false;
static volatile float gyro_offset_x = GYRO_OFFSET_X;
static volatile struct {
	float sum;
	int count;
} gyro_autotuning_mean;

/* ----- robot state  ----- */

static volatile bool running = false;
static volatile float target_rpm = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
void InputProcessorTask(void const * argument);
void MotorControllerTask(void const * argument);
void UITask(void const * argument);
void SerialTxFlushTask(void const * argument);
void SerialRxTask(void const * argument);
void StartStopTask(void const * argument);
void AutotuningTask(void const * argument);

/* USER CODE BEGIN PFP */
static void initializeFilter();
static void initializePID();

static void initializeMotorA();
static void startMotorA();
static void stopMotorB();
static void stepForwardMotorA(bool wait_falling_edge);
static void stepBackwardMotorA(bool wait_falling_edge);
static void stepFallingEdgeMotorA();

static void initializeMotorB();
static void startMotorB();
static void stopMotorB();
static void stepForwardMotorB(bool wait_falling_edge);
static void stepBackwardMotorB(bool wait_falling_edge);
static void stepFallingEdgeMotorB();

static void initializeMotors();
static void startMotors();
static void stopMotors();

static void initializeSensors();
static void startSensors();
static void stopSensors();

static void initializeRobot();
static void startRobot();
static void stopRobot();

static void cmdPrintConfig(const char *arg);
static void cmdHelp(const char *arg);
static void cmdSetSampleRate(const char *arg);
static void cmdSetTau(const char *arg);
static void cmdSetKp(const char *arg);
static void cmdSetKd(const char *arg);
static void cmdSetKi(const char *arg);
static void cmdSetSetpoint(const char *arg);
static void cmdSetGyroXOffset(const char *arg);
static void cmdSetOrToggleSimulationMode(const char *arg);
static void cmdSetOrToggleDisplayStepResponse(const char *arg);
static void cmdSetOrToggleDisplayMotorResponsiveness(const char *arg);
static void cmdSetOrToggleEchoCommands(const char *arg);
static void cmdStepMotorA(const char *arg);
static void cmdStepMotorB(const char *arg);
static void cmdSpinMotors(const char *arg);
static void cmdGyroAutotuning(const char *arg);
static void cmdStart(const char *arg);
static void cmdStop(const char *arg);

#if VERBOSE
static void cmdSetOrToggleVerbose(const char *arg);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


const SerialCommand COMMANDS[] = {
	{.prefix = CMD_PRINT_CONFIG_PREFIX,
			.arg_help = "",
			.help = "print configuration",
			.fn = cmdPrintConfig},
	{.prefix = CMD_HELP_PREFIX,
			.arg_help = "",
			.help = "show help",
			.fn = cmdHelp},
	{.prefix = CMD_DEFAULT_SAMPLE_RATE_PREFIX,
			.arg_help = "<INT>",
			.help = "set 'sample_rate'",
			.fn = cmdSetSampleRate},
	{.prefix = CMD_TAU_PREFIX,
			.arg_help = "<FLOAT>",
			.help = "set 'tau'",
			.fn = cmdSetTau},
	{.prefix = CMD_KP_PREFIX,
			.arg_help = "<FLOAT>",
			.help = "set 'kp'",
			.fn = cmdSetKp},
	{.prefix = CMD_KI_PREFIX,
			.arg_help = "<FLOAT>",
			.help = "set 'ki'",
			.fn = cmdSetKi},
	{.prefix = CMD_KD_PREFIX,
			.arg_help = "<FLOAT>",
			.help = "set 'kd'",
			.fn = cmdSetKd},
	{.prefix = CMD_DEFAULT_SETPOINT_DEG_PREFIX,
			.arg_help = "<FLOAT",
			.help = "set 'setpoint_deg'",
			.fn = cmdSetSetpoint},
	{.prefix = CMD_GYRO_X_OFFSET_PREFIX,
			.arg_help = "<FLOAT>",
			.help = "set the gyroscope x offset",
			.fn = cmdSetGyroXOffset},
	{.prefix = CMD_DEFAULT_SIMULATION_MODE_PREFIX,
			.arg_help = "[0|1]",
			.help = "set or toggle 'simulation_mode'",
			.fn = cmdSetOrToggleSimulationMode},
	{.prefix = CMD_DEFAULT_DISPLAY_STEP_RESPONSE_PREFIX,
			.arg_help = "[0|1]",
			.help = "set or toggle 'display_step_response'",
			.fn = cmdSetOrToggleDisplayStepResponse},
	{.prefix = CMD_DEFAULT_DISPLAY_MOTOR_RESPONSIVENESS_PREFIX,
			.arg_help = "[0|1]",
			.help = "set or toggle 'display_motor_responsiveness'",
			.fn = cmdSetOrToggleDisplayMotorResponsiveness},
	{.prefix = CMD_DEFAULT_ECHO_COMMANDS_PREFIX,
			.arg_help = "[0|1]",
			.help = "set or toggle 'echo_commands'",
			.fn = cmdSetOrToggleEchoCommands},
#if VERBOSE
	{.prefix = CMD_VERBOSE_PREFIX,
			.arg_help = "[0|1]",
			.help = "set or toggle 'verbose'",
			.fn = cmdSetOrToggleVerbose},
#endif
	{.prefix = CMD_STEP_MOTOR_A_PREFIX,
			.arg_help = "[f|b]",
			.help = "step forward motor A",
			.fn = cmdStepMotorA},
	{.prefix = CMD_STEP_MOTOR_B_PREFIX,
			.arg_help = "[f|b]",
			.help = "step forward motor A",
			.fn = cmdStepMotorB},
	{.prefix = CMD_SPIN_MOTORS_PREFIX,
			.arg_help = "<FLOAT>",
			.help = "start the motors with the given RPM",
			.fn = cmdSpinMotors},
	{.prefix = CMD_GYRO_AUTOTUNING_PREFIX,
			.arg_help = "<INT>",
			.help = "start the gyroscope tuning for the given seconds",
			.fn = cmdGyroAutotuning},
	{.prefix = CMD_START_PREFIX,
			.arg_help = "",
			.help = "start the self-balancing robot",
			.fn = cmdStart},
	{.prefix = CMD_STOP_PREFIX,
			.arg_help = "",
			.help = "stop the self-balancing robot",
			.fn = cmdStop}
};

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
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  UTime_Init(&htim2);
  UTime_Start();

  Serial_Config serial_cfg = { .huart = &huart1 };
  Serial_Init(serial_cfg);

  println(
	  "======= SELF BALANCING ROBOT =======" SERIAL_ENDL
	  "Build date: " __DATE__ " " __TIME__ SERIAL_ENDL
	  "===================================="
  );
  println("Type h for help...");


  averboseln("Starting FreeRTOS kernel...");

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of sensorsMutex */
  osMutexStaticDef(sensorsMutex, &sensorsMutexControlBlock);
  sensorsMutexHandle = osMutexCreate(osMutex(sensorsMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of inputProcessorTask */
  osThreadStaticDef(inputProcessorTask, InputProcessorTask, osPriorityHigh, 0, 256, inputProcessorTaskBuffer, &inputProcessorTaskControlBlock);
  inputProcessorTaskHandle = osThreadCreate(osThread(inputProcessorTask), NULL);

  /* definition and creation of motorControllerTask */
  osThreadStaticDef(motorControllerTask, MotorControllerTask, osPriorityHigh, 0, 256, motorControllerTaskBuffer, &motorControllerTaskControlBlock);
  motorControllerTaskHandle = osThreadCreate(osThread(motorControllerTask), NULL);

  /* definition and creation of uiTask */
  osThreadStaticDef(uiTask, UITask, osPriorityLow, 0, 256, uiTaskBuffer, &uiTaskControlBlock);
  uiTaskHandle = osThreadCreate(osThread(uiTask), NULL);

  /* definition and creation of serialTxFlushTask */
  osThreadStaticDef(serialTxFlushTask, SerialTxFlushTask, osPriorityBelowNormal, 0, 256, serialTxFlushTaskBuffer, &serialTxFlushTaskControlBlock);
  serialTxFlushTaskHandle = osThreadCreate(osThread(serialTxFlushTask), NULL);

  /* definition and creation of serialRxTask */
  osThreadStaticDef(serialRxTask, SerialRxTask, osPriorityBelowNormal, 0, 256, serialRxTaskBuffer, &serialRxTaskControlBlock);
  serialRxTaskHandle = osThreadCreate(osThread(serialRxTask), NULL);

  /* definition and creation of startStopTask */
  osThreadStaticDef(startStopTask, StartStopTask, osPriorityRealtime, 0, 256, startStopTaskBuffer, &startStopTaskControlBlock);
  startStopTaskHandle = osThreadCreate(osThread(startStopTask), NULL);

  /* definition and creation of autotuningTask */
  osThreadStaticDef(autotuningTask, AutotuningTask, osPriorityNormal, 0, 256, autotuningTaskBuffer, &autotuningTaskControlBlock);
  autotuningTaskHandle = osThreadCreate(osThread(autotuningTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, Motor_A_Sleep_Pin|Motor_A_Direction_Pin|Motor_B_Direction_Pin|Motor_B_Sleep_Pin
                          |LED_1_Pin|Motor_B_Step_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor_A_Step_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Blue_Pin */
  GPIO_InitStruct.Pin = Button_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_A_Sleep_Pin Motor_A_Direction_Pin Motor_B_Direction_Pin Motor_B_Sleep_Pin
                           LED_1_Pin Motor_B_Step_Pin */
  GPIO_InitStruct.Pin = Motor_A_Sleep_Pin|Motor_A_Direction_Pin|Motor_B_Direction_Pin|Motor_B_Sleep_Pin
                          |LED_1_Pin|Motor_B_Step_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_A_Step_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = Motor_A_Step_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LSM6DSL_INT_1_Pin */
  GPIO_InitStruct.Pin = LSM6DSL_INT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LSM6DSL_INT_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	static uint32_t button_last_t = 0;
	uint32_t t = GetMilliseconds();

	if (pin == LSM6DSL_INT_1_Pin) {
		// SENSORS event (wake up InputProcessorTask)
		osSignalSet(inputProcessorTaskHandle, INPUT_PROCESSOR_TASK_SIGNAL_INPUT_READY);
	} else if (pin == Button_Blue_Pin) {
		// BUTTON event
		bool rising = GPIO_Pin_Read(button);
		if (rising) {
			if (t - button_last_t > BUTTON_LONG_PRESS_MS) {
				// Long press: auto tuning
				osSignalSet(autotuningTaskHandle, AUTOTUNING_TASK_SIGNAL);
			} else {
				// Short press: start/stop
				osSignalSet(startStopTaskHandle, START_STOP_TASK_SIGNAL_TOGGLE);
			}
		}

		button_last_t = t;
	}
}

static void initializeFilter()
{
	ComplementaryFilter_Config cfg = {
		.alpha = COMPLEMENTARY_FILTER_ALPHA(tau, sample_rate),
		.sample_rate = (float) sample_rate
	};

	ComplementaryFilter_Init(&filter, cfg);
}

static void initializePID()
{
	PID_Config cfg = {
		.Kp = kp,
		.Ki = ki,
		.Kd = kd,
		.setpoint = setpoint_deg,
		.sample_rate = (float) sample_rate,
		.limit_output = true,
		.output_limits = {
			/* Better not to exceed 1000Hz. */
			.min = -995 * FREQ_TO_RPM,
			.max = +995 * FREQ_TO_RPM
		},
		.direction = PID_DIRECTION_DIRECT
	};

	PID_Init(&pid, cfg);
}

static void initializeMotorA()
{
	A4988_Config cfg_a = {
		.step = {
			.port = Motor_A_Step_GPIO_Port,
			.pin = Motor_A_Step_Pin
		},
		.direction = {
			.port = Motor_A_Direction_GPIO_Port,
			.pin = Motor_A_Direction_Pin
		},
		.sleep = {
			.port = Motor_A_Sleep_GPIO_Port,
			.pin = Motor_A_Sleep_Pin
		},
		.inverted = true,
	};

	A4988_Init(&a4988_a, cfg_a);
}

static void startMotorA()
{
	if (!simulation_mode)
		A4988_Enable(&a4988_a);
	else
		a4988_a.enabled = true;
}

static void stopMotorA()
{
	if (!simulation_mode)
		A4988_Disable(&a4988_a);
	else
		a4988_a.enabled = false;
}

static void stepForwardMotorA(bool wait_falling_edge)
{
	if (a4988_a.enabled) {
		if (!simulation_mode) {
			A4988_StepForward(&a4988_a, wait_falling_edge);
		}
		osSignalSet(uiTaskHandle, UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_A);
	}
}

static void stepBackwardMotorA(bool wait_falling_edge)
{
	if (a4988_a.enabled) {
		if (!simulation_mode)
			A4988_StepBackward(&a4988_a, wait_falling_edge);
		osSignalSet(uiTaskHandle, UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_A);
	}
}

static void stepFallingEdgeMotorA()
{
	if (a4988_a.enabled) {
		if (!simulation_mode)
			A4988_StepFallingEdge(&a4988_a);
	}
}

static void initializeMotorB()
{
	A4988_Config cfg_b = {
		.step = {
			.port = Motor_B_Step_GPIO_Port,
			.pin = Motor_B_Step_Pin
		},
		.direction = {
			.port = Motor_B_Direction_GPIO_Port,
			.pin = Motor_B_Direction_Pin
		},
		.sleep = {
			.port = Motor_B_Sleep_GPIO_Port,
			.pin = Motor_B_Sleep_Pin
		},
		.inverted = false,
	};

	A4988_Init(&a4988_b, cfg_b);
}

static void startMotorB()
{
	if (!simulation_mode)
		A4988_Enable(&a4988_b);
	else
		a4988_b.enabled = true;
}

static void stopMotorB()
{
	if (!simulation_mode)
		A4988_Disable(&a4988_b);
	else
		a4988_a.enabled = false;
}

static void stepForwardMotorB(bool wait_falling_edge)
{
	if (a4988_b.enabled) {
		if (!simulation_mode)
			A4988_StepForward(&a4988_b, wait_falling_edge);
		osSignalSet(uiTaskHandle, UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_B);
	}
}

static void stepBackwardMotorB(bool wait_falling_edge)
{
	if (a4988_b.enabled) {
		if (!simulation_mode)
			A4988_StepBackward(&a4988_b, wait_falling_edge);
		osSignalSet(uiTaskHandle, UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_B);
	}
}

static void stepFallingEdgeMotorB()
{
	if (!simulation_mode)
		A4988_StepFallingEdge(&a4988_b);
}


static void initializeMotors()
{
	initializeMotorA();
	initializeMotorB();
}

static void startMotors()
{
	startMotorA();
	startMotorB();
	osDelay(A4988_MINIMUM_STEP_AFTER_WAKE_UP_MILLISECONDS);
}

static void stopMotors()
{
	stopMotorA();
	stopMotorB();
}

static void initializeSensors()
{
	osMutexWait(sensorsMutexHandle, osWaitForever);
	LSM6DSL_Init(&hi2c2);
	osMutexRelease(sensorsMutexHandle);
}

static void startSensors()
{
	osMutexWait(sensorsMutexHandle, osWaitForever);
	bool ok =
		LSM6DSL_EnableAccelerometer(lsm6dsl_freq, LSM6DSL_FS_XL_2_G) == HAL_OK &&
		LSM6DSL_EnableGyroscope(lsm6dsl_freq, LSM6DSL_FS_G_250_DPS) == HAL_OK &&
		LSM6DSL_SetGyroscopeInterrupt(LSM6DSL_INT_1) == HAL_OK;
	osMutexRelease(sensorsMutexHandle);

	if (!ok) {
		println("ERROR: sensors communication failed");
		Error_Handler();
	}
}

static void stopSensors()
{
	osMutexWait(sensorsMutexHandle, osWaitForever);
	bool ok =
		LSM6DSL_DisableAccelerometer() == HAL_OK &&
		LSM6DSL_DisableGyroscope() == HAL_OK &&
		LSM6DSL_UnsetGyroscopeInterrupt(LSM6DSL_INT_1) == HAL_OK;
	osMutexRelease(sensorsMutexHandle);

	if (!ok) {
		println("ERROR: sensors communication failed");
		Error_Handler();
	}
}

static void initializeRobot()
{
	averboseln("Initializing robot...");
	initializeSensors();
	initializeMotors();
	initializeFilter();
	initializePID();
}

static void startRobot()
{
	aprintln("=== START ===");
	startSensors();
	startMotors();
}

static void stopRobot()
{
	aprintln("=== STOP ===");
	stopSensors();
	stopMotors();
}


static void gyroAutotuning(int seconds) {
	int ms = seconds * 1000;
	gyro_autotuning = true;
	gyro_autotuning_mean.sum = 0;
	gyro_autotuning_mean.count = 0;
	aprintln("----------------------------");
	aprintln("--- GYROSCOPE AUTOTUNING ---");
	aprintln("----------------------------");
	initializeSensors();
	startSensors();

	uint32_t start = GetMilliseconds();
	uint32_t end = start + ms;
	while (true) {
		osDelay(2000);
		uint32_t now = GetMilliseconds();
		if (now > end)
			break;
		float progress = 100.0f * (now - start) / ms;
		aprintln("[%.1f%%] %f", progress, gyro_autotuning_mean.sum / gyro_autotuning_mean.count);
	}

	stopSensors();
	osSignalSet(inputProcessorTaskHandle, INPUT_PROCESSOR_TASK_SIGNAL_RESET);
	gyro_autotuning = false;
	gyro_offset_x = gyro_autotuning_mean.sum / gyro_autotuning_mean.count;
	aprintln("----------------------------");
	aprintln("New gyro_offset_x offset is: %f", gyro_offset_x);
}


static bool _setFloatFromArg(float *var, const char *arg)
{
	char *endptr;
	float result = strtof(arg, &endptr);
	if (result == 0 && *endptr != '\0') {
		aprintln("Conversion of '%s' to float failed", arg);
		return false;
	}

	*var = result;
	return true;
}


static bool _setIntFromArg(int *var, const char *arg)
{
	char *endptr;
	int result = strtol(arg, &endptr, 10);
	if (result == 0 && *endptr != '\0') {
		aprintln("Conversion of '%s' to int failed", arg);
		return false;
	}

	*var = result;
	return true;
}

static bool _setBoolFromArg(bool *var, const char *arg)
{
	char *endptr;
	int result = strtol(arg, &endptr, 10);

	if (result == 0 && *endptr != '\0') {
		aprintln("Conversion of '%s' to bool failed", arg);
		return false;
	}
	if (result != 0 && result != 1) {
		aprintln("Conversion of '%s' to bool failed (only 0 or 1 is allowed)", arg);
		return false;
	}

	*var = result;
	return true;
}

static bool _setOrToggleBoolFromArg(bool *var, const char *arg)
{
	if (*arg == '\0') {
		*var = !(*var);
		return true;
	}

	return _setBoolFromArg(var, arg);
}

static void cmdPrintConfig(const char *arg)
{
	static const int PARAMETER_COLUMN_SIZE = 40;
	static const int VALUE_COLUMN_SIZE = 16;
	static const int DEFAULT_VALUE_COLUMN_SIZE = 16;

#define HR "|-----------------------------------------------------------------------------------|"
#define INT_PARAM_FMT 	"| %c%*s | %c%*d | %*d |"
#define FLOAT_PARAM_FMT "| %c%*s | %c%*f | %*f |"
#define BOOL_PARAM 		"| %c%*s | %c%*s | %*s |"
#define EXPAND_PARAM(cmd_prefix, name, value, def_value) \
	cmd_prefix, PARAMETER_COLUMN_SIZE, name, \
	((value) == (def_value)) ? ' ' : '*', VALUE_COLUMN_SIZE, value, \
	DEFAULT_VALUE_COLUMN_SIZE, def_value


	aprintln(
			HR SERIAL_ENDL
			"| %*s |  %*s | %*s |" SERIAL_ENDL
			HR,
			PARAMETER_COLUMN_SIZE, "PARAMETER",
			VALUE_COLUMN_SIZE, "VALUE",
			DEFAULT_VALUE_COLUMN_SIZE, "DEFAULT_VALUE"
	);

	aprintln(INT_PARAM_FMT,
			EXPAND_PARAM(CMD_DEFAULT_SAMPLE_RATE_PREFIX, "sample_rate",
					sample_rate, DEFAULT_SAMPLE_RATE));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(CMD_TAU_PREFIX, "tau",
					tau, DEFAULT_COMPLEMENTARY_FILTER_TAU));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(CMD_KP_PREFIX, "kp",
					kp, DEFAULT_PID_KP));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(CMD_KI_PREFIX, "ki",
					ki, DEFAULT_PID_KI));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(CMD_KD_PREFIX, "kd",
					kd, DEFAULT_PID_KD));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(CMD_DEFAULT_SETPOINT_DEG_PREFIX, "setpoint_deg",
					setpoint_deg, DEFAULT_SETPOINT_DEG));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(CMD_GYRO_X_OFFSET_PREFIX, "gyro_offset_x",
					gyro_offset_x, GYRO_OFFSET_X));
	aprintln(FLOAT_PARAM_FMT,
			EXPAND_PARAM(' ', "giveup_deg",
					GIVEUP_DEG, GIVEUP_DEG));
	aprintln(INT_PARAM_FMT,
			EXPAND_PARAM(' ', "microsteps_per_step",
					MOTOR_MICROSTEPS_PER_STEP, MOTOR_MICROSTEPS_PER_STEP));
	aprintln(BOOL_PARAM,
			EXPAND_PARAM(CMD_DEFAULT_SIMULATION_MODE_PREFIX, "simulation_mode",
					BOOL_TO_STR(simulation_mode), BOOL_TO_STR(DEFAULT_SIMULATION_MODE)));
	aprintln(BOOL_PARAM,
			EXPAND_PARAM(CMD_DEFAULT_DISPLAY_STEP_RESPONSE_PREFIX, "display_step_response",
					BOOL_TO_STR(display_step_response), BOOL_TO_STR(DEFAULT_DISPLAY_STEP_RESPONSE)));
	aprintln(BOOL_PARAM,
			EXPAND_PARAM(CMD_DEFAULT_DISPLAY_MOTOR_RESPONSIVENESS_PREFIX, "display_motor_responsiveness",
					BOOL_TO_STR(display_motor_responsiveness), BOOL_TO_STR(DEFAULT_DISPLAY_MOTOR_RESPONSIVENESS)));
	aprintln(BOOL_PARAM,
			EXPAND_PARAM(CMD_DEFAULT_ECHO_COMMANDS_PREFIX, "echo_commands",
					BOOL_TO_STR(echo_commands), BOOL_TO_STR(DEFAULT_ECHO_COMMANDS)));
#if VERBOSE
	aprintln(BOOL_PARAM,
			EXPAND_PARAM(CMD_VERBOSE_PREFIX, "verbose",
					BOOL_TO_STR(verbose), BOOL_TO_STR(DEFAULT_VERBOSE)));
#endif
	aprintln(HR);

#undef HR
#undef INT_PARAM_FMT
#undef FLOAT_PARAM_FMT
#undef BOOL_PARAM
#undef EXPAND_PARAM
}

static void cmdHelp(const char *arg)
{
	static int longest_arg_help = -1;
	if (longest_arg_help < 0) {
		// Compute the longest help length, just the first time
		for (int i = 0; i < ARRAY_SIZE(COMMANDS); i++)
			longest_arg_help = MAX(longest_arg_help, (int) strlen(COMMANDS[i].arg_help));
	}

	for (int i = 0; i < ARRAY_SIZE(COMMANDS); i++) {
		const SerialCommand *cmd = &COMMANDS[i];
		aprintln("%c%s%*s : %s",
				cmd->prefix != '\0' ? cmd->prefix : ' ',
				cmd->arg_help,
				longest_arg_help - strlen(cmd->arg_help), "",
				cmd->help);
	}
}

static void cmdSetSampleRate(const char *arg) {
	int val;
	if (!_setIntFromArg(&val, arg))
		return;
	LSM6DSL_Frequency freq = LSM6DSL_Frequency_FromInt(val);
	if (freq == LSM6DSL_POWER_DOWN) {
		aprintln("Unexpected argument '%d'" SERIAL_ENDL
				"(only 12, 26, 52, 104, 208, 416, 833, 1666, 3333, 6666 are allowed) ", val);
		return;
	}
	lsm6dsl_freq = freq;
	sample_rate = LSM6DSL_Frequency_ToInt(lsm6dsl_freq);
}

static void cmdSetTau(const char *arg)
{
	_setFloatFromArg(&tau, arg);
}

static void cmdSetKp(const char *arg)
{
	_setFloatFromArg(&kp, arg);
}

static void cmdSetKd(const char *arg)
{
	_setFloatFromArg(&kd, arg);
}

static void cmdSetKi(const char *arg)
{
	_setFloatFromArg(&ki, arg);
}

static void cmdSetSetpoint(const char *arg)
{
	_setFloatFromArg(&setpoint_deg, arg);
}

static void cmdSetOrToggleSimulationMode(const char *arg)
{
	_setOrToggleBoolFromArg(&simulation_mode, arg);
}

static void cmdSetOrToggleDisplayStepResponse(const char *arg)
{
	_setOrToggleBoolFromArg((bool*) &display_step_response, arg);
}

static void cmdSetOrToggleDisplayMotorResponsiveness(const char *arg)
{
	_setOrToggleBoolFromArg((bool*) &display_motor_responsiveness, arg);
}

static void cmdSetOrToggleEchoCommands(const char *arg)
{
	_setOrToggleBoolFromArg((bool*) &echo_commands, arg);
}

#if VERBOSE
static void cmdSetOrToggleVerbose(const char *arg)
{
	_setOrToggleBoolFromArg(&verbose, arg);
}
#endif


static void cmdStepMotorA(const char *arg)
{
	bool forward = !arg || arg[0] == '\0' || arg[0] == 'f';

	initializeMotors();
	startMotors();
	if (forward)
		stepForwardMotorA(true);
	else
		stepBackwardMotorA(true);
}


static void cmdStepMotorB(const char *arg)
{
	bool forward = !arg || arg[0] == '\0' || arg[0] == 'f';

	initializeMotors();
	startMotors();
	if (forward)
		stepForwardMotorB(true);
	else
		stepBackwardMotorB(true);
}

static void cmdSpinMotors(const char *arg)
{
	float val;
	if (!_setFloatFromArg(&val, arg))
		return;

	initializeMotors();
	startMotors();
	running = true;
	target_rpm = val;
	osSignalSet(motorControllerTaskHandle, MOTOR_CONTROLLER_TASK_SIGNAL_OUTPUT_READY);
}

static void cmdSetGyroXOffset(const char *arg)
{
	float val;
	if (!_setFloatFromArg(&val, arg))
		return;
	gyro_offset_x = val;
	averboseln("new gyro_offset_x offset value set to %f", gyro_offset_x);
}

static void cmdGyroAutotuning(const char *arg)
{
	int val;
	if (!_setIntFromArg(&val, arg))
		return;
	gyroAutotuning(val);
}

static void cmdStart(const char *arg)
{
	averboseln("Command START");
	osSignalSet(startStopTaskHandle, START_STOP_TASK_SIGNAL_START);
}

static void cmdStop(const char *arg)
{
	averboseln("Command STOP");
	osSignalSet(startStopTaskHandle, START_STOP_TASK_SIGNAL_STOP);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_InputProcessorTask */
/* Input Processor Task.
 * Wait for new data from sensors (gyro+accel) and process those using
 * a complementary filter and a pid to compute the desired output of the robot
 * ('target_rpm').
 */
/* USER CODE END Header_InputProcessorTask */
void InputProcessorTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	TASK_PROLOGUE(InputProcessorTask, /* verbose */ true);

	uint32_t samples = 0;
	uint8_t status;
	dim3_f xl, g;
	bool xl_ready, g_ready;

	while (true) {
		osEvent ev;
		do {
			Task_SignalWaitGet(
					ev,
					INPUT_PROCESSOR_TASK_SIGNAL_INPUT_READY | INPUT_PROCESSOR_TASK_SIGNAL_RESET,
					osWaitForever);
		} while(!(ev.value.signals & INPUT_PROCESSOR_TASK_SIGNAL_INPUT_READY ||
				ev.value.signals & INPUT_PROCESSOR_TASK_SIGNAL_RESET));

		if (ev.value.signals & INPUT_PROCESSOR_TASK_SIGNAL_RESET) {
			samples = 0;
			continue;
		}

		osMutexWait(sensorsMutexHandle, osWaitForever);

		// Read the status
		if (!LSM6DSL_ReadStatus(&status) == HAL_OK) {
			Task_averboseln("Failed to read LSM6DSL status");
			osMutexRelease(sensorsMutexHandle);
			continue;
		}

		// Read the sensors
		xl_ready = (status & LSM6DSL_REG_STATUS_BIT_XLDA) &&
				LSM6DSL_ReadAccelerometer_g(&xl) == HAL_OK;
		g_ready = (status & LSM6DSL_REG_STATUS_BIT_GDA) &&
				LSM6DSL_ReadGyroscope_dps(&g) == HAL_OK;

		osMutexRelease(sensorsMutexHandle);

		Task_averboseln("LSM6DSL status = 0x%02X", status);

		if (!(xl_ready && g_ready)) {
			Task_averboseln("Skipping uncompleted sensors data");
			continue;
		}

		// The first samples usually have some noise; discard them
		if (samples < LSM6DSL_INITIAL_SAMPLES_TO_DISCARD) {
			Task_averboseln("Discarding sample n. %u", samples);
			samples++;
			continue;
		}

		// Handle the sensors measurement
		Task_averboseln("Accelerometer: (x=%f, y=%f, z=%f)g", xl.x, xl.y, xl.z);
		Task_averboseln("Gyroscope:     (x=%f, y=%f, z=%f)dps", g.x, g.y, g.z);

		if (gyro_autotuning) {
			gyro_autotuning_mean.sum += g.x;
			gyro_autotuning_mean.count++;
			Task_SignalSet(uiTaskHandle, UI_TASK_SIGNAL_AUTOTUNING);
			continue;
		}

		float gx_raw = g.x;

		// Apply adjustments to sensors measurements
		xl.x -= ACCEL_OFFSET_X; xl.y -= ACCEL_OFFSET_Y; xl.z -= ACCEL_OFFSET_Z;
		g.x -= gyro_offset_x;

		// Estimate the angle using measures from both sensors with a filter
		ComplementaryFilter_Compute(&filter, xl, g);
		float input = filter.roll;

		Task_averboseln("Roll = %.2f deg", input);

		// Do not even try to balance if the robot is too much sloped
		if (ABS(input) > GIVEUP_DEG) {
			target_rpm = 0;
		} else {
			// Compute the motor's output using a PID
			PID_Compute(&pid, input);

			target_rpm = pid.output;

			Task_averboseln("RPM = %.2f", target_rpm);

			if (display_step_response) {
				taprintln("%f deg (xl_x_raw=%f, gyro_x_raw=%f, gyro_x=%f) | %f rpm | %f Hz",
							input, xl.x, gx_raw, g.x, target_rpm, target_rpm * RPM_TO_FREQ);
			}
		}
		// Wake up the motor task since the target rpm is changed
		Task_SignalSet(motorControllerTaskHandle, MOTOR_CONTROLLER_TASK_SIGNAL_OUTPUT_READY);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MotorControllerTask */
/* Motor Controller Task.
 * Is responsible for submit step signals to the motors drivers at regular
 * interval, depending on the current 'target_rpm'.
 * Furthermore is able to detect changes to 'target_rpm' before the step period
 * elapses thanks to the MOTOR_CONTROLLER_TASK_SIGNAL_OUTPUT_READY signal.
 *
 */
/* USER CODE END Header_MotorControllerTask */
void MotorControllerTask(void const * argument)
{
  /* USER CODE BEGIN MotorControllerTask */
	TASK_PROLOGUE(MotorControllerTask, /* verbose */ true);

	float target_rpm_snapshot;
	uint32_t target_step_period_us;
	uint32_t last_step_us;

#define MICROSECONDS_TO_STEP() ((int64_t) last_step_us + target_step_period_us - GetMicroseconds())

	while (true) {
		bool idling = !running;
		uint32_t wait_ms  = idling ?
			osWaitForever : /* wait forever if in idle mode (at the beginning or after a reset) */
			MAX(0, ((int64_t) last_step_us + target_step_period_us - GetMicroseconds())) / 1000
		;

		Task_averboseln("Waiting for %u ms...", wait_ms);
		osEvent ev;
		Task_SignalWaitGet(
				ev,
				(MOTOR_CONTROLLER_TASK_SIGNAL_OUTPUT_READY | MOTOR_CONTROLLER_TASK_SIGNAL_RESET),
				wait_ms
		);

		if (ev.status == osEventSignal &&
			ev.value.signals & MOTOR_CONTROLLER_TASK_SIGNAL_RESET) {
			// Received RESET signal, go into idle mode
			Task_averboseln("WakeUp reason: reset; going into idle mode");
			continue;
		}

		bool perform = false;
		if (ev.status == osEventTimeout || ev.status == osOK) {
			// Step period elapsed, always perform the step in this case
			Task_averboseln(
					"WakeUp reason: period timeout (last wait = %u ms | total period = %u ms)",
					wait_ms, target_step_period_us);
			perform = true;
		} else if (	ev.status == osEventSignal &&
					ev.value.signals & MOTOR_CONTROLLER_TASK_SIGNAL_OUTPUT_READY) {
			// Target RPM changed, we have to perform a step only if we are
			// late relative to the new step period
			Task_averboseln(
					"WakeUp reason: target_rpm changed to %f -> new period = %u ms",
					target_rpm_snapshot, target_step_period_us / 1000);
			target_rpm_snapshot = target_rpm;
			target_step_period_us = lroundf(1000000.0f / ((ABS(target_rpm_snapshot) * RPM_TO_FREQ)));
			perform = MICROSECONDS_TO_STEP() <= 0;
		}

		if (idling) {
			// Don't perform since in idle mode,
			// just keep track of the time the new data has been handled
			Task_averboseln("Not performing since in idle mode");
			last_step_us = GetMicroseconds();
			continue;
		}

		if (perform) {
			int64_t remaining_us_before, remaining_us, remaining_us_after;
			remaining_us_before = remaining_us = MICROSECONDS_TO_STEP();
			while (remaining_us > 50) {
				Task_averboseln("Micro sleep wait of %u us before step", remaining_us);
				if (remaining_us > 1000)
					osDelay(remaining_us / 1000);
				else
					SleepMicroseconds(remaining_us);

				remaining_us = MICROSECONDS_TO_STEP();
			}

			if (target_rpm_snapshot > 0) {
				stepForwardMotorA(false);
				stepForwardMotorB(false);
			} else {
				stepBackwardMotorA(false);
				stepBackwardMotorB(false);
			}

			SleepMicroseconds(A4988_MINIMUM_STEP_PULSE_MICROSECONDS);
			stepFallingEdgeMotorA();
			stepFallingEdgeMotorB();

			remaining_us_after = MICROSECONDS_TO_STEP();
			last_step_us = GetMicroseconds();

			if (display_motor_responsiveness) {
				if (ev.status == osEventTimeout || ev.status == osOK) {
					taprintln("STEP | reason = period timeout (%u us) | "
							"remaining us = (%lld...%lld...%lld) | %s",
							target_step_period_us,
							remaining_us_before, remaining_us, remaining_us_after,
							remaining_us_before < 0 ? "[late]" : "[ok: in-time]"
					);
				} else {
					taprintln("STEP | reason = rpm changed to %f (new period is %u us) | "
							 " remaining us = (%lld...%lld...%lld) | [ok: justified]",
							 target_rpm_snapshot,
							 target_step_period_us,
							 remaining_us_before, remaining_us, remaining_us_after);
				}
			}
		} else {
			if (display_motor_responsiveness)
				taprintln("NOP: reason = rpm changed (new period %u us) | remaining us = %lld",
						target_step_period_us, MICROSECONDS_TO_STEP());
		}
	}

#undef MICROSECONDS_TO_STEP
  /* USER CODE END MotorControllerTask */
}

/* USER CODE BEGIN Header_UITask */
/*
 * UI Task.
 * Blinks internal LED1 and LED2 depending on what's happening.
 * Actually the leds might blink for two reason:
 * - the robot is going forward or backward
 * - autotuning is going on
 */
/* USER CODE END Header_UITask */
void UITask(void const * argument)
{
  /* USER CODE BEGIN UITask */
	TASK_PROLOGUE(MotorControllerTask, /* verbose */ true);

	GPIO_Pin led_forward = led1;
	GPIO_Pin led_backward = led2;

	GPIO_Pin_Low(led_forward);
	GPIO_Pin_Low(led_backward);

	uint8_t steps_forward = 0;
	uint8_t micro_forward = 0;

	uint8_t steps_backward = 0;
	uint8_t micro_backward = 0;

	uint8_t autotuning_reads = 0;

	while (true) {
		osEvent ev;
		Task_SignalWaitGet(
			ev,
			(UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_A | UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_A |
			UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_B | UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_B |
			UI_TASK_SIGNAL_AUTOTUNING),
			osWaitForever
		);
		if (ev.value.signals & UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_A ||
				ev.value.signals & UI_TASK_SIGNAL_STEP_FORWARD_MOTOR_B) {
			micro_backward = 0;
			steps_backward = 0;
			micro_forward = (micro_forward + 1) % MOTOR_MICROSTEPS_PER_STEP;
			steps_forward =
					(steps_forward + (micro_forward == 0))
					% UI_BLINK_AFTER_MOTOR_STEPS;
			GPIO_Pin_Low(led_backward);
			GPIO_Pin_Write(led_forward,
				steps_forward == UI_BLINK_AFTER_MOTOR_STEPS - 1 &&
				micro_forward == MOTOR_MICROSTEPS_PER_STEP - 1);
		}
		if (ev.value.signals & UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_A ||
				ev.value.signals & UI_TASK_SIGNAL_STEP_BACKWARD_MOTOR_B) {
			micro_forward = 0;
			steps_forward = 0;
			micro_backward = (micro_backward + 1) % MOTOR_MICROSTEPS_PER_STEP;
			steps_backward =
					(steps_backward + (micro_backward == 0))
					% UI_BLINK_AFTER_MOTOR_STEPS;
			GPIO_Pin_Low(led_forward);
			GPIO_Pin_Write(led_backward,
				steps_backward == UI_BLINK_AFTER_MOTOR_STEPS - 1 &&
				micro_backward == MOTOR_MICROSTEPS_PER_STEP - 1);
		}
		if (ev.value.signals & UI_TASK_SIGNAL_AUTOTUNING) {
			autotuning_reads = (autotuning_reads + 1) % UI_BLINK_AFTER_AUTOTUNING_SENSOR_READ;
			if (autotuning_reads == 0) {
				GPIO_Pin_Toggle(led1);
				GPIO_Pin_Toggle(led2);
			}
		}
	}

  /* USER CODE END UITask */
}

/* USER CODE BEGIN Header_SerialTxFlushTask */
/*
 * Serial Transmission Flusher Task.
 * Is responsible for flush the async message written to the serial
 * with the aprintXXX functions.
 */
/* USER CODE END Header_SerialTxFlushTask */
void SerialTxFlushTask(void const * argument)
{
  /* USER CODE BEGIN SerialTxFlushTask */
	TASK_PROLOGUE(SerialTxFlushTask, /* verbose */ true);

	while (true) {
		// Blocks until a new data is available, then prints it
		Serial_Flush_Tx();
	}
  /* USER CODE END SerialTxFlushTask */
}

/* USER CODE BEGIN Header_SerialRxTask */
/*
 * Serial Receiver Task.
 * Reads from the serial and process the command
 * when the message is submitted ('\r').
 */
/* USER CODE END Header_SerialRxTask */
void SerialRxTask(void const * argument)
{
  /* USER CODE BEGIN SerialRxTask */
	TASK_PROLOGUE(SerialRxTask, /* verbose */ true);

	char input[COMMAND_MAX_LEN];
	while (true) {
		Serial_ReadStringCR_Async(input, COMMAND_MAX_LEN);
		if (!Serial_Wait_Rx()) {
			aprintln(SERIAL_ENDL "Failed to read command");
			continue;
		}

		if (echo_commands)
			aprintln(">> %s", input);

		char cmd = input[0];
		bool command_found = false;
		for (int i = 0; i < ARRAY_SIZE(COMMANDS) && !command_found; i++) {
			if (COMMANDS[i].prefix == cmd) {
				command_found = true;
				COMMANDS[i].fn(&input[1]);
			}
		}

		if (!command_found) {
			aprintln("Unrecognized command: '%s'", input);
		}
	}

  /* USER CODE END SerialRxTask */
}

/* USER CODE BEGIN Header_StartStopTask */
/*
 * Start/Stop Task.
 * Wait for either START or STOP events and perform the actual
 * start/stop job, eventually notifying the tasks about the new state.
 */
/* USER CODE END Header_StartStopTask */
void StartStopTask(void const * argument)
{
  /* USER CODE BEGIN StartStopTask */
	TASK_PROLOGUE(StartStopTask, /* verbose */ true);

	while (true) {
		osEvent ev;
		do {
			Task_SignalWaitGet(
					ev,
					START_STOP_TASK_SIGNAL_START |
					START_STOP_TASK_SIGNAL_STOP |
					START_STOP_TASK_SIGNAL_TOGGLE,
					osWaitForever);
		} while(!(ev.value.signals & START_STOP_TASK_SIGNAL_START ||
				  ev.value.signals & START_STOP_TASK_SIGNAL_STOP ||
				  ev.value.signals & START_STOP_TASK_SIGNAL_TOGGLE));

		if (ev.value.signals & START_STOP_TASK_SIGNAL_START ||
			((ev.value.signals & START_STOP_TASK_SIGNAL_TOGGLE) && !running)) {
			running = true;
			initializeRobot();
			startRobot();
		} else if (ev.value.signals & START_STOP_TASK_SIGNAL_STOP ||
				((ev.value.signals & START_STOP_TASK_SIGNAL_TOGGLE) && running)) {
			running = false;
			initializeRobot();
			stopRobot();
			Task_SignalSet(inputProcessorTaskHandle, INPUT_PROCESSOR_TASK_SIGNAL_RESET);
			Task_SignalSet(motorControllerTaskHandle, MOTOR_CONTROLLER_TASK_SIGNAL_RESET);
		}
	}
  /* USER CODE END StartStopTask */
}

/* USER CODE BEGIN Header_AutotuningTask */
/**
* @brief Function implementing the autotuningTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AutotuningTask */
void AutotuningTask(void const * argument)
{
  /* USER CODE BEGIN AutotuningTask */
	TASK_PROLOGUE(AutotuningTask, /* verbose */ true);

	while(true) {
		osEvent ev;
		do {
			Task_SignalWaitGet(
					ev,
					AUTOTUNING_TASK_SIGNAL,
					osWaitForever);
		} while(!(ev.value.signals & AUTOTUNING_TASK_SIGNAL));

		if (ev.status == osEventSignal &&
			ev.value.signals & AUTOTUNING_TASK_SIGNAL) {
			gyroAutotuning(30);
		}
	}
  /* USER CODE END AutotuningTask */
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
	// Blinks LED1/LED2 forever.
	GPIO_Pin_High(led1);
	GPIO_Pin_High(led2);
	while (true) {
		GPIO_Pin_Toggle(led1);
		GPIO_Pin_Toggle(led2);
		SleepMicroseconds(5000); // blocking, retain execution control
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
