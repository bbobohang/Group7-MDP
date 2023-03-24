/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ICM20948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayOLED */
osThreadId_t displayOLEDHandle;
const osThreadAttr_t displayOLED_attributes = {
  .name = "displayOLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for moveDistTask */
osThreadId_t moveDistTaskHandle;
const osThreadAttr_t moveDistTask_attributes = {
  .name = "moveDistTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servoMotor */
osThreadId_t servoMotorHandle;
const osThreadAttr_t servoMotor_attributes = {
  .name = "servoMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for batteryReader */
osThreadId_t batteryReaderHandle;
const osThreadAttr_t batteryReader_attributes = {
  .name = "batteryReader",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
	//Pos of car
	typedef struct _Position {
		long double x; // x-coordinate
		long double y; // y-coordinate
		long double theta; // orientation
	} position;

	position pos = {0.0, 0.0, 0.0};

	//Estimated distance ticks
	typedef struct _WheelEncoderConfig {
	    int oldCount;
		int newCount;
	} WheelEncoderConfig;

	WheelEncoderConfig encoderL = {0}, encoderR = {0};
	static const WheelEncoderConfig emptyEncoder = {0};


	typedef struct _DistanceTickConfig {
		int targetTick;
		int currDistTick;
		uint32_t startTick;
	} DistanceTickConfig;

	DistanceTickConfig distTick = {0};
	static const DistanceTickConfig emptyDistTick= {0};

	//PID
	typedef struct _DutyConfig{
		int newCount;       // Timer 2 counter
		int oldCount;
		int newPwmVal;
		int pwmVal;       //pwm value to control motor speed
		int current_Rpm;         // speed in rpm number of count/sec * 60 sec  divide by 330 count per round
		int target_Rpm;
		int err; // status for checking return

		int error;           // error between target and actual
		int error_area;  // area under error - to calculate I for PI implementation
		int error_old, error_change;//for Kd
		float error_rate; // to calculate D for PID control
		int32_t millisOld, millisNow, dt; // to calculate I and D for PID control
	} DutyConfig;

	DutyConfig dutyL = {0}, dutyR = {0}; //Initialize all members to 0
	static const DutyConfig emptyDuty = {0}; //Empty struct to reset struct members

	typedef struct _ServoConfig{
			float servoValue;       //currentServo Value starting from 140-160
			float current_Angle;    // current reading from the gyro meter
			float target_Angle;	//Target angle we want to maintain

			float error;           // error between target and actual
			float error_area;  // area under error - to calculate I for PI implementation
			float error_old, error_change;//for Kd
			float error_rate; // to calculate D for PID control
			int32_t millisOld, millisNow, dt; // to calculate I and D for PID control
		} ServoConfig;

		ServoConfig servo; //Initialize all members to 0
		static const ServoConfig emptyServo = {0}; //Empty struct to reset struct members

	typedef struct _pidConfig {
		float Kp;
		float Ki;
		float Kd;
	} PIDConfig;

	PIDConfig dcPidCfg = {4, 0, 0.08}, servoPidCfg = {10, 0.01, 0.008};
		//{4, 0, 0.01}:300	(5,0,0.01):500			(30, 0.01, 0.008: 300 lowBatt) (18,0.01,0.01) (5, 0.01, 0.008) 600rpm {18, 0.01, 0.008} 800rpm

	int totalPPR = 0;
	int prevDir = 1;

//	typedef struct _ServoConfig{
//		int FR90, FRFW, FRBW;
//		int FL90, FLFW, FLBW;
//		int BR90, BRFW, BW;
//
//		int BL90;
//		int FRL;
//		int FLR;
//		int BRL;
//		int BLR;
//	} ServoConfig;
//
	//ICM9020948 Gyro Var
	uint8_t gyroData[2] = {0};
	float currAngle = 0.0;
	float targetAngle = 0.0;
	int16_t gyroZAxis = 0;

	//UARTS communication
	uint8_t aRxBuffer[20];

	typedef struct _command {
		uint8_t index;
		uint16_t val;
	} Command;

	Command currCmd = {100, 0};

	typedef struct _commandQueue {
		uint8_t head;
		uint8_t tail;
		uint8_t size;
		Command buffer[CMD_BUFFER_SIZE];
	} CommandQueue;

	CommandQueue commandQueue;

	enum TASK_TYPE{
		TASK_MOVE,
		TASK_MOVE_BACKWARD,
		TASK_FL,
		TASK_FR,
		TASK_BL,
		TASK_BR,
		TASK_BUZZER,
		TASK_NONE,
		TASK_FL2,
		TASK_FR2,
		TASK_FRL,
		TASK_FLR,
		TASK_P1R, TASK_P2R, TASK_P3R,
		TASK_P1L, TASK_P2L, TASK_P3L
	};
	enum TASK_TYPE currTask = TASK_NONE;

	int angle = 0;

	uint8_t manualMode = 0;
	//IR Sensors
	uint32_t adcVal = 0;
	uint16_t dataPoint = 0;
	float obsDist_IR = 0;

	//Ultrasonic Sensors
	uint32_t IC_Val1 = 0;
	uint32_t IC_Val2 = 0;
	uint32_t Difference = 0;
	uint8_t Is_First_Captured = 0;  // is the first value captured ?
	//uint8_t Distance_US[2]  = 0;
	int part = 0;
	//Battery
	float batteryVal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void displayOLED_Task(void *argument);
void Command_Task(void *argument);
void MoveDistance_Task(void *argument);
void servoMotor_Task(void *argument);
void BatterReader_Task(void *argument);

/* USER CODE BEGIN PFP */
//Methods declaration
void DC_PID_Control(PIDConfig *dcPidCfg, DutyConfig *duty, WheelEncoderConfig *wheelEncoder); //Calculate duty required to achieve target RPM
void SERVO_PID_Control(PIDConfig *servoPidCfg, ServoConfig *servo, int dir); //Calculate servo output to achieve straight line movement

void Set_Motor_Direction(uint8_t dir); //Set motor to go forward dir = (1) or backward dir =(0)
int Calculate_IntervalPulse(int oldCount, int newCount, int dutyLR); //Calcuate difference of ticks between the sampling frequency time of the wheel encoders
void Calculate_Target_Ticks(float targetDistance, DistanceTickConfig * distTick); //Convert distance in metres to ticks for distance estimation
void Calculate_Car_Distance(DistanceTickConfig * distTick, WheelEncoderConfig *encoderL, WheelEncoderConfig *encoderR); //Calculate the current tick travelled and store it in distTick.currDistTick
void Init_Duty_Config(int rpm, float distance); //Initialise all parameters required to start a new journey
void Reset_Duty_Config(); //Reset all parameters to 0 & call Init_Duty_Config to input the new args for next journey

void Read_Gyro_DataZ(int16_t *gyroZAxis); //Read the Z axis of the gyro (rad/s) and store it in gyroZAxis
void SERVO_Straight_Control(int dir); //This fn will control the car to ensure it is going straight through the use of gyro Z axis & PID
void Calculate_StraightLine(float *targetAngle, int RLDir); //Fn will measure whether the car has hit the target angle during turning and straighten the wheels
void Servo_Turn_Right_90(); //Fn will turn servo to max right & start moving. Calling Calculate_StraightLine to stop the turn once target is reached
void Servo_Turn_Left_90(); //Similarly as above
void Servo_Turn_Right_180(); //Similarly as above
void Servo_Turn_Left_180(); //Similarly as above
void Servo_Turn_Right_90_Back();
void Servo_Turn_Left_90_Back();
void Servo_Turn_Right_180_Back();
void Servo_Turn_Left_180_Back();
void Servo_Turn_Right_90_2(); //Fn will turn servo to max right & start moving. Calling Calculate_StraightLine to stop the turn once target is reached
void Servo_Turn_Left_90_2();
void Servo_Turn_LeftRight_Indoor();
void Servo_Turn_LeftRight_Outdoor();
void Servo_Turn_RightLeft_Outdoor();
void Servo_Turn_RightLeft_Indoor();
void Servo_Turn_RightLeft();
void Servo_Turn_LeftRight();
void Servo_Turn_RightLeft_Back();
void Servo_Turn_LeftRight_Back();

void DC_MoveForward(int rpm, int dir);
void DC_MoveDistForward(int rpm, float distance, int dir);

int CommandQueue_IsEmpty(CommandQueue *commandQueue);
void CommandQueue_Enqueue(CommandQueue *commandQueue, uint8_t index, uint16_t val);
void CommandQueue_Dequeue(CommandQueue *commandQueue, Command *currCmd); //Store the tail command into currCmd & dequeue the command

void CurrentCommand_Clear(Command *currCmd);
void CurrentCommand_Pending(Command *currCmd);

void Task_ACK();
void Task_END(); //Assign current task to TASK_NONE

void ADC_Read_Distance(ADC_HandleTypeDef* adc);
void US_Read_Distance(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void Update_Wheel_EncoderConfig(WheelEncoderConfig *encoder); //Not In Use
void Wheel_Odometry(position *pos, WheelEncoderConfig encoderL, WheelEncoderConfig encoderR); //Not In Use

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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  	//Initialise commandQueue
  	int i;
	commandQueue.head = 0;
	commandQueue.tail = 0;
	commandQueue.size = CMD_BUFFER_SIZE;

	for (i = 0; i < CMD_BUFFER_SIZE; i++) {
	  Command cmd;
	  cmd.index = 100;
	  cmd.val = 0;
	  commandQueue.buffer[i] = cmd;
	}

	//Initialise OLED
	OLED_Init();

	//Initialise Gyroscope
	ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_2000DPS);

	//Initiate UART
	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, ARX_BUFFER_SIZE);

	//Initiate the motors
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

	//Initiate ServoMotor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//Start MotorA Encoder
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	//Start MotorB Encoder
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

	//Init left wheel 1st tick before PID
	dutyL.millisOld = HAL_GetTick();
	//Init right wheel 1st tick before PID
	dutyR.millisOld = HAL_GetTick();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of displayOLED */
  displayOLEDHandle = osThreadNew(displayOLED_Task, NULL, &displayOLED_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(Command_Task, NULL, &commandTask_attributes);

  /* creation of moveDistTask */
  moveDistTaskHandle = osThreadNew(MoveDistance_Task, NULL, &moveDistTask_attributes);

  /* creation of servoMotor */
  servoMotorHandle = osThreadNew(servoMotor_Task, NULL, &servoMotor_attributes);

  /* creation of batteryReader */
  batteryReaderHandle = osThreadNew(BatterReader_Task, NULL, &batteryReader_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |TRIG_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           TRIG_Pin LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |TRIG_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/*Prevent unused argument(s) compliation warning */
	UNUSED(huart);
	int val = 0;

	val = ((aRxBuffer[2] - '0') * 100 + (aRxBuffer[3] - '0') * 10 + (aRxBuffer[4] - '0'));

	if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T') { // only STOP can preempt any greedy task
		Task_END();
		manualMode = 0;

		if (CommandQueue_IsEmpty(&commandQueue)) {
			CurrentCommand_Clear(&currCmd);
			Task_ACK();
		}
		else {
			//CommandQueue_Dequeue(&commandQueue, &currCmd);
		}

	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'W') { //FW
		if(aRxBuffer[2] == '-' && aRxBuffer[3] == '-'){
			CommandQueue_Enqueue(&commandQueue, 11, 0);
		}else{
			//val *= 10;
			CommandQueue_Enqueue(&commandQueue, 1, val);
		}
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'W') { //BW
		if(aRxBuffer[2] == '-' && aRxBuffer[3] == '-'){
			CommandQueue_Enqueue(&commandQueue, 12, 0);
		}else{
			//val *= 10;
			CommandQueue_Enqueue(&commandQueue, 2, val);
		}
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L' && aRxBuffer[2] == '9') { //Forward Left 90
		CommandQueue_Enqueue(&commandQueue, 3, 0);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R' && aRxBuffer[2] == '9') { //Forward Right 90
		CommandQueue_Enqueue(&commandQueue, 4, 0);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L' && aRxBuffer[2] == '9') { //Backward Left 90
		CommandQueue_Enqueue(&commandQueue, 5, 0);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R' && aRxBuffer[2] == '9') { //Backward Right 90
		CommandQueue_Enqueue(&commandQueue, 6, 0);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L' && aRxBuffer[2] == '1') { //Forward Left 180
		CommandQueue_Enqueue(&commandQueue, 7, 0);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R' && aRxBuffer[2] == '1') { //Forward Right 180
		CommandQueue_Enqueue(&commandQueue, 8, 0);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L' && aRxBuffer[2] == '1') { //Backward Left 180
		CommandQueue_Enqueue(&commandQueue, 9, 0);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R' && aRxBuffer[2] == '1') { //Backward Right 180
		CommandQueue_Enqueue(&commandQueue, 10, 0);
	}
	else if(aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R' && aRxBuffer[2] == 'L') {
		CommandQueue_Enqueue(&commandQueue, 11, 0);
	}
	else if(aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L' && aRxBuffer[2] == 'R') {
		CommandQueue_Enqueue(&commandQueue, 12, 0);
	}
	else if(aRxBuffer[0] == 'O'){ //Ultrasonic Sensor Distance

		if(aRxBuffer[1] == 'L'){

			CommandQueue_Enqueue(&commandQueue, 13, val);

		}

		else if(aRxBuffer[1] == 'R'){

			CommandQueue_Enqueue(&commandQueue, 14, val);

		}
	}
//	else if(aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L' && aRxBuffer[2] == '-'){
//		CommandQueue_Enqueue(&commandQueue, 3, 0);
//	}
//	else if(aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R' && aRxBuffer[2] == '-'){
//		CommandQueue_Enqueue(&commandQueue, 4, 0);
//	}
	//Every callback, dequeue the latest command if the task is empty
	//if (!CommandQueue_IsEmpty(&commandQueue) && currTask == TASK_NONE) {
	if (!CommandQueue_IsEmpty(&commandQueue) || currTask == TASK_NONE) {
		CommandQueue_Dequeue(&commandQueue, &currCmd);
	}

	// clear ARX buffer
	__HAL_UART_FLUSH_DRREGISTER(&huart3);
	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, ARX_BUFFER_SIZE);

}

void Set_Motor_Direction(uint8_t dir) {
	if (dir){// move forward
		// GPIO_PIN_SET = Turn on, GPIO_PIN_RESET = Turn Off
		//To enable the motor circuit, one pin must be enabled and the other must be disabled.
		//MotorA(Left wheel)
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		//MotorB(Right wheel)
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
	  }
	else { // reverse
		//Swap the on/off between the 2 pins to switch the rotation of the motor
		//MotorA(Left wheel)
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		//MotorB(Right wheel)
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	}
}

int Calculate_IntervalPulse(int oldCount, int newCount, int dutyLR){
	int diff = 0, timerLR;

	if(dutyLR) //Determine if it is left duty or right duty
		timerLR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2); //left wheel
	else
		timerLR = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3); //right wheel

	if(timerLR){ //To consider overflow and underflow problem
		if(newCount < oldCount)
			diff = oldCount - newCount;
		else
			diff = (65535 - newCount) + oldCount; //Prevent overflow/underflow
	}else{
		if(newCount > oldCount)
			diff = newCount - oldCount;
		else
			diff = (65535 - oldCount) + newCount; //Prevent overflow/underflow
		}

	return diff;
}

void Calculate_Target_Ticks(float targetDistance, DistanceTickConfig * distTick){

	distTick->targetTick = targetDistance/(WHEEL_CIRCUMFERENCE/TICKS_PER_REVOLUTION);
}

void Calculate_Car_Distance(DistanceTickConfig * distTick, WheelEncoderConfig *encoderL, WheelEncoderConfig *encoderR){
	int deltaTicks = 0, rightTick = 0, leftTick = 0;

	encoderL->newCount = __HAL_TIM_GET_COUNTER(&htim2);
	encoderR->newCount = __HAL_TIM_GET_COUNTER(&htim3);

	if(encoderL->newCount != encoderL->oldCount)
		leftTick = Calculate_IntervalPulse(encoderL->oldCount, encoderL->newCount, 1);
	if(encoderR->newCount != encoderR->oldCount)
		rightTick = Calculate_IntervalPulse(encoderR->oldCount, encoderR->newCount, 0);


	if(leftTick < 65000 && rightTick < 65000){
		deltaTicks = (leftTick + rightTick)/2;
		distTick->currDistTick += deltaTicks;
	}

	//Bring the previous count to the next iteration
	encoderL->oldCount = encoderL->newCount;
	encoderR->oldCount = encoderR->newCount;
}

void Init_Duty_Config(int rpm, float distance){
	//Set the desired wheel's RPM target, roughly 2200+-PWM = 500RPM(330PPR) 500pwm = 20rpm(1500PPR)
	dutyL.target_Rpm = rpm;
	dutyR.target_Rpm = rpm;

	//Set desire target distance, in metres
	Calculate_Target_Ticks(distance, &distTick); //Result of target ticks will be stored in distTick.targetTick

	//Get left wheel 1st Tick count (MotorA Encoder) for PID
	dutyL.oldCount = __HAL_TIM_GET_COUNTER(&htim2);
	//Get right wheel 1st Tick count (MotorB Encoder) for PID
	dutyR.oldCount = __HAL_TIM_GET_COUNTER(&htim3);

	//Get left wheel 1st Tick count (MotorA Encoder) for DistEst
	encoderL.oldCount = __HAL_TIM_GET_COUNTER(&htim2);
	//Get right wheel 1st Tick count (MotorB Encoder) for DistEst
	encoderR.oldCount = __HAL_TIM_GET_COUNTER(&htim3);
}

void Reset_Duty_Config(){
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update PWM to 0 to halt
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update PWM to 0 to halt

	dutyL = emptyDuty; //Reset struct DutyConfig var to 0
	dutyR = emptyDuty; //Reset struct DutyConfig var to 0

	encoderL = emptyEncoder; //Reset struct WheelEncoderConfig var to 0
	encoderR = emptyEncoder; //Reset struct WheelEncoderConfig var to 0

	distTick = emptyDistTick; //Reset struct WheelEncoderConfig var to 0

//	servo = emptyServo; //Reset struct ServoConfig var to 0
//	currAngle = 0.0; //Reset currAngle to 0 to start another journey
//	gyroZAxis = 0;
}

void DC_PID_Control(PIDConfig *dcPidCfg, DutyConfig *duty, WheelEncoderConfig *wheelEncoder){
	//Control Loop
	//char displayStr[20];
	int ppr=0, dutyLR = 0; //dutyLR = 1(Left), =0(right)
	if(duty->target_Rpm == 0)
		return;

	if(duty == &dutyL){
		dutyLR = 1;
		duty->newCount = __HAL_TIM_GET_COUNTER(&htim2);
	}
	else if(duty == &dutyR)
		duty->newCount = __HAL_TIM_GET_COUNTER(&htim3);

	if(duty->newCount != duty->oldCount)
		ppr = Calculate_IntervalPulse(duty->oldCount, duty->newCount, dutyLR);

//	if(dutyLR && ppr < 65000)
//		wheelEncoder->ppr = ppr; //Store the ppr of this interval for Odometry fn Leftwheel
//	else if(!dutyLR && ppr < 65000) //65535
//		wheelEncoder->ppr = ppr; //Store the ppr of this interval for Odometry fn Rightwheel

	if(ppr < 65000)
		duty->current_Rpm = ((ppr * 60)/ (PULSE_PER_REVOLUTION));  // ppr = ticks per encoder sample ,PULSE_PER_REV * frequency (0.5s == 500 ticks)

	//When car is stationary, accurate RPM will not be defined
	if(duty->current_Rpm > 11000) //11915
		duty->current_Rpm = 0;

	duty->error = duty->target_Rpm - duty->current_Rpm;

	duty->millisNow = HAL_GetTick();
	duty->dt = (duty->millisNow - duty->millisOld); // time elapsed in millisecond (delta time)
	duty->millisOld = duty->millisNow; // store the current time for next round

	duty->error_area = duty->error_area + duty->error*duty->dt; // area under error for Ki

	duty->error_change = duty->error - duty->error_old; // change in error
	duty->error_old = duty->error; //store the error for next round
	duty->error_rate = duty->error_change/duty->dt; // for Kd

	duty->newPwmVal = (int)(duty->error * dcPidCfg->Kp + duty->error_area * dcPidCfg->Ki + duty->error_rate * dcPidCfg->Kd);  // PID

	duty->pwmVal += duty->newPwmVal;

//	sprintf(displayStr, "pwmVO:%6d\0", duty->pwmVal);
//	OLED_ShowString(10, 50, displayStr);

	if (duty->pwmVal > PWM_MAX) // Clamp the PWM to its maximum value
		duty->pwmVal = PWM_MAX;
	else if(duty->pwmVal < PWM_MIN){ //If PWMValue goes to negative value, restart the PWMvalue back to 500
		duty->pwmVal = 500;
	}

	duty->oldCount = duty->newCount; //Bring over the newCount to the next iteration
}

void SERVO_PID_Control(PIDConfig *servoPidCfg, ServoConfig *servo, int dir){
	//Control Loop

	if(dir)
		servo->error =  servo->current_Angle - servo->target_Angle;
	else
		servo->error =  servo->target_Angle - servo->current_Angle;

	servo->millisNow = HAL_GetTick();
	servo->dt = (servo->millisNow - servo->millisOld); // time elapsed in millisecond (delta time)
	servo->millisOld = servo->millisNow; // store the current time for next round

	servo->error_area = servo->error_area + servo->error*servo->dt; // area under error for Ki

	servo->error_change = servo->error - servo->error_old; // change in error
	servo->error_old = servo->error; //store the error for next round
	servo->error_rate = servo->error_change/servo->dt; // for Kd

	servo->servoValue = (int)(servo->error * servoPidCfg->Kp + servo->error_area * servoPidCfg->Ki + servo->error_rate * servoPidCfg->Kd);  // PID

	if (servo->servoValue > 75) // Clamp the degree angle to its maximum value 75 for LEFT
		servo->servoValue = 75;
	else if(servo->servoValue < -72){ //Clamp the degree angle to its minimum value -72 for RIGHT
		servo->servoValue = -72;
	}
}

void Update_Wheel_EncoderConfig(WheelEncoderConfig *encoder){
//	long double wheelRotation;
//	wheelRotation = encoder->ppr / PULSE_PER_REVOLUTION;
//	encoder->distance = wheelRotation * WHEEL_CIRCUMFERENCE;
}

void Wheel_Odometry(position *pos, WheelEncoderConfig encoderL, WheelEncoderConfig encoderR){ //4.1 == 1m (need to scale)
//	long double deltaDistance, deltaTheta;

//	deltaDistance = (encoderL.distance + encoderR.distance) / 2.0;
//	deltaTheta = (encoderR.distance - encoderL.distance) / WHEEL_BASE;
//	pos->theta += deltaTheta; //Car, Orientation of car in Radians
//	pos->x += deltaDistance * cos(pos->theta); //Car, X horizontal position (X axis) in meters
//	pos->y += deltaDistance * sin(pos->theta); //Car, Y vertical position (Y axis) in meters


//	deltaDistance = (encoderL.distance + encoderR.distance) / 2.0;
//	deltaTheta = (encoderR.distance - encoderL.distance) / (2.0 * WHEEL_RADIUS);
//	pos->x += deltaDistance * cos(pos->theta + deltaTheta / 2.0); //Car, X horizontal position (X axis) in meters
//	pos->y += deltaDistance * sin(pos->theta + deltaTheta / 2.0); //Car, Y vertical position (Y axis) in meters
//	pos->theta += deltaTheta; //Car, Orientation of car in Radians
}

void Read_Gyro_DataZ(int16_t * gyroZAxis){
	HAL_I2C_Mem_Read(&hi2c1,ICM20948__I2C_SLAVE_ADDRESS_1 << 1, ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER, I2C_MEMADD_SIZE_8BIT, gyroData, 2, 0xFFFF);
	*gyroZAxis = gyroData[0] << 8 | gyroData[1];
}

void SERVO_Straight_Control(int dir){
	//Straight value of servo = 150
	int servoOutput = CENTER, offset = 0;
	char displayStr1[20];

	//Read from gyro
	Read_Gyro_DataZ(&gyroZAxis);
	currAngle += gyroZAxis / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

	servo.target_Angle = 0;
	servo.current_Angle = currAngle;

//	if(servo.error >= 0 && servo.current_Angle < 0)// If wheel is recovering from right turn, require extra effort to straighten car
//		offset = -20;

	//Call PID to calculate the servo output required to hit the target angle (straight)
	SERVO_PID_Control(&servoPidCfg, &servo, dir);

	//servo_output = round((output - MIN_PID_OUTPUT) * (MAX_SERVO_RANGE - MIN_SERVO_RANGE) / (MAX_PID_OUTPUT - MIN_PID_OUTPUT) + MIN_SERVO_RANGE);
	servoOutput = round((servo.servoValue - (-72)) * (240 - 54) / (75 - (-72)) + 54);
													//64						64
//	servoOutput += offset; //Add offsets to consider recovery from right turn scenario

//		if (servoOutput >= 145 && servoOutput <= 155) {
//			offset = servoOutput - 145;
//		  if (servo.servoValue < 0) {
//			  servoOutput = 140 - offset;
//		  } else {
//			  servoOutput = 160 + offset;
//		  }
//		}
	if(servoOutput > RIGHT_MAX)
		servoOutput = RIGHT_MAX;
	else if(servoOutput < LEFT_MAX)
		servoOutput = LEFT_MAX;

//		sprintf(displayStr1, "S: %6d\0", servoOutput);
//		OLED_ShowString(10, 40, displayStr1);

	htim1.Instance->CCR4 = servoOutput;

}

void Calculate_StraightLine(float * targetAngle, int RLDir){ //RLDir = Left/Right
	int lastTick;
	currAngle = 0.0; gyroZAxis = 0;

	lastTick = HAL_GetTick();

	do{
		if(HAL_GetTick() - lastTick >= 10){
			Read_Gyro_DataZ(&gyroZAxis);
			currAngle += gyroZAxis / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

			if(abs(currAngle - *targetAngle) < 0.01)
				break;

			lastTick = HAL_GetTick();
		}
	}while(1);
	htim8.Instance->CCR1 = 0;
	htim8.Instance->CCR2 = 0;

	if(RLDir)
		htim1.Instance->CCR4 = CENTER;
	else
		htim1.Instance->CCR4 = CENTER;
	osDelay(TURN_TIME);

	currAngle = 0.0; gyroZAxis = 0;
}

void Servo_Turn_Right_90(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -80;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 4000;
	htim8.Instance->CCR2 = 3500;

	Calculate_StraightLine(&targetAngle, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Right_90_2(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -84;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 3000;
	htim8.Instance->CCR2 = 800;

	Calculate_StraightLine(&targetAngle, 0);
	DC_MoveDistForward(400, 0.2, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Left_90(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 80;

	htim1.Instance->CCR4 = 82;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 3500;
	htim8.Instance->CCR2 = 4000;

	Calculate_StraightLine(&targetAngle, 1);

	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Left_90_2(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 82;
	DC_MoveDistForward(400, 0.015, 1);
	htim1.Instance->CCR4 = 82;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 3000;

	Calculate_StraightLine(&targetAngle, 1);
	DC_MoveDistForward(400, 0.303, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_RightLeft_Indoor(){ //Indoor
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -82;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 4000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);

	targetAngle = 82;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 4000;
	Calculate_StraightLine(&targetAngle, 1);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_LeftRight_Indoor(){ //Indoor
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 82;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 4000;
	Calculate_StraightLine(&targetAngle, 1);

	targetAngle = -82;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 4000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_LeftRight_Outdoor(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 80;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 4000;
	Calculate_StraightLine(&targetAngle, 1);

	DC_MoveDistForward(300, 0.1, 1);

	targetAngle = -82;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 4000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_RightLeft_Outdoor(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -80;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 4000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);

	DC_MoveDistForward(300, 0.1, 1);

	targetAngle = 82;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 4000;
	Calculate_StraightLine(&targetAngle, 1);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_RightLeft(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -36;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 2000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);

	targetAngle = 37;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 2000;
	Calculate_StraightLine(&targetAngle, 1);
	DC_MoveDistForward(400, 0.12, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_LeftRight(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 34;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 2000;
	Calculate_StraightLine(&targetAngle, 1);

	targetAngle = -32;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 2000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);
	DC_MoveDistForward(400, 0.102, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}


void Servo_Turn_RightLeft_Back(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 36;

	DC_MoveDistForward(400, 0.115, 1);

	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 2000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);

	targetAngle = -36;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 2000;
	Calculate_StraightLine(&targetAngle, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_LeftRight_Back(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -38;
	DC_MoveDistForward(400, 0.13, 1);

	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 2000;
	Calculate_StraightLine(&targetAngle, 1);

	targetAngle = 40;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 2000;
	htim8.Instance->CCR2 = 800;
	Calculate_StraightLine(&targetAngle, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Right_180(){
	targetAngle = -150;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 3000;
	htim8.Instance->CCR2 = 800;

	Calculate_StraightLine(&targetAngle, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Left_180(){
	targetAngle = 149;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(1);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 3000;

	Calculate_StraightLine(&targetAngle, 1);

	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Right_90_Back(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = 85;
	DC_MoveDistForward(400, 0.24, 1);
	osDelay(TURN_TIME);
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 3000;
	htim8.Instance->CCR2 = 800;

	Calculate_StraightLine(&targetAngle, 0);
	DC_MoveDistForward(400, 0.06, 0);

	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Left_90_Back(){
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;

	targetAngle = -81;
	DC_MoveDistForward(400, 0.29, 1);
	osDelay(TURN_TIME);
	htim1.Instance->CCR4 = 82;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 3000;
	Calculate_StraightLine(&targetAngle, 1);

	DC_MoveDistForward(400, 0.02, 0);
	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}


void Servo_Turn_Right_180_Back(){
	targetAngle = 149;
	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 3000;
	htim8.Instance->CCR2 = 800;

	Calculate_StraightLine(&targetAngle, 0);

	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}

void Servo_Turn_Left_180_Back(){
	targetAngle = -150;
	htim1.Instance->CCR4 = LEFT_MAX;
	osDelay(TURN_TIME);

	Set_Motor_Direction(0);
	htim8.Instance->CCR1 = 800;
	htim8.Instance->CCR2 = 3000;

	Calculate_StraightLine(&targetAngle, 1);

	//Reset Servo PID configuration
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
}


void Task2_Part1_Servo_Turn_Left_Indoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = 120;
	 osDelay(860);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(1150);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(1000);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

void Task2_Part1_Servo_Turn_Left_Outdoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = 120;
	 osDelay(900);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(1200);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(1000);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

void Task2_Part1_Servo_Turn_Right_Indoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = 200;
	 osDelay(800);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(1400);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(950);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

/*void Task2_Part1_Servo_Turn_Right_Outdoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1800); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1800); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = 190;
	 //DC_MoveDistForward(400, 1, 1);
	 osDelay(1200);

	 //htim1.Instance->CCR4 = CENTER;
	 //osDelay(200);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(1780);

	 //htim1.Instance->CCR4 = CENTER;
	 //osDelay(800);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(1220);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}*/

void Task2_Part1_Servo_Turn_Right_Outdoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1800); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 1800); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = 190;
	 osDelay(1000);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(300);

	 htim1.Instance->CCR4 = CENTER;
	 osDelay(800);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(900);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

void Task2_Part2_Servo_Turn_Left_Indoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = LEFT_MAX;
	 //DC_MoveDistForward(400, 1, 1);
	osDelay(1200);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(2700);


	 htim1.Instance->CCR4 = CENTER;
	 osDelay(1600);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2700); //Update the latest PWMValue after PID update for L
	 osDelay(1280);

	 //htim1.Instance->CCR4 = RIGHT_MAX;
	 //osDelay(950);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

void Task2_Part2_Servo_Turn_Left_Outdoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = LEFT_MAX;
	 //DC_MoveDistForward(400, 1, 1);
	osDelay(1000);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(2500);


	 htim1.Instance->CCR4 = CENTER;
	 osDelay(1200);

	 htim1.Instance->CCR4 = RIGHT_MAX;
	 osDelay(1300);

	 //htim1.Instance->CCR4 = RIGHT_MAX;
	 //osDelay(950);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

void Task2_Part2_Servo_Turn_Right_Indoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = RIGHT_MAX;
	 //DC_MoveDistForward(400, 1, 1);
	osDelay(1000);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(2500);


	 htim1.Instance->CCR4 = CENTER;
	 osDelay(1600);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(1200);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

void Task2_Part2_Servo_Turn_Right_Outdoor(){
	Set_Motor_Direction(1);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R

	htim1.Instance->CCR4 = RIGHT_MAX;
	osDelay(1000);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(2500);


	 htim1.Instance->CCR4 = CENTER;
	 osDelay(1300);

	 htim1.Instance->CCR4 = LEFT_MAX;
	 osDelay(1150);

	 htim1.Instance->CCR4 = CENTER;


	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
}

//void Task2_Part3_Servo_Return_Right(){
//	Set_Motor_Direction(1);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R
////	DC_MoveDistForward(2500,0.5,1);
//
//	// Move to the first box position
//	osDelay(200);
//
//	// Turn the wheel to the specific degree to return the box
//	 htim1.Instance->CCR4 = CENTER;
//	 osDelay(5000);
//
//
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
//	int lastTick, RLDir = 1;
//	currAngle = 0.0; gyroZAxis = 0;
//
//	targetAngle = 84;
//	htim1.Instance->CCR4 = 103;
//	osDelay(TURN_TIME);
//
//	Set_Motor_Direction(1);
//	htim8.Instance->CCR1 = 800;
//	htim8.Instance->CCR2 = 3000;
//
//	lastTick = HAL_GetTick();
//
//	do{
//		if(HAL_GetTick() - lastTick >= 10){
//			Read_Gyro_DataZ(&gyroZAxis);
//			currAngle += gyroZAxis / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
//
//			if(abs(currAngle - targetAngle) < 0.01)
//				break;
//
//			lastTick = HAL_GetTick();
//		}
//	}while(1);
//
//	if(RLDir)
//		htim1.Instance->CCR4 = CENTER;
//	else
//		htim1.Instance->CCR4 = CENTER;
//}

//void Task2_Part3_Servo_Return_Left(){
//
//	Set_Motor_Direction(1);
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 2500); //Update the latest PWMValue after PID update for L
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 2500); //Update the latest PWMValue after PID update for R
////	DC_MoveDistForward(2500,0.5,1);
//
//	// Move to the first box position
//	osDelay(200);
//
//	// Turn the wheel to the specific degree to return the box
//	 htim1.Instance->CCR4 = CENTER;
//	 osDelay(5000);
//
//
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0); //Update the latest PWMValue after PID update for L
//	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0); //Update the latest PWMValue after PID update for R
//	int lastTick, RLDir = 1;
//	currAngle = 0.0; gyroZAxis = 0;
//
//	targetAngle = 84;
//	htim1.Instance->CCR4 = 103;
//	osDelay(TURN_TIME);
//
//	Set_Motor_Direction(1);
//	htim8.Instance->CCR1 = 800;
//	htim8.Instance->CCR2 = 3000;
//
//	lastTick = HAL_GetTick();
//
//	do{
//		if(HAL_GetTick() - lastTick >= 10){
//			Read_Gyro_DataZ(&gyroZAxis);
//			currAngle += gyroZAxis / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
//
//			if(abs(currAngle - targetAngle) < 0.01)
//				break;
//
//			lastTick = HAL_GetTick();
//		}
//	}while(1);
//
//	if(RLDir)
//		htim1.Instance->CCR4 = CENTER;
//	else
//		htim1.Instance->CCR4 = CENTER;
//}

void Task2_Part3_Servo_Return_Right(){
	Servo_Turn_LeftRight_Outdoor();
}

void Task2_Part3_Servo_Return_Left(){
	Servo_Turn_RightLeft_Outdoor();
}
void DC_MoveForward(int rpm, int dir){
	uint32_t dcTick, servoTick;

		//Initiate timer to count when to trigger PID
		dcTick = HAL_GetTick();
		//Initiate timer to count when to trigger PID
		servoTick = HAL_GetTick();

		//Start the motor
		Set_Motor_Direction(dir);
		Init_Duty_Config(rpm, 0);
		//Straighten wheels
		htim1.Instance->CCR4 = CENTER_RIGHT;
		htim1.Instance->CCR4 = CENTER_LEFT;

		do{
			if(!manualMode)
				break;
			if(HAL_GetTick() - servoTick >= 10){ //Sample frequency of 10ms
				SERVO_Straight_Control(dir);
				servoTick = HAL_GetTick();
			}

			if(HAL_GetTick() - dcTick >= 1000L){
				DC_PID_Control(&dcPidCfg, &dutyL, &encoderL); //Call PID Control for LeftWheel
				DC_PID_Control(&dcPidCfg, &dutyR, &encoderR); //Call PID Control for RightWheel


				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, dutyL.pwmVal); //Update the latest PWMValue after PID update for L
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, dutyR.pwmVal); //Update the latest PWMValue after PID update for R
				dcTick = HAL_GetTick(); //Reset tick for comparison in later iteration
			}
		}while(1);

		Reset_Duty_Config();

}

void DC_MoveDistForward(int rpm, float distance, int dir){
	uint32_t dcTick, servoTick, offset = 0;

	//Check if direction changed, if change reset gyro value
	if(prevDir != dir){
		currAngle = 0.0; gyroZAxis = 0;
		servo = emptyServo;
		prevDir = dir;
	}
	//Initiate timer to count when to trigger PID
	dcTick = HAL_GetTick();
	//Initiate timer to count when to trigger PID
	servoTick = HAL_GetTick();
	//Initiate timer to count when to trigger DistEst
	distTick.startTick = HAL_GetTick();

	if(distance <= 0.45){
		rpm = 300;
		offset = 75;
	}else{
		offset = 650;
	}

	//Start the motor
	Set_Motor_Direction(dir);
	Init_Duty_Config(rpm, distance);
	currAngle = 0.0; gyroZAxis = 0;
	servo = emptyServo;
	//Straighten wheels
	//htim1.Instance->CCR4 = CENTER;
//	osDelay(TURN_TIME);


	do{
		if(HAL_GetTick() - servoTick >= 10){ //Sample frequency of 10ms
			SERVO_Straight_Control(dir);
			servoTick = HAL_GetTick();
		}

		if(HAL_GetTick() - distTick.startTick >= 20){ //Sample frequency of 50ms
			Calculate_Car_Distance(&distTick, &encoderL, &encoderR); //Calculate dist and update the currentTick

//			if(distTick.targetTick - distTick.currDistTick <= 330){ //Once target distance reached, slow the car
//				dutyL.target_Rpm -= 100;
//				dutyR.target_Rpm -= 100;
//			}

			if(distTick.currDistTick >= distTick.targetTick - offset){ //Once target distance reached, stop the car
				Reset_Duty_Config(); //Set all duty param to 0 & stop the vehicle
				break;
			}

			distTick.startTick = HAL_GetTick();
		}

		if(HAL_GetTick() - dcTick >= 1000L){
			DC_PID_Control(&dcPidCfg, &dutyL, &encoderL); //Call PID Control for LeftWheel
			DC_PID_Control(&dcPidCfg, &dutyR, &encoderR); //Call PID Control for RightWheel


			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, dutyL.pwmVal); //Update the latest PWMValue after PID update for L
			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, dutyR.pwmVal); //Update the latest PWMValue after PID update for R
			dcTick = HAL_GetTick(); //Reset tick for comparison in later iteration
		}
	}while(1);

	osDelay(500);

}

int CommandQueue_IsEmpty(CommandQueue *commandQueue){
	if(commandQueue->head == commandQueue->tail)
		return 1;

	return 0;
}

void CommandQueue_Enqueue(CommandQueue *commandQueue, uint8_t index, uint16_t val){
	commandQueue->buffer[commandQueue->head].index = index;
	commandQueue->buffer[commandQueue->head].val = val;
	commandQueue->head = (commandQueue->head + 1) % commandQueue->size; //Move head to the next empty buffer
}

void CommandQueue_Dequeue(CommandQueue *commandQueue, Command *currCmd){
	currCmd->index = commandQueue->buffer[commandQueue->tail].index; //Assign the latest command into currCommand
	currCmd->val = commandQueue->buffer[commandQueue->tail].val; //Assign the latest command into currCommand
	commandQueue->tail = (commandQueue->tail +1) % commandQueue->size; //Move tail to the next command in the buffer (Dequeue process)
}

void CurrentCommand_Clear(Command *currCmd){
	currCmd->index = 100;
	currCmd->val = 0;
}

void CurrentCommand_Pending(Command *currCmd){
	currCmd->index = 99;
}

void Task_ACK(){
	char msg[11];
	sprintf(msg, "ACK");
	HAL_UART_Transmit(&huart3, (uint8_t *) msg, 10, 0xFFFF);
}

void Task_END(){
	currTask = TASK_NONE;
}


void ADC_Read_Distance(ADC_HandleTypeDef* adc){
		HAL_ADC_Start(adc);
		HAL_ADC_PollForConversion(adc,20);
		adcVal += HAL_ADC_GetValue(adc);
		dataPoint = (dataPoint + 1) % IR_SAMPLE;
		if(dataPoint == IR_SAMPLE-1){
			obsDist_IR = IR_CONST_A / ((adcVal / dataPoint) - IR_CONST_B);
			adcVal = 0;
		}
		HAL_ADC_Stop(&hadc1);
}


void US_Read_Distance(void){
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	osDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}
			// Got error
			//Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t tick, servoTick;
	char displayStr[20];
	int dir = 1; //By default enable

	//Initiate timer to count when to trigger PID
	tick = HAL_GetTick();
	//Initiate timer to count when to trigger PID
	servoTick = HAL_GetTick();
	//Initiate timer to count when to trigger DistEst
	distTick.startTick = HAL_GetTick();

	//Straighten wheels upon starting
	htim1.Instance->CCR4 = CENTER;
	osDelay(TURN_TIME);

  /* Infinite loop */
	for(;;)
	{
//		//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//		//osDelay(1000);
	}


  //Proper termination of thread incase of accidental loop exit
  	osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_displayOLED_Task */
/**
* @brief Function implementing the displayOLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_displayOLED_Task */
void displayOLED_Task(void *argument)
{
  /* USER CODE BEGIN displayOLED_Task */
  /* Infinite loop */
//	for(;;){
//		OLED_Refresh_Gram();
//	};

	char displayStr1[20];
	char buff[20];
	char dist[20];
  for(;;)
  { //theta_r = (theta - (2 * math.pi * math.floor((theta + math.pi)/(2*math.pi)))) * 180/math.pi (Theta in degree)

//	gcvt(servo.current_Angle, 5, buff);
//	sprintf(displayStr1, "A: %s\0", buff);
//	OLED_ShowString(10, 10, displayStr1);
//
//
//	sprintf(displayStr1, "Z: %6d\0", gyroZAxis);
//	OLED_ShowString(10, 20, displayStr1);

//	sprintf(displayStr1, "T: %5d\0", commandQueue.tail);
//	OLED_ShowString(10, 10, displayStr1);
//
//	sprintf(displayStr1, "2: %5d\0", commandQueue.buffer[1].index);
//	OLED_ShowString(10, 20, displayStr1);
//
//	sprintf(displayStr1, "C: %5d\0", currCmd.index);
//	OLED_ShowString(10, 40, displayStr1);

	sprintf(displayStr1, "Msg: %s\0", aRxBuffer);
	OLED_ShowString(10, 30, displayStr1);

//	sprintf(displayStr1, "B: %5d\0", (int)batteryVal);
//	OLED_ShowString(10, 40, displayStr1);

//	gcvt(obsDist_IR, 5, dist);
//	sprintf(displayStr1, "IR: %s\0", dist);
//	OLED_ShowString(10, 30, displayStr1);
//
//	sprintf(displayStr1, "US: %d\0", Distance);
//	OLED_ShowString(10, 40, displayStr1);
	sprintf(displayStr1, "rpmA: %5d\0", dutyL.current_Rpm);
	//Arg1 pos x, Arg2 pos y
	OLED_ShowString(10, 10, displayStr1);
	sprintf(displayStr1, "rpmB: %5d\0", dutyR.current_Rpm);
	OLED_ShowString(10, 20, displayStr1);
//
//	sprintf(displayStr1, "target: %5d\0", dutyR.target_Rpm);
//	OLED_ShowString(10, 40, displayStr1);
//	sprintf(displayStr1, "tar: %5d\0", distTick.targetTick);
//	OLED_ShowString(10, 30, displayStr1);
//
//	sprintf(displayStr1, "cur: %5d\0", distTick.currDistTick);
//	OLED_ShowString(10, 40, displayStr1);


	//IR sensor
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1,20);
//	adcVal = HAL_ADC_GetValue(&hadc1);
//	dataPoint = (dataPoint + 1) % IR_SAMPLE;
//	if(dataPoint == IR_SAMPLE-1){
//		obsDist_IR = IR_CONST_A / (adcVal / dataPoint - IR_CONST_B);
//		adcVal = 0;
//	}
//	HAL_ADC_Stop(&hadc1);
//	osDelay(5);
//	sprintf(displayStr1, "IR: %.4f\0", obsDist_IR);
//	OLED_ShowString(10, 40, displayStr1);

	//Refreshing of the status on the hardware to ensure that new display value will be updated.
	OLED_Refresh_Gram();
	osDelay(100);
  }
  //Proper termination of thread incase of accidental loop exit
  	osThreadTerminate(NULL);
  /* USER CODE END displayOLED_Task */
}

/* USER CODE BEGIN Header_Command_Task */
/**
* @brief Function implementing the commandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Command_Task */
void Command_Task(void *argument)
{
  /* USER CODE BEGIN Command_Task */
  /* Infinite loop */
  for(;;)
  {
	  switch(currCmd.index) {
	  	  	 case 1: //Manual FW
	  	  	 case 2: //Manual BW
	  	  		currTask = currCmd.index == 1 ? TASK_MOVE : TASK_MOVE_BACKWARD;
	  	  		manualMode = 1;
	  	  		CurrentCommand_Pending(&currCmd);
	  	  		 break;
	  	  	 case 3:
	  	  		 currTask = TASK_FL;
	  	  		 angle = 90;
	  	  		 CurrentCommand_Pending(&currCmd);
	  	  		 break;
	  	  	 case 4:
	  	  		 currTask = TASK_FR;
	  	  		 angle = 90;
	  	  		 CurrentCommand_Pending(&currCmd);
	  	  		 break;
	  	     case 5:
	  	  	  	 currTask = TASK_BL;
	  	  	  	 angle = 90;
	  	  	     CurrentCommand_Pending(&currCmd);
	  	  	  	 break;
	  	     case 6:
	  	   	  	 currTask = TASK_BR;
	  	   	  	 angle = 90;
	  	   	  	 CurrentCommand_Pending(&currCmd);
	  	   	  	 break;
	  	     case 7:
	  	  	  	 currTask = TASK_FL;
	  	  	  	 angle = 180;
	  	  	  	 CurrentCommand_Pending(&currCmd);
	  	  	  	 break;
	  	  	 case 8:
	  	  	  	 currTask = TASK_FR;
	  	  	  	 angle = 180;
	  	  	  	 CurrentCommand_Pending(&currCmd);
	  	  	  	 break;
	  	  	 case 9:
	  	  	  	 currTask = TASK_BL;
	  	  	  	 angle = 180;
	  	  	  	 CurrentCommand_Pending(&currCmd);
	  	  	  	 break;
	  	  	 case 10:
	  	  	  	 currTask = TASK_BR;
	  	  	  	 angle = 180;
	  	  	  	 CurrentCommand_Pending(&currCmd);
	  	  	  	 break;
	  	  	 case 11:
	  	  		 currTask = TASK_FRL;
	  	  		 CurrentCommand_Pending(&currCmd);
	  	  		 break;
	  	  	 case 12:
	  	  		 currTask = TASK_FLR;
				 CurrentCommand_Pending(&currCmd);
				break;
	  	  	 case 13:
	  	  		 currTask = TASK_P1L;
	  	  		 CurrentCommand_Pending(&currCmd);
	  	  		 break;
	  	  	 case 14:
				 currTask = TASK_P1R;
				 CurrentCommand_Pending(&currCmd);
				 break;
	  	  	 case 99:
	  	  		 break;
	  	  	 default:
	  	  		 break;
	  	  	 }

	  	  osDelay(100);
  }
  /* USER CODE END Command_Task */
}

/* USER CODE BEGIN Header_MoveDistance_Task */
/**
* @brief Function implementing the moveDistTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MoveDistance_Task */
void MoveDistance_Task(void *argument)
{
  /* USER CODE BEGIN MoveDistance_Task */
  /* Infinite loop */
	float dist;
  for(;;)
  {
	  if(currTask == TASK_MOVE){
		  dist = currCmd.val;
		  CurrentCommand_Clear(&currCmd);
		  DC_MoveDistForward(600, (dist/100), 1);
		  Task_ACK();
		  Task_END();
	  }
	  else if(currTask == TASK_MOVE_BACKWARD){
		  dist = currCmd.val;
		  CurrentCommand_Clear(&currCmd);
		  //0.07
		  DC_MoveDistForward(600, (dist/100), 0);
		  Task_ACK();
		  Task_END();
	  }
	  else if(currTask == TASK_FL){
	 	CurrentCommand_Clear(&currCmd);
	 	if(angle == 90){
	 		Servo_Turn_Left_90_2();
	 	}
	 	else{
	 		Servo_Turn_Left_180();
	 	}
	 	angle = 0;
	 	Task_ACK();
	 	Task_END();
	  }
	  else if(currTask == TASK_FR){
		  CurrentCommand_Clear(&currCmd);
		  if(angle == 90){
		  	 Servo_Turn_Right_90();
		  }
		  else{
		  	 Servo_Turn_Right_180();
		  }
		  angle = 0;
		  Task_ACK();
		  Task_END();
	  }
	  else if(currTask == TASK_BL){
		 CurrentCommand_Clear(&currCmd);
		 if(angle == 90){
	  		Servo_Turn_Left_90_Back();
		 }
		 else{
	  	 	Servo_Turn_Left_180_Back();
	  	 }
		 angle = 0;
	  	 Task_ACK();
	  	 Task_END();
	  }
	  else if(currTask == TASK_BR){
		CurrentCommand_Clear(&currCmd);
	  	if(angle == 90){
	  		Servo_Turn_Right_90_Back();
	  	}
	  	else{
	  		Servo_Turn_Right_180_Back();
	  	}
	  	angle = 0;
	  	Task_ACK();
	  	Task_END();
	  }
	  else if(currTask == TASK_FRL){
		  CurrentCommand_Clear(&currCmd);
		  Servo_Turn_RightLeft_Outdoor();
		  Task_ACK();
		  Task_END();
	  }
	  else if(currTask == TASK_FLR){
		  CurrentCommand_Clear(&currCmd);
		  Servo_Turn_LeftRight_Outdoor();
		  Task_ACK();
		  Task_END();
	  }
	  else if(currTask == TASK_FL2){
		  CurrentCommand_Clear(&currCmd);
		  Servo_Turn_Left_90_2();
		  osDelay(1000);
		  Task_ACK();
		  Task_END();
	  }
	  else if(currTask == TASK_FR2){
		  CurrentCommand_Clear(&currCmd);
	      Servo_Turn_Right_90_2();
		  osDelay(1000);
		  Task_ACK();
	      Task_END();
	  }
	  else if(currTask == TASK_P1L){
		  part = currCmd.val;
		  CurrentCommand_Clear(&currCmd);
		  if(part == 1){
			  Task2_Part1_Servo_Turn_Left_Outdoor();
		  }
		  else if(part == 2){
			  Task2_Part2_Servo_Turn_Left_Outdoor();
		  }
		  else if(part == 3){
			  Task2_Part3_Servo_Return_Right();
		  }
		  osDelay(1000);
		  Task_ACK();
		  Task_END();
	  }

	  else if(currTask == TASK_P1R){
		  part = currCmd.val;
		  CurrentCommand_Clear(&currCmd);
		  if(part == 1){
			  Task2_Part1_Servo_Turn_Right_Outdoor();
		  }
		  else if(part == 2){
			  Task2_Part2_Servo_Turn_Right_Outdoor();
		  }
		  else if(part == 3){

			  Task2_Part3_Servo_Return_Left();
		  }
		  osDelay(1000);
		  Task_ACK();
		  Task_END();
	 }
  }
  /* USER CODE END MoveDistance_Task */
}

/* USER CODE BEGIN Header_servoMotor_Task */
/**
* @brief Function implementing the servoMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servoMotor_Task */
void servoMotor_Task(void *argument)
{
  /* USER CODE BEGIN servoMotor_Task */
//  /* Infinite loop */
	//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  for(;;)
  {
	  //Servo motor can turn up to 180 degrees but this is not applicable in our wheel scenario
	  //Doing so may damage the servoMotor. Thus we need to be cautious and not input extreme values during testing of the servo motor
	  //For instance, we need to refer to the servoMotor manual to understand what it is about

	  //Extreme Left: 70(Min) value must be above 70 to avoid damage
	  //Centre value: 160-165(straight)
	  //Extreme right: 230(Max) value must be below 230 to avoid damage
	 }


	//Proper termination of thread incase of accidental loop exit
	osThreadTerminate(NULL);
  /* USER CODE END servoMotor_Task */
}

/* USER CODE BEGIN Header_BatterReader_Task */
/**
* @brief Function implementing the batteryReader thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BatterReader_Task */
void BatterReader_Task(void *argument)
{
  /* USER CODE BEGIN BatterReader_Task */
  /* Infinite loop */
  for(;;)
  {
//	HAL_ADC_Start(&hadc2);
//	HAL_ADC_PollForConversion(&hadc2,20);
//	batteryVal = HAL_ADC_GetValue(&hadc2) / 1421.752066 * 100;
//	HAL_ADC_Stop(&hadc2);
//	osDelay(30000); // check battery level every 30 seconds
  }
  /* USER CODE END BatterReader_Task */
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
