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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t           channel;
} ServoChannel;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_COUNT      10U     // motor_id 0~9
#define CAN_SERVO_STDID  5U      // ROS -> STM 서보 노드의 CAN 표준 ID
#define CAN_MIN_DLC      3U      // [motor_id][angle_H][angle_L]
#define ANGLE_X10_MAX    1800U   // 180.0도
#define SERVO_MIN_US     500U    // 펄스폭(CCR) 하드 리밋 하한. 서보별 보정값도 이 안에 있어야 한다
#define SERVO_MAX_US     2500U   // 펄스폭(CCR) 하드 리밋 상한

#define FSR_COUNT        6U      // 압력센서(FSR) 6개 (Stm32pinmap.md 4장)
#define FSR_PERIOD_MS    100U    // 측정·송신 주기
#define CAN_FSR_STDID    6U      // STM -> ROS 압력센서 프레임의 CAN 표준 ID [미확정: ROS 쪽과 맞출 것]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// index = motor_id (Stm32servosepc.md 4장, B안). 핀/채널은 hal_msp.c와 일치해야 한다
static const ServoChannel servo[SERVO_COUNT] = {
	{&htim3, TIM_CHANNEL_1}, {&htim3, TIM_CHANNEL_2},	// 0 F1_1 PA6,  1 F1_2 PA7
	{&htim4, TIM_CHANNEL_1}, {&htim4, TIM_CHANNEL_4},	// 2 F2_1 PB6,  3 F2_2 PB9
	{&htim2, TIM_CHANNEL_1}, {&htim4, TIM_CHANNEL_2},	// 4 F3_1 PA0,  5 F3_2 PB7
	{&htim4, TIM_CHANNEL_3},							// 6 F_ROT PB8
	{&htim3, TIM_CHANNEL_3}, {&htim3, TIM_CHANNEL_4},	// 7 W_X PB0,   8 W_YAW PB1
	{&htim2, TIM_CHANNEL_2},							// 9 W_PITCH PA1
};

// 서보별 펄스폭(us): 0.0도일 때 / 180.0도일 때 (index = motor_id).
// tools/calibrate.py가 아래 두 줄을 실측값으로 갱신한다. 끝값에서 서보가 떨리거나 뜨거우면 안쪽으로 줄인다.
// 두 값을 서로 바꿔 적으면 그 서보의 회전 방향이 뒤집힌다
static const uint16_t servo_us_at_0[SERVO_COUNT]   = { 500, 500, 500, 500, 500, 500, 500, 500, 500, 500 };
static const uint16_t servo_us_at_180[SERVO_COUNT] = { 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500 };

// 콜백(ISR)이 쓰고 main 루프가 읽는다
static volatile uint16_t target_x10[SERVO_COUNT];
static volatile uint8_t  dirty[SERVO_COUNT];

// index = 센서 번호 - 1 (Stm32pinmap.md 4장). 핀은 hal_msp.c의 ADC1 아날로그 설정과 일치해야 한다
static const uint32_t fsr_channel[FSR_COUNT] = {
	ADC_CHANNEL_11, ADC_CHANNEL_12,	// 센서1 PC1, 센서2 PC2
	ADC_CHANNEL_14, ADC_CHANNEL_15,	// 센서3 PC4, 센서4 PC5
	ADC_CHANNEL_2,  ADC_CHANNEL_3,	// 센서5 PA2, 센서6 PA3
};

// 마지막 측정값, 12비트 원시값 0~4095. main 루프만 쓴다.
// 읽는 코드가 없어도 최적화로 지워지지 않고 디버거(SWD)에서 실시간으로 보이도록 volatile
static volatile uint16_t fsr_raw[FSR_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
static void Servo_Start(void);
static void Servo_Update(void);
static void Can_Set(void);
static void Fsr_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 각도(x10) -> 펄스폭(us). 타이머 1틱 = 1us 이므로 CCR 값이 곧 펄스폭이다.
// at_0 > at_180 이어도(방향 반전) 부호 있는 계산이라 그대로 동작한다
static uint32_t angle_to_ccr(uint32_t id, uint16_t angle_x10)
{
	int32_t at0   = (int32_t)servo_us_at_0[id];
	int32_t at180 = (int32_t)servo_us_at_180[id];
	int32_t us    = at0 + ((int32_t)angle_x10 * (at180 - at0)) / (int32_t)ANGLE_X10_MAX;

	if (us < (int32_t)SERVO_MIN_US) {
		us = (int32_t)SERVO_MIN_US;
	} else if (us > (int32_t)SERVO_MAX_US) {
		us = (int32_t)SERVO_MAX_US;
	}
	return (uint32_t)us;
}

// 부팅 시 1회. 10채널 모두 CCR=0(펄스 없음)으로 시작해 첫 명령 전에는 서보가 움직이지 않는다
static void Servo_Start(void)
{
	for (uint32_t i = 0; i < SERVO_COUNT; i++) {
		__HAL_TIM_SET_COMPARE(servo[i].htim, servo[i].channel, 0);
		if (HAL_TIM_PWM_Start(servo[i].htim, servo[i].channel) != HAL_OK) {
			Error_Handler();
		}
	}
}

// main 루프에서 호출. dirty인 채널만 CCR을 갱신한다 (HAL_Delay 금지)
static void Servo_Update(void)
{
	for (uint32_t i = 0; i < SERVO_COUNT; i++) {
		if (dirty[i]) {
			// dirty를 먼저 내리고 target을 읽는다. 사이에 ISR이 새 값을 써도 놓치지 않는다
			dirty[i] = 0;
			__HAL_TIM_SET_COMPARE(servo[i].htim, servo[i].channel, angle_to_ccr(i, target_x10[i]));
		}
	}
}

//main에서 1회 호출. accept-all 필터(FIFO0), ID 검사는 수신 콜백에서 한다
static void Can_Set(void)
{
	CAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterBank= 0;
	sFilterConfig.FilterMode= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale= CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh= 0x0000;
	sFilterConfig.FilterIdLow= 0x0000;
	sFilterConfig.FilterMaskIdHigh= 0x0000;
	sFilterConfig.FilterMaskIdLow= 0x0000;
	sFilterConfig.FilterFIFOAssignment= CAN_RX_FIFO0;
	sFilterConfig.FilterActivation= ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
}

// CAN 수신 인터럽트 (MX_CAN1에서 CAN1_RX0_IRQn 활성화됨)
// 검증 후 target/dirty만 저장하고 끝낸다. CCR은 건드리지 않는다
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
		return;
	}

	// 규칙 위반 프레임은 조용히 버린다
	if (rxHeader.IDE != CAN_ID_STD || rxHeader.RTR != CAN_RTR_DATA ||
	    rxHeader.StdId != CAN_SERVO_STDID || rxHeader.DLC < CAN_MIN_DLC) {
		return;
	}

	uint8_t id = rxData[0];
	if (id >= SERVO_COUNT) {
		return;
	}

	// 부호 있는 16비트로 읽어 음수는 0도, 1800 초과는 180도로 자른다
	int16_t raw = (int16_t)(((uint16_t)rxData[1] << 8) | rxData[2]);
	uint16_t angle_x10;
	if (raw < 0) {
		angle_x10 = 0;
	} else if (raw > (int16_t)ANGLE_X10_MAX) {
		angle_x10 = ANGLE_X10_MAX;
	} else {
		angle_x10 = (uint16_t)raw;
	}

	target_x10[id] = angle_x10;
	dirty[id] = 1;
}

// ADC1으로 채널 하나를 한 번 변환한다. 소프트웨어 트리거, 단일 변환, 폴링.
// 샘플링 144 cycles + 변환 12 cycles, ADC 클럭 21MHz(PCLK2 84MHz / 4) → 약 7.4us.
// FSR 분압 회로처럼 출력 임피던스가 큰 입력이라 샘플링 시간을 길게 준다
static HAL_StatusTypeDef Fsr_ReadChannel(uint32_t channel, uint16_t *out)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel      = channel;
	sConfig.Rank         = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK ||
	    HAL_ADC_Start(&hadc1) != HAL_OK ||
	    HAL_ADC_PollForConversion(&hadc1, 2) != HAL_OK) {
		return HAL_ERROR;
	}
	*out = (uint16_t)HAL_ADC_GetValue(&hadc1);
	return HAL_OK;
}

// 압력센서 프레임 (STM -> ROS): StdId CAN_FSR_STDID, DLC 6, byte[k] = 센서 k+1 값 0~255.
// 0~255는 12비트 원시값을 4비트 내린 값이다 (0~4095 → 0~255)
static void Fsr_Send(void)
{
	CAN_TxHeaderTypeDef txHeader;
	uint8_t  txData[FSR_COUNT];
	uint32_t txMailbox;

	// 빈 메일박스가 없으면 이번 주기는 건너뛴다. 기다리지 않는다
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
		return;
	}

	txHeader.StdId              = CAN_FSR_STDID;
	txHeader.ExtId              = 0;
	txHeader.IDE                = CAN_ID_STD;
	txHeader.RTR                = CAN_RTR_DATA;
	txHeader.DLC                = FSR_COUNT;
	txHeader.TransmitGlobalTime = DISABLE;
	for (uint32_t i = 0; i < FSR_COUNT; i++) {
		txData[i] = (uint8_t)(fsr_raw[i] >> 4);
	}
	// 자동 재전송이 꺼져 있어(AutoRetransmission DISABLE) 실패한 프레임은 버려지고 다음 주기에 새로 보낸다
	(void)HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// main 루프에서 호출. FSR_PERIOD_MS마다 6채널을 읽고 CAN으로 보낸다.
// HAL_Delay 대신 틱을 비교하므로 주기 사이에는 바로 반환한다. 읽는 데는 6채널 합쳐 약 50us
static void Fsr_Update(void)
{
	static uint32_t last_ms;
	uint32_t now = HAL_GetTick();

	if ((uint32_t)(now - last_ms) < FSR_PERIOD_MS) {
		return;
	}
	last_ms = now;

	for (uint32_t i = 0; i < FSR_COUNT; i++) {
		uint16_t v;
		if (Fsr_ReadChannel(fsr_channel[i], &v) == HAL_OK) {
			fsr_raw[i] = v;		// 실패하면 이전 값을 유지한다
		}
	}
	Fsr_Send();
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
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // 부팅 확인용: 클럭·GPIO 초기화를 통과하면 온보드 초록 LED(LD4, PD12)를 켠다
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  Servo_Start();
  Can_Set();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Servo_Update();
    Fsr_Update();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SCK_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(SPI1_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
