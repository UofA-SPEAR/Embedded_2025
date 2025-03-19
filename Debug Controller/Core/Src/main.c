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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	DISPLAY_LEFT = 0U,
	DISPLAY_RIGHT
} DISPLAY_SELECT;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CAN_DATA_SIZE 4
#define DISPLAY_NOTHING 255

#define CAN_ID 0b1111

#define CAN_ID_MASK 		0b00000000000011110000000000000000
#define CAN_ACTUATOR_MASK 	0b00000000000000001111000000000000

#define CAN_QUEUE_SiZE 10

#define JSTICK_R_MAX 0x0be6
#define JSTICK_L_MAX 0x0eec
#define DEAD_ZONE_SIZE 0.15
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc4;

CAN_HandleTypeDef hcan;

osThreadId MainTaskHandle;
osThreadId InputPollHandle;
osThreadId ManageCanBusHandle;
/* USER CODE BEGIN PV */
struct InputStates {
	uint8_t sw1: 1;
	uint8_t sw2: 1;
	uint8_t sw3: 1;
	uint8_t sw4: 1;
	uint8_t blt: 1;
	uint8_t blb: 1;
	uint8_t brt: 1;
	uint8_t brb: 1;
};

struct InputStates inputStateCurr;
struct InputStates inputStatePrev;

struct {
	uint32_t blt;
	uint32_t blb;
	uint32_t brt;
	uint32_t brb;
} debounceTicks;

struct {
	float r;
	float l;
	uint32_t rraw;
	uint32_t lraw;
} joystickValues;

struct ModeFunctions {
	void (*Entry)(void);
	void (*Loop)(void);
	void (*Exit)(void);
};

uint8_t selectedID = 0;
uint8_t selectedActuator = 0;
uint8_t selectedIDPrev = 0;
uint8_t selectedActuatorPrev = 0;
uint8_t debugId = CAN_ID;
uint8_t priority = 1;
uint8_t commandId = 0x03;

CAN_TxHeaderTypeDef CAN_TxHeader;
CAN_RxHeaderTypeDef CAN_RxHeader;
uint32_t CAN_TxMailbox = 0;
uint8_t CAN_TxData[CAN_DATA_SIZE] = {};
uint8_t CAN_RxData[CAN_DATA_SIZE] = {};

uint32_t CAN_TxHeader_Queue[CAN_QUEUE_SiZE] = {};
uint32_t CAN_TxData_Queue[CAN_QUEUE_SiZE] = {};
uint8_t CAN_Tx_Queue_I = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC4_Init(void);
void StartMainTask(void const * argument);
void StartInputPoll(void const * argument);
void StartManageCanBus(void const * argument);

/* USER CODE BEGIN PFP */
void SetDisplay(DISPLAY_SELECT select, uint8_t value);
float map(float x,float minVal, float maxVal, float nMin, float nMax);
bool Debounce(uint32_t* dTick, bool toCheck, uint32_t debounceSet);
void DeadZone(float* val, float zone);
void CAN_Filter(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* CAN_TxHeader);
void CAN_Tx_Queue_Add(uint32_t header, uint32_t data);
void CAN_Tx_Queue_Clear(void);

void ModeDebugEntry(void);
void ModeDebugLoop(void);
void ModeDebugExit(void);
void ModeDriveEntry(void);
void ModeDriveLoop(void);
void ModeDriveExit(void);
void ModeNothingEntry(void);
void ModeNothingLoop(void);
void ModeNothingExit(void);
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
  MX_CAN_Init();
  MX_ADC2_Init();
  MX_ADC4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc4);

  CAN_Filter(&hcan, &CAN_TxHeader);
  HAL_CAN_Start(&hcan);
  /* USER CODE END 2 */

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
  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityNormal, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* definition and creation of InputPoll */
  osThreadDef(InputPoll, StartInputPoll, osPriorityNormal, 0, 128);
  InputPollHandle = osThreadCreate(osThread(InputPoll), NULL);

  /* definition and creation of ManageCanBus */
  osThreadDef(ManageCanBus, StartManageCanBus, osPriorityNormal, 0, 128);
  ManageCanBusHandle = osThreadCreate(osThread(ManageCanBus), NULL);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = ENABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

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
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB7
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SetDisplay(DISPLAY_SELECT select, uint8_t value){
	uint8_t digits[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
	GPIO_TypeDef* gpio;
	switch (select){
	case (DISPLAY_LEFT):
		gpio = GPIOB;
		break;
	case (DISPLAY_RIGHT):
		gpio = GPIOA;
		break;
	}
	HAL_GPIO_WritePin(gpio, digits[8], GPIO_PIN_SET);
	if (value == DISPLAY_NOTHING){
	} else if (value == 11){
		HAL_GPIO_WritePin(gpio, 0x49, GPIO_PIN_RESET);
	} else if (value == 10){
		HAL_GPIO_WritePin(gpio, digits[8], GPIO_PIN_RESET);
	} else if (value < 10){
		HAL_GPIO_WritePin(gpio, digits[value], GPIO_PIN_RESET);
	}
}

float map(float x,float minVal, float maxVal, float nMin, float nMax){
	if (maxVal == minVal) return 0;
	return nMin + ((x - minVal) * (nMax - nMin)) / (maxVal - minVal);
}

bool Debounce(uint32_t* dTick, bool toCheck, uint32_t debounceSet){
	if (*dTick > 0) (*dTick)--;
	if (toCheck == true){
		if (*dTick > 0) return false;
		*dTick = debounceSet;
		return true;
	} else {
		*dTick = 0;
		return false;
	}
}

void DeadZone(float* val, float zone){
	if(*val < -zone){
		*val = (*val+zone)/(1-zone);
	} else if(*val > zone){
		*val = (*val-zone)/(1-zone);;
	} else {
		*val = 0;
	}
}

void CAN_Filter(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* CAN_TxHeader)
{ // This function initializes the CAN filter for the board.
  // The CANID ports are named for their respective address bits, i.e., 0 to the 0th bit.

    // Setting up the TxHeader
    CAN_TxHeader->IDE = CAN_ID_EXT; // Extended identifier, not the standard length.
    CAN_TxHeader->RTR = CAN_RTR_DATA; // Specifying data frames, not remote frames.
    CAN_TxHeader->DLC = CAN_DATA_SIZE; // CAN_SIZE_DATA; //The data size (5 bytes)
    CAN_TxHeader->ExtId = 0; // Needs to be changed depending on the frame

    CAN_FilterTypeDef CAN_FILTER_CONFIG; // Declaring the filter structure.
    CAN_FILTER_CONFIG.FilterFIFOAssignment = CAN_FILTER_FIFO0; // Choosing the FIFO0 set.
    CAN_FILTER_CONFIG.FilterIdHigh = (uint32_t)(CAN_ID >> 1);
    CAN_FILTER_CONFIG.FilterIdLow = (uint32_t)((CAN_ID << 15) & 0xFFFF);
    CAN_FILTER_CONFIG.FilterMaskIdHigh = (uint32_t)(CAN_ID_MASK >> 16);
    CAN_FILTER_CONFIG.FilterMaskIdLow = (uint32_t)(CAN_ID_MASK & 0xFFFF);
    CAN_FILTER_CONFIG.FilterBank = 0;
    CAN_FILTER_CONFIG.FilterMode = CAN_FILTERMODE_IDMASK; // Using the mask mode to ignore certain bits.
    CAN_FILTER_CONFIG.FilterScale = CAN_FILTERSCALE_32BIT; // Using the extended ID so 32bit filters.
    CAN_FILTER_CONFIG.FilterActivation = CAN_FILTER_ENABLE; // Enabling the filter.
    HAL_CAN_ConfigFilter(hcan, &CAN_FILTER_CONFIG);
}

void CAN_Tx_Queue_Add(uint32_t header, uint32_t data){
	if (!(CAN_Tx_Queue_I < CAN_QUEUE_SiZE)) return;
	CAN_Tx_Queue_I++;
	CAN_TxHeader_Queue[CAN_Tx_Queue_I-1] = header;
	CAN_TxData_Queue[CAN_Tx_Queue_I-1] = data;
}

void CAN_Tx_Queue_Clear(void){
	CAN_Tx_Queue_I = 0;
}

//Mode 0 functions: Debug mode
void ModeDebugEntry(void){
	SetDisplay(DISPLAY_LEFT, 0);
	SetDisplay(DISPLAY_RIGHT, 0);
	selectedID = 0;
	selectedActuator = 0;
	commandId = 0x03;

	debounceTicks.blt = 0;
	debounceTicks.blb = 0;
	debounceTicks.brt = 0;
	debounceTicks.brb = 0;
}

void ModeDebugLoop(void){
	if (Debounce(&(debounceTicks.blt), inputStateCurr.blt, 20) && selectedID < 9) selectedID++;
	if (Debounce(&(debounceTicks.blb), inputStateCurr.blb, 20) && selectedID > 0) selectedID--;
	if (Debounce(&(debounceTicks.brt), inputStateCurr.brt, 20) && selectedActuator < 9) selectedActuator++;
	if (Debounce(&(debounceTicks.brb), inputStateCurr.brb, 20) && selectedActuator > 0) selectedActuator--;
	SetDisplay(DISPLAY_LEFT, selectedID);
	SetDisplay(DISPLAY_RIGHT, selectedActuator);
	if (selectedID != selectedIDPrev || selectedActuator != selectedActuatorPrev){
		CAN_Tx_Queue_Add(
			priority<<24|commandId<<16|selectedIDPrev<<12|selectedActuatorPrev<<8|debugId<<4,
			0
		);
		selectedIDPrev = selectedID;
		selectedActuatorPrev = selectedActuator;
		return;
	}
	selectedIDPrev = selectedID;
	selectedActuatorPrev = selectedActuator;

	float data = map(joystickValues.l, -1, 1, 0.1, 1) * -joystickValues.r;
	uint32_t ndata = 0;
	memcpy(&ndata, &data, sizeof data);

	CAN_Tx_Queue_Add(
		priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
		ndata
	);
}

void ModeDebugExit(void){
	SetDisplay(DISPLAY_LEFT, DISPLAY_NOTHING);
	SetDisplay(DISPLAY_RIGHT, DISPLAY_NOTHING);

	CAN_Tx_Queue_Clear();
	CAN_Tx_Queue_Add(
		priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
		0
	);
}

//Mode 1 functions: Drive mode
void ModeDriveEntry(void){
	SetDisplay(DISPLAY_LEFT, 8);
	SetDisplay(DISPLAY_RIGHT, 8);
	selectedID = 0;
	selectedActuator = 0;
	commandId = 0x03;

	debounceTicks.blt = 0;
	debounceTicks.blb = 0;
	debounceTicks.brt = 0;
	debounceTicks.brb = 0;

	CAN_Tx_Queue_Clear();
}

void ModeDriveLoop(void){
	uint8_t drive[6] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
	float driveMod[6] = {1, 1, 1, -1, -1, -1};
	uint8_t steer[4] = {0x22, 0x23, 0x24, 0x25}; //BL, FL, BR, FR
	float steerRight[4] = {-0.75, 0.75, -1, 1}; //BL, FL, BR, FR
	float steerLeft[4] = {-1, 1, -0.75, 0.75}; //BL, FL, BR, FR
	float steerAngle = 0.5 * 50; //rad

	for (uint8_t i=0; i<6; i++){
		selectedID = (drive[i] >> 4) & 0x0F;
		selectedActuator = (drive[i] >> 0) & 0x0F;
		commandId = 0x03;
		float data = joystickValues.r * driveMod[i] * 0.6;
		uint32_t ndata = 0;
		memcpy(&ndata, &data, sizeof data);
		CAN_Tx_Queue_Add(
			priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
			ndata
		);
	}
	for (uint8_t i=0; i<4; i++){
		selectedID = (steer[i] >> 4) & 0x0F;
		selectedActuator = (steer[i] >> 0) & 0x0F;
		commandId = 0x02;
		if (joystickValues.l > 0){
			float data = joystickValues.l * steerRight[i] * -steerAngle;
			uint32_t ndata = 0;
			memcpy(&ndata, &data, sizeof data);
			CAN_Tx_Queue_Add(
				priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
				ndata
			);
		} else {
			float data = joystickValues.l * steerLeft[i] * -steerAngle;
			uint32_t ndata = 0;
			memcpy(&ndata, &data, sizeof data);
			CAN_Tx_Queue_Add(
				priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
				ndata
			);
		}
	}
	osDelay(80);
}

void ModeDriveExit(void){
	uint8_t drive[6] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
	uint8_t steer[4] = {0x22, 0x23, 0x24, 0x25}; //BL, FL, BR, FR

	CAN_Tx_Queue_Clear();
	for (uint8_t i=0; i<6; i++){
		selectedID = (drive[i] >> 4) & 0x0F;
		selectedActuator = (drive[i] >> 0) & 0x0F;
		commandId = 0x03;
		CAN_Tx_Queue_Add(
			priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
			0
		);
	}
	for (uint8_t i=0; i<4; i++){
		selectedID = (steer[i] >> 4) & 0x0F;
		selectedActuator = (steer[i] >> 0) & 0x0F;
		commandId = 0x02;
		CAN_Tx_Queue_Add(
			priority<<24|commandId<<16|selectedID<<12|selectedActuator<<8|debugId<<4,
			0
		);
	}
}

//Mode 2 and 3 functions: Unset mode
void ModeNothingEntry(void){
	SetDisplay(DISPLAY_LEFT, 11);
	SetDisplay(DISPLAY_RIGHT, 11);
}

void ModeNothingLoop(void){

}

void ModeNothingExit(void){
	SetDisplay(DISPLAY_LEFT, DISPLAY_NOTHING);
	SetDisplay(DISPLAY_RIGHT, DISPLAY_NOTHING);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t modeCurr = 0;
	uint8_t modePrev = 4;

	struct ModeFunctions modeFuncs[5];
	modeFuncs[0].Entry = &ModeDebugEntry;
	modeFuncs[0].Loop = &ModeDebugLoop;
	modeFuncs[0].Exit = &ModeDebugExit;
	modeFuncs[1].Entry = &ModeDriveEntry;
	modeFuncs[1].Loop = &ModeDriveLoop;
	modeFuncs[1].Exit = &ModeDriveExit;
	modeFuncs[2].Entry = &ModeNothingEntry;
	modeFuncs[2].Loop = &ModeNothingLoop;
	modeFuncs[2].Exit = &ModeNothingExit;
	modeFuncs[3].Entry = &ModeNothingEntry;
	modeFuncs[3].Loop = &ModeNothingLoop;
	modeFuncs[3].Exit = &ModeNothingExit;
	modeFuncs[4].Entry = &ModeNothingEntry;
	modeFuncs[4].Loop = &ModeNothingLoop;
	modeFuncs[4].Exit = &ModeNothingExit;

	/* Infinite loop */
	for(;;)
	{
		modeCurr = inputStateCurr.sw4? 3: modeCurr;
		modeCurr = inputStateCurr.sw3? 2: modeCurr;
		modeCurr = inputStateCurr.sw2? 1: modeCurr;
		modeCurr = inputStateCurr.sw1? 0: modeCurr;

		if (modeCurr != modePrev){
			modeFuncs[modePrev].Exit();
			modeFuncs[modeCurr].Entry();
			modePrev = modeCurr;
			osDelay(40);
			continue;
		}
		modeFuncs[modeCurr].Loop();

		osDelay(20);
	}
	osThreadTerminate(NULL);

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInputPoll */
/**
* @brief Function implementing the InputPoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputPoll */
void StartInputPoll(void const * argument)
{
  /* USER CODE BEGIN StartInputPoll */
	bool convert(GPIO_PinState state){
		if (state == GPIO_PIN_SET){
			return true;
		} else if (state == GPIO_PIN_RESET){
			return false;
		} else {
			return false;
		}
	}
	/* Infinite loop */
	for(;;)
	{
		//Poll for button and switch digital inputs.
		inputStateCurr.sw1 = convert(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
		inputStateCurr.sw2 = convert(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
		inputStateCurr.sw3 = convert(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14));
		inputStateCurr.sw4 = false;
		inputStateCurr.blt = convert(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));
		inputStateCurr.blb = convert(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8));
		inputStateCurr.brt = convert(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15));
		inputStateCurr.brb = convert(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14));

		//Poll for joystick analog inputs.
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
		joystickValues.rraw = HAL_ADC_GetValue(&hadc2);
		joystickValues.r = -map((float)(joystickValues.rraw), 0, JSTICK_R_MAX, -1, 1);
		DeadZone(&(joystickValues.r), DEAD_ZONE_SIZE);
		HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
		joystickValues.lraw = HAL_ADC_GetValue(&hadc4);
		joystickValues.l = map((float)(joystickValues.lraw), 0, JSTICK_L_MAX, -1, 1);
		DeadZone(&(joystickValues.l), DEAD_ZONE_SIZE);

		osDelay(40);
	}
	osThreadTerminate(NULL);
  /* USER CODE END StartInputPoll */
}

/* USER CODE BEGIN Header_StartManageCanBus */
/**
* @brief Function implementing the ManageCanBus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartManageCanBus */
void StartManageCanBus(void const * argument)
{
  /* USER CODE BEGIN StartManageCanBus */
	/* Infinite loop */
	for(;;)
	{
		osDelay(10);
		if (!(CAN_Tx_Queue_I > 0)) continue;
		CAN_TxHeader.ExtId = CAN_TxHeader_Queue[CAN_Tx_Queue_I-1];
		CAN_TxData[0] = CAN_TxData_Queue[CAN_Tx_Queue_I-1] >> 24;
		CAN_TxData[1] = CAN_TxData_Queue[CAN_Tx_Queue_I-1] >> 16;
		CAN_TxData[2] = CAN_TxData_Queue[CAN_Tx_Queue_I-1] >> 8;
		CAN_TxData[3] = CAN_TxData_Queue[CAN_Tx_Queue_I-1] >> 0;
		HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, CAN_TxData, &CAN_TxMailbox);
		if (result != HAL_OK) continue;
		CAN_Tx_Queue_I--;
	}
	osThreadTerminate(NULL);
  /* USER CODE END StartManageCanBus */
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
