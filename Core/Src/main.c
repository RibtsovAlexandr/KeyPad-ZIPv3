/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>
#include "cmsis_os.h"
#include "DWIN_menu_logic.c"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// KeyPad-ZIP user LED (PC13)
#define LED_PORT    GPIOC
#define LED_PIN     GPIO_PIN_13
// Rows defines
#define ROWS_PORT   GPIOC
#define ROWS_PINS   6
//#define ROW_A_PIN   GPIO_PIN_4
//#define ROW_B_PIN   GPIO_PIN_5
//#define ROW_C_PIN   GPIO_PIN_6
//#define ROW_D_PIN   GPIO_PIN_7
//#define ROW_E_PIN   GPIO_PIN_8
//#define ROW_F_PIN   GPIO_PIN_9
// Columns defines
#define COLS_PORT   GPIOC
#define COLS_PINS   4
//#define COL1_PIN    GPIO_PIN_0
//#define COL2_PIN    GPIO_PIN_1
//#define COL3_PIN    GPIO_PIN_2
//#define COL4_PIN    GPIO_PIN_3

#define LED_PORT_CLK_ENABLE   __HAL_RCC_GPIOC_CLK_ENABLE
#define ROWS_PORT_CLK_ENABLE  __HAL_RCC_GPIOC_CLK_ENABLE
#define COLS_PORT_CLK_ENABLE  __HAL_RCC_GPIOC_CLK_ENABLE
#define LED_CLK_TICKS   500 //BIG LAMP Togle pin timeout in milliseconds

#define PROGRAM_LOG_LEVEL_NO  0
#define PROGRAM_LOG_LEVEL_MED 1
#define PROGRAM_LOG_LEVEL_MAX 2
const uint8_t currentLogLevel = 0;

// Keypad Dimension
const uint8_t ROWS = ROWS_PINS; // Realy I need this? :))
const uint8_t COLS = COLS_PINS; // Realy I need this? :))
const uint8_t KeyPushedErrorTicksCount = 3; // contact error ticks counter (must be 2 .. 255)
const uint8_t KeypadScanPeriod = 10; // Period of Keypad Scaning (in miliseconds)

volatile uint8_t counterCOLS  = 0;
volatile uint8_t PinSCode     = 0;    // Pin Pad Scan Code
// Cycles counter for each key
volatile uint8_t CountKeys[ROWS_PINS][COLS_PINS] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

// Scan (char) codes array
const char ScanKeys[ROWS_PINS][COLS_PINS] = {
  { '#', '9', '6', '3' },
  { '0', '8', '5', '2' },
  { '*', '7', '4', '1' },
  { 'R', 'q', 'U', 'D' },
  { 'Q', 'L', 'e', 'd' },
  { 'c', 'b', 'a', 'd' }
};
// just scanned Key
char Key_Pushed ='.';
bool Key_Obtained = true;
bool Mode_Measuring = false;
bool Mode_Measuring_AUTO = true;

/* for DWIN
#define REQUEST_FRAME_BUFFER 7
#define WRITE_FRAME_BUFFER 8
#define READ_FRAME_BUFFER_DEFAULT 9 
*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myLamp */
osThreadId_t myLampHandle;
const osThreadAttr_t myLamp_attributes = {
  .name = "myLamp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBtnScan */
osThreadId_t myBtnScanHandle;
const osThreadAttr_t myBtnScan_attributes = {
  .name = "myBtnScan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void StartLamp(void *argument);
void StartBtnScan(void *argument);

/* USER CODE BEGIN PFP */
void KeyPad_Init(void);
void TimerCallScan (void);
void ReadRows (uint8_t ColIndex);
void DeCodePushedRow (uint8_t PushedRow, uint8_t Col);
void DeCodeReleasedRow (uint8_t ReleasedRow, uint8_t Col);

// void DWIN_setPage(uint8_t pageID);
// uint8_t DWIN_getPage();

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
  uint8_t frame8[WRITE_FRAME_BUFFER] = { 0x5A, 0xA5, 0x05, 0x82, 0x50, 0x01, 0x00, 0x00};
  uint8_t frame8_length = WRITE_FRAME_BUFFER;
  uint8_t REQ_frame[REQUEST_FRAME_BUFFER] = { 0x5A, 0xA5, 0x04, 0x83, 0x51, 0x03, 0x01};
  uint8_t REQ_frame_length = REQUEST_FRAME_BUFFER;
  uint8_t READ_frame[READ_FRAME_BUFFER_DEFAULT] = { 0, 0, 0, 0, 0x00, 0x00, 0, 0x00, 0x00};
  uint8_t READ_frame_length = READ_FRAME_BUFFER_DEFAULT;
  //const uint16_t DWIN_Slider = 0x5103;
  //REQ_frame[4] = DWIN_Slider;
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
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
  HAL_TIM_Base_Start_IT(&htim3);
  KeyPad_Init();
  
  HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 100);  
  HAL_Delay(1000);

  printf("Hello, %s!\n", "KeyPad");
  HAL_Delay(100);
  printf("For messages we using, %s\n", "USART3");
  HAL_Delay(2000);
  DWIN_Menu_Logic_InitProcedure();

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

  /* creation of myLamp */
  myLampHandle = osThreadNew(StartLamp, NULL, &myLamp_attributes);

  /* creation of myBtnScan */
  myBtnScanHandle = osThreadNew(StartBtnScan, NULL, &myBtnScan_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_4);
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1679;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1679;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart3.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|Coil_0_Pin|Coil_1_Pin|Coil_2_Pin
                          |Coil_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Coil_0_Pin Coil_1_Pin Coil_2_Pin Coil_3_Pin */
  GPIO_InitStruct.Pin = Coil_0_Pin|Coil_1_Pin|Coil_2_Pin|Coil_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Row_A_Pin Row_B_Pin Row_C_Pin Row_D_Pin
                           Row_E_Pin Row_F_Pin */
  GPIO_InitStruct.Pin = Row_A_Pin|Row_B_Pin|Row_C_Pin|Row_D_Pin
                          |Row_E_Pin|Row_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void DeCodePushedRow (uint8_t PushedRow, uint8_t Col){
  if (CountKeys[PushedRow][Col] < KeyPushedErrorTicksCount){
    // We desided that this key is pushed enough
//    printf("Pushed Pin on Col:%u, Row:%u, Value:%c\n", Col, PushedRow, ScanKeys[PushedRow][Col]);
    CountKeys[PushedRow][Col]++;
  };
}

void DeCodeReleasedRow (uint8_t ReleasedRow, uint8_t Col){
  if (CountKeys[ReleasedRow][Col] >= KeyPushedErrorTicksCount){
    Key_Pushed = ScanKeys[ReleasedRow][Col];
    Key_Obtained = false;
    switch (currentLogLevel)
    {
    case PROGRAM_LOG_LEVEL_MED:
      printf("Pushed '%c'.\n", ScanKeys[ReleasedRow][Col]);
      osDelay(20);
      break;
    case PROGRAM_LOG_LEVEL_MAX:
      printf("Pushed '%c' on Col:%u, Row:%u.\n", ScanKeys[ReleasedRow][Col], Col, ReleasedRow);
      osDelay(70);
      break;
    default:
      break;
    }
  };
  CountKeys[ReleasedRow][Col]=0;
}

void ReadRows (uint8_t ColIndex){

  // Read ALL Rows and check for pushed or relased condition
  if( HAL_GPIO_ReadPin(ROWS_PORT, Row_A_Pin) == GPIO_PIN_SET ){
    DeCodePushedRow(0,ColIndex);
  } else DeCodeReleasedRow(0,ColIndex);
  if( HAL_GPIO_ReadPin(ROWS_PORT, Row_B_Pin) == GPIO_PIN_SET ){
    DeCodePushedRow(1,ColIndex);
  } else DeCodeReleasedRow(1,ColIndex);
  if( HAL_GPIO_ReadPin(ROWS_PORT, Row_C_Pin) == GPIO_PIN_SET ){
    DeCodePushedRow(2,ColIndex);
  } else DeCodeReleasedRow(2,ColIndex);
  if( HAL_GPIO_ReadPin(ROWS_PORT, Row_D_Pin) == GPIO_PIN_SET ){
    DeCodePushedRow(3,ColIndex);
  } else DeCodeReleasedRow(3,ColIndex);
  if( HAL_GPIO_ReadPin(ROWS_PORT, Row_E_Pin) == GPIO_PIN_SET ){
    DeCodePushedRow(4,ColIndex);
  } else DeCodeReleasedRow(4,ColIndex);
  if( HAL_GPIO_ReadPin(ROWS_PORT, Row_F_Pin) == GPIO_PIN_SET ){
    DeCodePushedRow(5,ColIndex);
  } else DeCodeReleasedRow(5,ColIndex);
}

void TimerCallScan (void){

  // GPIO_PinState  ReadedState = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(COLS_PORT, Coil_0_Pin|Coil_1_Pin|Coil_2_Pin|Coil_3_Pin, GPIO_PIN_RESET); // Pins reset
  switch (counterCOLS % COLS_PINS)
  {
    case 0:
      HAL_GPIO_WritePin(COLS_PORT, Coil_0_Pin, GPIO_PIN_SET);
      ReadRows(0);
      break;
    case 1:
      HAL_GPIO_WritePin(COLS_PORT, Coil_1_Pin, GPIO_PIN_SET);
      ReadRows(1);
      break;
    case 2:
      HAL_GPIO_WritePin(COLS_PORT, Coil_2_Pin, GPIO_PIN_SET);
      ReadRows(2);
      break;
    case 3:
      HAL_GPIO_WritePin(COLS_PORT, Coil_3_Pin, GPIO_PIN_SET);
      ReadRows(3);
      break;
    default:
      if (currentLogLevel!=PROGRAM_LOG_LEVEL_NO){
        printf("Wrong Counter pins! \n");
        osDelay(50);
      } 
  };
  counterCOLS++;  
}

void KeyPad_Init(void){
  uint8_t i,j;
  //                                   Array must be zero inited
  for ( i=0 ; i <= ROWS ; i++  ){
      for ( j=0 ; j <= COLS ; j++ ){
      CountKeys[i][j]=0; 
    }
  }
}

// The following makes printf() write to USART3:
#define STDOUT_FILENO   1
#define STDERR_FILENO   2

int _write(int file, uint8_t *ptr, int len)
{
  switch (file)
  {
    case STDOUT_FILENO:
      HAL_UART_Transmit(&huart3, ptr, len, HAL_MAX_DELAY);
      break;

    case STDERR_FILENO:
      HAL_UART_Transmit(&huart3, ptr, len, HAL_MAX_DELAY);
      break;

    default:
      return -1;
  }

  return len;
}

// DWIN functions section

// Change Page 
void DWIN_setPage(uint8_t page){
    //5A A5 07 82 00 84 5a 01 00 02
    uint8_t sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x07, CMD_WRITE, 0x00, 0x84, 0x5A, 0x01, 0x00, page};
    HAL_UART_Transmit(&huart5, (uint8_t*)sendBuffer, sizeof(sendBuffer), CMD_SEND_TIMEOUT); 
    //readDWIN();
} 

// Get Current Page ID
uint8_t DWIN_getPage(){
    uint8_t sendBuffer[] = {CMD_HEAD1, CMD_HEAD2, 0x04, CMD_READ, 0x00 , 0x14, 0x01};
    HAL_UART_Transmit(&huart5, (uint8_t*)sendBuffer, sizeof(sendBuffer), CMD_SEND_TIMEOUT); 
    //return readCMDLastByte();
    return 0;
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
  uint8_t tmpSelectedMenuItemNumber;
  /* Infinite loop */
  for(;;)
  {
    if (!Key_Obtained){
      switch (Key_Pushed)
      {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
          tmpSelectedMenuItemNumber = (uint8_t)(Key_Pushed-'0');
          //printf("main: Try switch to %d page\n", tmpSelectedMenuItemNumber);
          MenuItemSwitch((MenuItem)tmpSelectedMenuItemNumber);
          osDelay(50);
          ///DWIN_setPage(tmpSelectedMenuItemNumber);
        break;
      default:
          printf("No key trap\n");
          osDelay(20);
        break;
      } 
      Key_Obtained = true;
    };
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLamp */
/**
* @brief Function implementing the myLamp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLamp */
void StartLamp(void *argument)
{
  /* USER CODE BEGIN StartLamp */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    osDelay(LED_CLK_TICKS);
  }
  /* USER CODE END StartLamp */
}

/* USER CODE BEGIN Header_StartBtnScan */
/**
* @brief Function implementing the myBtnScan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBtnScan */
void StartBtnScan(void *argument)
{
  /* USER CODE BEGIN StartBtnScan */
  /* Infinite loop */
  for(;;)
  {
    TimerCallScan();
    osDelay(10);
  }
  /* USER CODE END StartBtnScan */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
    if(htim->Instance == TIM3) //check if the interrupt comes from TIM3
  {
 //   HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
//    TimerCallScan ();
  };
  if(htim->Instance == TIM5) //check if the interrupt comes from TIM5
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
  //  TimerCallScan ();
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
