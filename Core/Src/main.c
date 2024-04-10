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
#include <stdbool.h> // bool, true, false

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

// for DWIN
#define REQUEST_FRAME_BUFFER 7
#define WRITE_FRAME_BUFFER 8
#define READ_FRAME_BUFFER_DEFAULT 9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void KeyPad_Init(void);
void TimerCallScan (void);
void ReadRows (uint8_t ColIndex);
void DeCodePushedRow (uint8_t PushedRow, uint8_t Col);
void DeCodeReleasedRow (uint8_t ReleasedRow, uint8_t Col);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM3) //check if the interrupt comes from TIM3
  {
 //   HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    TimerCallScan ();
  };
  if(htim->Instance == TIM5) //check if the interrupt comes from TIM5
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
  //  TimerCallScan ();
  }
}
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //HAL_UART_Transmit(&huart3, (uint8_t*)"Who's where?\n", 13, 1000);
    // Send command to DWIN
    /*for(int i=0; i<=READ_frame_length; i++){
      READ_frame[i]=0; // Поэлементная очистка буффера чтения
    };
    HAL_UART_Transmit_IT(&huart5, (uint8_t*)REQ_frame, REQ_frame_length);
    while( HAL_UART_GetState (&huart5) == HAL_UART_STATE_BUSY_TX );
    HAL_UART_Receive(&huart5, (uint8_t*)READ_frame, READ_frame_length,1000);
    HAL_Delay(500);
    printf ("May be WE readed, %u.\n",READ_frame[0]);*/
    if (!Key_Obtained){
      switch (Key_Pushed)
      {
      case 'a':
          frame8[5]=0x01; // пишем в 5001
          if (Mode_Measuring_AUTO){
            frame8[7]=0x01; // пишем 1
            Mode_Measuring_AUTO = false;
          }else{
            frame8[7]=0x02; // пишем 2
            Mode_Measuring_AUTO = true;
          }
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
          frame8[5]=0x03; // пишем в 5003
          frame8[7]=0x00; // число  0
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
          frame8[5]=0x04; // пишем в 5004
          frame8[7]=0x00; // число  0
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
        break;
      case 'c':
          Mode_Measuring_AUTO = true;
          frame8[5]=0x01; // пишем в 5001
          frame8[7]=0x00; // число  0
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
          frame8[5]=0x03; // пишем в 5003
          frame8[7]=0x02; // число  1
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
          frame8[5]=0x04; // пишем в 5004
          frame8[7]=0x00; // число  0
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
        break;
      case 'd':
          Mode_Measuring_AUTO = true;
          frame8[5]=0x01; // пишем в 5001
          frame8[7]=0x00; // число  0
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
          frame8[5]=0x03; // пишем в 5003
          frame8[7]=0x00; // число  1
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
          frame8[5]=0x04; // пишем в 5004
          frame8[7]=0x01; // число  0
          HAL_UART_Transmit(&huart5, (uint8_t*)frame8, frame8_length, 50);  
        break;
      default:
        break;
      } 
      Key_Obtained = true;
    };
//    HAL_Delay(1000);
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
  htim3.Init.Period = 500;
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
  htim5.Init.Period = 50000;
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
    printf("Pushed '%c' (Col:%u, Row:%u)\n", ScanKeys[ReleasedRow][Col], Col+1, ReleasedRow+1);
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
      printf("Wrong Counter pins! \n");
      HAL_Delay (500);
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
