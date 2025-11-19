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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "lcd16x2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//---------------------------------------------------------Button and LED pairs
typedef struct {
    uint16_t buttonPin;          // Button GPIO Pin
    GPIO_TypeDef* buttonPort;    // Button GPIO Port
    uint16_t ledPin;             // LED GPIO Pin
    GPIO_TypeDef* ledPort;       // LED GPIO Port
    uint8_t State;               // LED State
    uint32_t blinkDuration;      // Blink duration in terms of the number of timer ticks (e.g., 50ms)
} ButtonLedPair;

typedef enum {
    CALIBRATION_STATE_IDLE,
    CALIBRATION_STATE_EN_START,
    CALIBRATION_STATE_EN_WAIT,
    CALIBRATION_STATE_EN_STOP,
    CALIBRATION_STATE_SP_START,
    CALIBRATION_STATE_SP_MEASURE,
    CALIBRATION_STATE_COMPLETE
} CalibrationState;

typedef struct {
    CalibrationState state;
    uint32_t timestamp;
    uint32_t CalibrationCounter;
    uint32_t readIntervalCounter;
} CalibrationContext;

typedef enum {
    STATE_IDLE,
    STATE_SET_HOURS,
    STATE_SET_MINUTES,
    STATE_SET_SECONDS,
    STATE_SET_DAY,
    STATE_SET_MONTH,
    STATE_SET_YEAR,
    STATE_COMPLETED
} RTC_SetState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Calibration Context
CalibrationContext calibrationContext = {CALIBRATION_STATE_IDLE, 0, 0, 0};

// RTC State and Data
RTC_SetState rtcState = STATE_IDLE;
RTC_TimeTypeDef rtcTime = {0};
RTC_DateTypeDef rtcDate = {0};

// UART-related variables
uint8_t rxDataByte[1];                 // Buffer for receiving one byte of data via UART
char stdnum[13] = "&_24660051_*\n";    // Standard number string to transmit via UART
#define UART_BUFFER_SIZE 12            // Buffer size for the command
char uartBuffer[UART_BUFFER_SIZE];     // Buffer to store received characters
int bufferIndex = 0;                   // Index for the next character in the buffer

// Timer-related variables
#define DEBOUNCE_DELAY 40              // Debounce delay in ms
#define EN_MEASURE_TIME 1000           // 1 second for environment measurement
#define SP_MEASURE_TIME 6000           // 8 seconds total for the duty cycle sweep

// Global variable to track button press
volatile uint32_t buttonPressed = 0;

// Environment Sensing-related variables
volatile uint32_t E_measuring = 0;     // Flag for measuring state: 0 - Not measuring, 1 - Measuring
volatile uint32_t P_measuring = 0;     // Flag for Power measurement
volatile uint32_t calibration = 0;     // Flag for Calibration
volatile uint8_t settingTime = 0;      // Flag for setting RTC time

// Pulse counting and temperature variables for digital temperature measurement
volatile uint32_t last_pulse_train_start = 0; // Timestamp for the start of the last pulse train
volatile uint32_t pulse_count = 0;            // Count of pulses in the current pulse train
volatile uint32_t total_pulse_count = 0;      // Total pulses counted across all trains
volatile uint32_t pulse_train_count = 0;      // Count of pulse trains
volatile float Amb_temp = 0.0f;               // Ambient temperature
volatile float Pann_temp = 0.0f;              // Panel temperature
volatile float lightIntensity = 0.0f;         // Light intensity

// Solar Panel Data
volatile int Vpv = 0.0f;                      // PV voltage
volatile int Ipv = 0.0f;                      // PV current
volatile int Ppv = 0.0f;                      // PV power
volatile int Eff = 0.0f;                      // Efficiency
volatile int Vmpp = 0.0f;                     // Maximum power point voltage
volatile int Impp = 0.0f;                     // Maximum power point current
volatile int Pmpp = 0.0f;                     // Maximum power point power
volatile int Pmpp_calibrated = 0;             // Calibrated MPP power
volatile int Vmpp_calibrated = 0;             // Calibrated MPP voltage
volatile int Impp_calibrated = 0;             // Calibrated MPP current
volatile int currentDutyCycle = 0;            // Current PWM duty cycle
volatile float maxPower = 0.0f;               // Maximum power
volatile float Lux_calibrated= 0.0f;          // Calibrated light intensity

// Button-LED configurations
ButtonLedPair buttonLedPairs[] = {
    {GPIO_PIN_9, GPIOB, GPIO_PIN_4, GPIOB, 0, 50}, // Pair 0: Top Button on GPIO_PIN_9, LED on GPIO_PIN_4, with 50ms blink duration
    {GPIO_PIN_5, GPIOA, GPIO_PIN_10, GPIOB, 0, 100}, // Pair 1: Bottom Button & LED D2
    {GPIO_PIN_8, GPIOB, GPIO_PIN_10, GPIOA, 0, 500}, // Pair 2: Left Button
    {GPIO_PIN_6, GPIOB, GPIO_PIN_5, GPIOB, 0, 200}, // Pair 3: Right Button
    // Additional pairs can be added here
};

// Global LCD handle
volatile uint32_t displayMode = 1;  // Flag to track display mode, 'volatile' if updated in an interrupt

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // Callback function for timer period elapsed interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); // Callback function for UART receive complete interrupt

// UART Communication Functions
void EN_Command(const char *commandBuffer); // Handles the "&_EN_*" UART command
void SP_Command(const char *commandBuffer); // Handles the "&_SP_*" UART command
void CA_Command(const char *commandBuffer); // Handles the "&_CA_*" UART command
void PowerConditions(int Vmpp, int Impp, int Pmpp, int Eff); // Formats and sends power conditions data over UART
void EnvironmentConditions(float Amb_temp, float Pann_temp, float lightintensity); // Formats and sends environmental conditions data over UART

// PV unit Data Functions
void ReadSolarPanelData(void); // Reads solar panel data
void ProcessCalibration(void); // Processes the calibration procedure
int CalculateEfficiency(); // Calculates the efficiency of the solar panel

// Environment Sensing Functions
float LightSensing(); // Reads and converts ADC value to light intensity
float AnalogTemperatureSensing(); // Reads and converts ADC value to ambient temperature
float DigitalTemperatureSensing(); // Handles temperature measurement and processing for digital sensor

// RTC Setting Functions
void decrementRTCSetting(RTC_SetState state); // Decrements the RTC setting based on the current state
void incrementRTCSetting(RTC_SetState state); // Increments the RTC setting based on the current state
void updateRTCDisplay(RTC_SetState state); // Updates the RTC display based on the current state
void completeRTCSetting(void); // Completes the RTC setting process and applies changes

// LCD Functions
void setup(void); // Initial setup function
void updateDisplay(uint32_t displayMode, float Amb_temp, float Pann_temp, float lightIntensity, int Vmpp, int Impp, int Pmpp, int Eff); // Updates the display based on the current mode and data
void lcdLiveMeasuring(int Vpv, int Ipv, int Ppv, int Eff); // Updates the LCD with live measuring data

// LED Control Functions
void toggleLedBlinking(ButtonLedPair *pair); // Toggles LED blinking based on button press
void ControlLED4BasedOnEfficiency(int Eff); // Controls LED4 based on efficiency value

// PWM Control Functions
void SetPWM_DutyCycle(uint32_t dutyCyclePercentage); // Sets the PWM duty cycle

// ADC Channel Selection Functions
void ADC_Select_CH1(void); // Selects ADC Channel 1
void ADC_Select_CH4(void); // Selects ADC Channel 4
void ADC_Select_CH14(void); // Selects ADC Channel 14
void ADC_Select_CH15(void); // Selects ADC Channel 15


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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  setup();
  updateDisplay(displayMode,Amb_temp, Pann_temp, lightIntensity,Vmpp,Impp,Pmpp,Eff);
  /* Send the standard number message only once after setup */
  HAL_UART_Transmit(&huart2, (uint8_t*) stdnum, 13, 200);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  SetPWM_DutyCycle(0);
  /* Re-enable UART receive interrupt */
  HAL_UART_Receive_IT(&huart2, rxDataByte, 1);
  HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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

//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_14;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_15;
//  sConfig.Rank = 2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = 3;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = 4;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 31999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1599;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim5, TIM_CHANNEL_1);
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_ODD;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D4_Pin|D5_Pin|LED1_Pin|D6_Pin
                          |RS_Pin|D7_Pin|E_Pin|LED2_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B_Button_Pin M_Button_Pin */
  GPIO_InitStruct.Pin = B_Button_Pin|M_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin LED1_Pin D6_Pin
                           RS_Pin D7_Pin E_Pin LED2_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|LED1_Pin|D6_Pin
                          |RS_Pin|D7_Pin|E_Pin|LED2_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED4_Pin */
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Digital_temp_Pin */
  GPIO_InitStruct.Pin = Digital_temp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Digital_temp_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Button_Pin L_Button_Pin T_Button_Pin */
  GPIO_InitStruct.Pin = R_Button_Pin|L_Button_Pin|T_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* Callback function for timer period elapsed interrupt */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /**
     * Static counters for LED blinking, display updates, and tick counter
     */
    static uint32_t blinkCounters[sizeof(buttonLedPairs) / sizeof(buttonLedPairs[0])] = {0};
    static uint32_t displayCounters = 0;
    static uint32_t tickCounter = 0;

    /**
     * Handle calibration process if TIM1 interrupt is triggered and calibration is enabled
     */
    if (htim->Instance == TIM1 && calibration) {
        ProcessCalibration();
    }

    /**
     * Handle LED blinking if TIM2 interrupt is triggered
     */
    if (htim->Instance == TIM2) {
        // Iterate through each button-LED pair
        for (int i = 0; i < sizeof(buttonLedPairs) / sizeof(buttonLedPairs[0]); i++) {
            if (buttonLedPairs[i].State == 1) {
                // Increment blink counter for each LED
                blinkCounters[i]++;
                if (blinkCounters[i] >= buttonLedPairs[i].blinkDuration) {
                    // Toggle LED state and reset blink counter
                    HAL_GPIO_TogglePin(buttonLedPairs[i].ledPort, buttonLedPairs[i].ledPin);
                    blinkCounters[i] = 0;
                }
            } else {
                // Reset blink counter if LED is not enabled
                blinkCounters[i] = 0;
            }
        }
    }

    /**
     * Handle PWM duty cycle updates and solar panel data measurement if TIM3 interrupt is triggered
     */
    if (htim->Instance == TIM3 && P_measuring) {
        // Increment tick counter
        tickCounter++;

        // Check if it's time to increment the duty cycle
        if (tickCounter >= 140) {
            tickCounter = 0;  // Reset the tick counter
            if (currentDutyCycle < 100) {
                currentDutyCycle++;  // Increment the duty cycle
            } else {
                currentDutyCycle = 0;  // Reset the duty cycle after reaching 100%
            }
            SetPWM_DutyCycle(currentDutyCycle);  // Update the PWM duty cycle
        }

        // Measure solar panel data and update display if enabled
        if (P_measuring && displayMode == 1) {
            ReadSolarPanelData();
            displayCounters++;
            if (displayCounters >= 1000) {
                Eff = CalculateEfficiency();
                lcdLiveMeasuring(Vpv, Ipv, Ppv, Eff);
                displayCounters = 0;
            }
        }
    }

    /**
     * Handle RTC time and date updates if TIM4 interrupt is triggered
     */
    if (htim->Instance == TIM4) {
        if (displayMode == 3) {
            RTC_TimeTypeDef sTime;
            RTC_DateTypeDef sDate;

            // Get the current time and date from the RTC
            HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
            HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

            // Update the display
            lcd16x2_1stLine();
            lcd16x2_printf("%02d/%02d/%04d          ", sDate.Date, sDate.Month, 2000 + sDate.Year);
            lcd16x2_2ndLine();
            lcd16x2_printf("%02d:%02d:%02d        ", sTime.Hours, sTime.Minutes, sTime.Seconds);
        }
        ControlLED4BasedOnEfficiency(Eff);
    }
}

/* Callback function for GPIO external interrupt */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t lastTimeButtonPressed = 0;  // Timestamp of the last button press
    uint32_t currentTime;

    if (GPIO_Pin == GPIO_PIN_7) {  // Middle button (RTC setting)
        currentTime = HAL_GetTick();
        if ((currentTime - lastTimeButtonPressed) > 50) {  // Debounce check
            lastTimeButtonPressed = currentTime;
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {  // Active low check
                if (!settingTime) {
                    settingTime = 1;
                    rtcState = STATE_SET_DAY;  // Start RTC setting mode
                } else {
                    // Cycle through RTC setting states
                    switch (rtcState) {
                        case STATE_SET_DAY:
                            rtcState = STATE_SET_MONTH;
                            break;
                        case STATE_SET_MONTH:
                            rtcState = STATE_SET_YEAR;
                            break;
                        case STATE_SET_YEAR:
                            rtcState = STATE_SET_HOURS;
                            break;
                        case STATE_SET_HOURS:
                            rtcState = STATE_SET_MINUTES;
                            break;
                        case STATE_SET_MINUTES:
                            rtcState = STATE_SET_SECONDS;
                            break;
                        case STATE_SET_SECONDS:
                            rtcState = STATE_COMPLETED;
                            completeRTCSetting();  // Apply the RTC settings
                            settingTime = 0;  // Exit setting mode
                            rtcState = STATE_IDLE;  // Reset state to idle
                            break;
                        default:
                            break;
                    }
                }
                updateRTCDisplay(rtcState);  // Update the display with the new state
            }
        }
    }

    if (GPIO_Pin == GPIO_PIN_8) {  // Left button (Display mode switch)
        currentTime = HAL_GetTick();
        if ((currentTime - lastTimeButtonPressed) > 150) {  // Debounce check
            lastTimeButtonPressed = currentTime;
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET) {  // Active low check
                displayMode++;
                if (displayMode > 3) {
                    displayMode = 1;  // Wrap around if mode exceeds 3
                }
                updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);
            }
        }
    }

    if (GPIO_Pin == GPIO_PIN_6) {  // Right button (Calibration)
        currentTime = HAL_GetTick();
        if ((currentTime - lastTimeButtonPressed) > 100) {  // Debounce check
            lastTimeButtonPressed = currentTime;
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET) {  // Active low check
                calibration = !calibration;
                if (calibration) {
                    CA_Command("&_CA_*\n");  // Start calibration
                }
            }
        }
    }

    if (GPIO_Pin == GPIO_PIN_9) {  // Top button (Environment measurement)
        currentTime = HAL_GetTick();
        if ((currentTime - lastTimeButtonPressed) > 75) {  // Debounce check
            lastTimeButtonPressed = currentTime;
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {  // Active low check
                if (settingTime) {
                    incrementRTCSetting(rtcState);  // Increment RTC setting
                    updateRTCDisplay(rtcState);  // Update the display
                } else {
                    E_measuring = !E_measuring;
                    displayMode = 2;  // Switch to environment display mode
                    updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);
                    toggleLedBlinking(&buttonLedPairs[0]);  // Toggle LED for environment measurement
                    if (!E_measuring) {
                        lightIntensity = LightSensing();
                        Amb_temp = AnalogTemperatureSensing();
                        Pann_temp = DigitalTemperatureSensing();
                        EnvironmentConditions(Amb_temp, Pann_temp, lightIntensity);
                        updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);
                    }
                }
            }
        }
    }

    if (GPIO_Pin == GPIO_PIN_5) {  // Bottom button (Power measurement)
        currentTime = HAL_GetTick();
        if ((currentTime - lastTimeButtonPressed) > 25) {  // Debounce check
            lastTimeButtonPressed = currentTime;
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) {  // Active low check
                if (settingTime) {
                    decrementRTCSetting(rtcState);  // Decrement RTC setting
                    updateRTCDisplay(rtcState);  // Update the display
                } else {
                    maxPower = 0;
                    P_measuring = !P_measuring;
                    displayMode = 1;  // Switch to power display mode
                    if (P_measuring) {
                        HAL_TIM_Base_Start_IT(&htim3);  // Start Timer 3
                    } else {
                        HAL_TIM_Base_Stop_IT(&htim3);  // Stop Timer 3
                        Eff = CalculateEfficiency();  // Calculate efficiency
                        updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);
                        PowerConditions(Vmpp, Impp, Pmpp, Eff);  // Update power conditions
                        SetPWM_DutyCycle(0);  // Reset PWM duty cycle
                    }
                    toggleLedBlinking(&buttonLedPairs[1]);  // Toggle LED for power measurement
                }
            }
        }
    }

    if (GPIO_Pin == Digital_temp_Pin) {  // Digital temperature sensor pulse counting
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_pulse_train_start) > 50) {
            last_pulse_train_start = current_time;
            if (E_measuring) {
                total_pulse_count += pulse_count;
                pulse_train_count++;
            }
            pulse_count = 0;
        }
        pulse_count++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    UNUSED(huart);

    // Append received character to the buffer and ensure null-termination
    if (bufferIndex < UART_BUFFER_SIZE - 1) {
        uartBuffer[bufferIndex++] = rxDataByte[0];
        uartBuffer[bufferIndex] = '\0';  // Null-terminate the string
    }

    // Check if the command is complete (ends with '\n')
    if (rxDataByte[0] == '\n') {
        // Process commands based on the received buffer content
        if (strcmp(uartBuffer, "&_EN_*\n") == 0) {
            EN_Command(uartBuffer);
        } else if (strcmp(uartBuffer, "&_SP_*\n") == 0) {
            SP_Command(uartBuffer);
        } else if (strcmp(uartBuffer, "&_CA_*\n") == 0) {
            CA_Command(uartBuffer);
        }

        // Reset the buffer for the next command
        memset(uartBuffer, 0, UART_BUFFER_SIZE);
        bufferIndex = 0;
    }

    // Buffer is full, reset the buffer
    if (bufferIndex == UART_BUFFER_SIZE - 1) {
        memset(uartBuffer, 0, UART_BUFFER_SIZE);
        bufferIndex = 0;
    }

    // Re-enable UART receive interrupt for the next byte
    HAL_UART_Receive_IT(&huart2, rxDataByte, 1);
}

/* Handles the "&_EN_*" UART command to start or stop environment measurement */
void EN_Command(const char *commandBuffer) {
    if (strcmp(commandBuffer, "&_EN_*\n") == 0) {  // Check for the "&_EN_*" command
        E_measuring = !E_measuring;  // Toggle environment measuring state
        displayMode = 2;  // Set display mode to environment mode
        updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);  // Update display

        toggleLedBlinking(&buttonLedPairs[0]);  // Toggle the LED for environment measurement

        if (!E_measuring) {  // If stopping the measurement
            lightIntensity = LightSensing();
            Amb_temp = AnalogTemperatureSensing();
            Pann_temp = DigitalTemperatureSensing();
            EnvironmentConditions(Amb_temp, Pann_temp, lightIntensity);  // Send environment data over UART
            updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);  // Update display again
        }
    }
}

/* Handles the "&_SP_*" UART command to start or stop power measurement */
void SP_Command(const char *commandBuffer) {
    if (strcmp(commandBuffer, "&_SP_*\n") == 0) {  // Check for the "&_SP_*" command
        P_measuring = !P_measuring;  // Toggle power measuring state
        displayMode = 1;  // Set display mode to power mode

        if (P_measuring) {
            HAL_TIM_Base_Start_IT(&htim3);  // Start Timer 3 for power measurement
        } else {
            HAL_TIM_Base_Stop_IT(&htim3);  // Stop Timer 3
            updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);  // Update display
            PowerConditions(Vmpp, Impp, Pmpp, Eff);  // Send power conditions over UART
        }
        toggleLedBlinking(&buttonLedPairs[1]);  // Toggle the LED for power measurement
    }
}

/* Handles the "&_CA_*" UART command to start the calibration process */
void CA_Command(const char *commandBuffer) {
    if (strcmp(commandBuffer, "&_CA_*\n") == 0) {  // Check for the "&_CA_*" command
        calibrationContext.state = CALIBRATION_STATE_EN_START;  // Start calibration state
        calibrationContext.timestamp = HAL_GetTick();  // Record the start time
        HAL_TIM_Base_Start_IT(&htim1);  // Start Timer 1
        toggleLedBlinking(&buttonLedPairs[3]);  // Start blinking LED pair 3
        toggleLedBlinking(&buttonLedPairs[0]);  // Start blinking LED pair 0
        ProcessCalibration();  // Start the calibration process
    }
}

/* Formats and sends power conditions data over UART */
void PowerConditions(int Vmpp, int Impp, int Pmpp, int Eff) {
    char msg[100];

    // Format the message according to the provided example
    sprintf(msg, "&_%04d_%03d_%03d_%03d_*\n", Vmpp, Impp, Pmpp, Eff);

    // Send the message over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/* Formats and sends environmental conditions data over UART */
void EnvironmentConditions(float Amb_temp, float Pann_temp, float lightintensity) {
    char msg[100];
    int A_temp_int = (int)Amb_temp;  // Integer part of Ambient temperature
    int P_temp_int = (int)Pann_temp;  // Integer part of Panel temperature
    int lightintensity_int = (int)(lightintensity * 10000);  // Adjust multiplier if needed to get desired precision

    // Ensure only 3 digits for temperatures
    A_temp_int = A_temp_int % 1000;
    P_temp_int = P_temp_int % 1000;

    // Convert light intensity to a 5-digit string with leading zeros
    char light_str[6];
    sprintf(light_str, "%05d", lightintensity_int);  // Left pads with zeros to make 5 digits

    // Format the message as specified
    sprintf(msg, "&_%03d_%03d_%s_*\n", A_temp_int, P_temp_int, light_str);

    // Send the message over UART
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

/* Reads data from the solar panel using ADC channels and calculates voltage, current, and power */
void ReadSolarPanelData(void) {
    float voltage, current, power;

    // Start ADC Sol1 conversion (Channel 1)
    ADC_Select_CH1();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t adcValue_C1 = HAL_ADC_GetValue(&hadc1);  // Voltage at R4-R5 divider
    HAL_ADC_Stop(&hadc1);  // Stop ADC after the last measurement

    // Start ADC Sol2 conversion (Channel 4)
    ADC_Select_CH4();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t adcValue_C2 = HAL_ADC_GetValue(&hadc1);  // Voltage at R2-R3 divider
    HAL_ADC_Stop(&hadc1);  // Stop ADC after the last measurement

    // Calculate voltages based on ADC readings
    float voltageAcrossR4_R5 = ((float)adcValue_C1 / 4095.0) * 3300.0;
    float voltageAcrossR2_R3 = ((float)adcValue_C2 / 4095.0) * 3300.0;

    float Voltage4 = voltageAcrossR4_R5 * ((12.0 + 18.0) / 18.0);
    float Voltage2 = voltageAcrossR2_R3 * ((12.0 + 18.0) / 18.0);

    // Calculate voltage, current, and power if Voltage4 is greater than Voltage2
    if (Voltage4 > Voltage2) {
        voltage = Voltage4 - Voltage2;
        current = voltage / 6.8;
        power = Voltage4 * current;

        // Convert for display and storage
        int tempPpv = (int)(power / 1000);  // Convert watts to milliwatts
        int tempVpv = (int)(Voltage4);  // Convert volts to millivolts
        int tempIpv = (int)(current);  // Convert amperes to milliamperes

        // Record maximum power
        if (P_measuring && tempPpv > maxPower) {
            maxPower = tempPpv;
            Pmpp = maxPower;
            Vmpp = tempVpv;
            Impp = tempIpv;
        }
    }

    // Store the latest readings
    Vpv = (int)(Voltage4);  // Voltage in millivolts
    Ipv = (int)(current);  // Current in milliamperes
    Ppv = (int)(power);  // Power in milliwatts
    Eff = 0;  // Placeholder for efficiency calculation
}

/* Processes the calibration procedure */
void ProcessCalibration(void) {
    switch (calibrationContext.state) {
        case CALIBRATION_STATE_EN_START:
            // Start environmental measurement and update display
            E_measuring = 1;
            displayMode = 2;  // Set display to environmental data mode
            updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp, Impp, Pmpp, Eff);
            calibrationContext.state = CALIBRATION_STATE_EN_WAIT;
            calibrationContext.timestamp = HAL_GetTick();
            break;

        case CALIBRATION_STATE_EN_WAIT:
            // Wait for 3 seconds
            if (HAL_GetTick() - calibrationContext.timestamp >= 3000) {
                calibrationContext.state = CALIBRATION_STATE_EN_STOP;
            }
            break;

        case CALIBRATION_STATE_EN_STOP:
            // Stop environmental measurement, record data, and toggle LEDs
            E_measuring = 0;
            Lux_calibrated = LightSensing();
            Amb_temp = AnalogTemperatureSensing();
            Pann_temp = DigitalTemperatureSensing();
            EnvironmentConditions(Amb_temp, Pann_temp, Lux_calibrated);
            updateDisplay(displayMode, Amb_temp, Pann_temp, Lux_calibrated, Vmpp, Impp, Pmpp, Eff);
            toggleLedBlinking(&buttonLedPairs[0]);  // Stop blinking LED pair 0
            toggleLedBlinking(&buttonLedPairs[1]);  // Start blinking LED pair 1
            calibrationContext.state = CALIBRATION_STATE_SP_START;
            break;

        case CALIBRATION_STATE_SP_START:
            // Start the PWM sweep for solar panel measurements
            P_measuring = 1;
            displayMode = 1;  // Set display to solar panel data mode
            HAL_TIM_Base_Start_IT(&htim3);
            calibrationContext.state = CALIBRATION_STATE_SP_MEASURE;
            calibrationContext.CalibrationCounter = 0;
            calibrationContext.readIntervalCounter = 0;
            break;

        case CALIBRATION_STATE_SP_MEASURE:
            // Perform normal SP measurement and assign to calibrated variables
            if (P_measuring && displayMode == 1) {
                ReadSolarPanelData();
                Pmpp_calibrated = Pmpp;
                Vmpp_calibrated = Vmpp;
                Impp_calibrated = Impp;
            }
            calibrationContext.CalibrationCounter++;
            if (calibrationContext.CalibrationCounter >= SP_MEASURE_TIME) {
                calibrationContext.state = CALIBRATION_STATE_COMPLETE;
            }
            break;

        case CALIBRATION_STATE_COMPLETE:
            // Complete the calibration process
            HAL_TIM_Base_Stop_IT(&htim1);
            P_measuring = 0;
            toggleLedBlinking(&buttonLedPairs[1]);  // Stop blinking LED pair 1
            toggleLedBlinking(&buttonLedPairs[3]);  // Stop blinking LED pair 3
            calibration = 0;  // End calibration
            SetPWM_DutyCycle(0);  // Reset the duty cycle
            PowerConditions(Vmpp_calibrated, Impp_calibrated, Pmpp_calibrated, Eff);
            displayMode = 1;
            updateDisplay(displayMode, Amb_temp, Pann_temp, lightIntensity, Vmpp_calibrated, Impp_calibrated, Pmpp_calibrated, Eff);
            calibrationContext.CalibrationCounter = 0;  // Reset the counter for future calibrations
            calibrationContext.state = CALIBRATION_STATE_IDLE;
            break;

        case CALIBRATION_STATE_IDLE:
        default:
            // Do nothing in idle state
            break;
    }
}

/* Calculates the efficiency of the solar panel */
int CalculateEfficiency() {
    // Constants
    const float T_STC = 25.0f;  // Standard Testing Conditions temperature in °C
    const float beta = -0.004;  // Temperature coefficient for the PV module

    // Measured values
    float Pmeasured = Pmpp;  // Measured power in mW
    float Tpv_panel = Pann_temp;  // Measured PV panel temperature in °C
    float Lux_measured = lightIntensity;  // Measured LUX value

    // Ensure measured values are reasonable
    if (Lux_measured == 0 || Pmpp_calibrated == 0) {
        return 0;  // Prevent division by zero
    }

    // Normalize the measured power with respect to temperature
    float PnormT = Pmeasured / (1 + beta * (Tpv_panel - T_STC));  // Power normalization for temperature

    // Normalize the measured power with respect to LUX
    float Pnormalised = PnormT * (Lux_calibrated / Lux_measured);  // Power normalization for light intensity

    // Calculate efficiency
    float efficiency = (Pnormalised / Pmpp_calibrated) * 100.0f;

    // Ensure efficiency does not exceed 100%
    if (efficiency > 100.0f) {
        efficiency = 100.0f;
    }

    return (int)efficiency;
}

/* Reads and converts ADC value to light intensity */
float LightSensing() {
    uint32_t adcValue = 0;
    float light = 0.0f;

    // Start ADC conversion for Channel 14
    ADC_Select_CH14();
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
        adcValue = HAL_ADC_GetValue(&hadc1);  // Get ADC value for channel 14
    }
    HAL_ADC_Stop(&hadc1);  // Stop ADC conversion

    // Convert ADC value to light intensity
    light = (((float)adcValue / 4095.0f) * 30);  // Example conversion formula
    return light;
}

/* Reads and converts ADC value to ambient temperature */
float AnalogTemperatureSensing() {
    uint32_t adcValue = 0;

    // Start ADC conversion for Channel 15
    ADC_Select_CH15();
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK) {
        adcValue = HAL_ADC_GetValue(&hadc1);  // Get ADC value for channel 15
    }
    HAL_ADC_Stop(&hadc1);  // Stop ADC conversion

    // Convert ADC value to ambient temperature
    float Amb_temp = (((float)adcValue / 4095.0f) * 330.0f) - 273;
    return Amb_temp;
}

/* Handles digital temperature measurement and processing */
float DigitalTemperatureSensing() {
    float P_temp = 0.0f;
    if (!E_measuring) {  // Check if the measurement has just stopped
        float avg_pulse_per_train = (pulse_train_count > 0) ? (float)total_pulse_count / pulse_train_count : 0;
        P_temp = ((avg_pulse_per_train * 256) / 4096) - 50;  // Adjust formula as needed

        // Reset measurement variables for the next cycle
        total_pulse_count = 0;
        pulse_train_count = 0;
    }
    return P_temp;  // Return the computed temperature
}

/* Decrements the RTC setting based on the current state */
void decrementRTCSetting(RTC_SetState state) {
    switch (state) {
        case STATE_SET_DAY:
            rtcDate.Date = (rtcDate.Date == 1) ? 31 : rtcDate.Date - 1;  // Decrement day, wrap to 31 if at 1
            break;
        case STATE_SET_MONTH:
            rtcDate.Month = (rtcDate.Month == 1) ? 12 : rtcDate.Month - 1;  // Decrement month, wrap to 12 if at 1
            break;
        case STATE_SET_YEAR:
            rtcDate.Year = (rtcDate.Year == 0) ? 99 : rtcDate.Year - 1;  // Decrement year, wrap to 99 if at 0
            break;
        case STATE_SET_HOURS:
            rtcTime.Hours = (rtcTime.Hours == 0) ? 23 : rtcTime.Hours - 1;  // Decrement hours, wrap to 23 if at 0
            break;
        case STATE_SET_MINUTES:
            rtcTime.Minutes = (rtcTime.Minutes == 0) ? 59 : rtcTime.Minutes - 1;  // Decrement minutes, wrap to 59 if at 0
            break;
        case STATE_SET_SECONDS:
            rtcTime.Seconds = (rtcTime.Seconds == 0) ? 59 : rtcTime.Seconds - 1;  // Decrement seconds, wrap to 59 if at 0
            break;
        default:
            break;
    }
}

/* Increments the RTC setting based on the current state */
void incrementRTCSetting(RTC_SetState state) {
    switch (state) {
        case STATE_SET_DAY:
            rtcDate.Date = (rtcDate.Date % 31) + 1;  // Increment day, wrap to 1 if at 31
            break;
        case STATE_SET_MONTH:
            rtcDate.Month = (rtcDate.Month % 12) + 1;  // Increment month, wrap to 1 if at 12
            break;
        case STATE_SET_YEAR:
            rtcDate.Year = (rtcDate.Year + 1) % 100;  // Increment year, wrap to 0 if at 99
            break;
        case STATE_SET_HOURS:
            rtcTime.Hours = (rtcTime.Hours + 1) % 24;  // Increment hours, wrap to 0 if at 23
            break;
        case STATE_SET_MINUTES:
            rtcTime.Minutes = (rtcTime.Minutes + 1) % 60;  // Increment minutes, wrap to 0 if at 59
            break;
        case STATE_SET_SECONDS:
            rtcTime.Seconds = (rtcTime.Seconds + 1) % 60;  // Increment seconds, wrap to 0 if at 59
            break;
        default:
            break;
    }
}

/* Updates the RTC display based on the current setting state */
void updateRTCDisplay(RTC_SetState state) {
    // Print the current date and time
    lcd16x2_setCursor(0, 0);
    lcd16x2_printf("%02d/%02d/%04d         ", rtcDate.Date, rtcDate.Month, 2000 + rtcDate.Year);
    lcd16x2_setCursor(1, 0);
    lcd16x2_printf("%02d:%02d:%02d         ", rtcTime.Hours, rtcTime.Minutes, rtcTime.Seconds);

    // Position and blink cursor based on the current setting state
    switch (state) {
        case STATE_SET_DAY:
            lcd16x2_enable_cursor_blink(0, 1);  // Enable blinking cursor at day
            break;
        case STATE_SET_MONTH:
            lcd16x2_enable_cursor_blink(0, 4);  // Enable blinking cursor at month
            break;
        case STATE_SET_YEAR:
            lcd16x2_enable_cursor_blink(0, 9);  // Enable blinking cursor at year
            break;
        case STATE_SET_HOURS:
            lcd16x2_enable_cursor_blink(1, 1);  // Enable blinking cursor at hours
            break;
        case STATE_SET_MINUTES:
            lcd16x2_enable_cursor_blink(1, 4);  // Enable blinking cursor at minutes
            break;
        case STATE_SET_SECONDS:
            lcd16x2_enable_cursor_blink(1, 7);  // Enable blinking cursor at seconds
            break;
        case STATE_COMPLETED:
            lcd16x2_cursorShow(false);  // Turn off cursor blinking
        case STATE_IDLE:
            // Disable cursor blinking when not setting
            lcd16x2_cursorShow(false);  // This assumes lcd16x2_cursorShow turns off the cursor entirely
            break;
    }
}

/* Completes the RTC setting process and applies changes */
void completeRTCSetting() {
    // Set time and date on the hardware RTC
    HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
    displayMode = 3;  // Set display mode to show RTC data
    HAL_TIM_Base_Start_IT(&htim4);  // Start Timer 4 for periodic RTC updates
}

void setup(void) {
    lcd16x2_init_4bits(RS_GPIO_Port, RS_Pin, E_Pin,
        // D0_GPIO_Port, D0_Pin, D1_Pin, D2_Pin, D3_Pin,
        D4_GPIO_Port, D4_Pin, D5_Pin, D6_Pin, D7_Pin);
}

/* Updates the LCD display based on the current display mode */
void updateDisplay(uint32_t displayMode, float Amb_temp, float Pann_temp, float lightIntensity, int Vmpp, int Impp, int Pmpp, int Eff) {
    if (displayMode == 1) {
        // Display voltage, current, power, and efficiency
        lcd16x2_1stLine();  // Clear the first line of the display
        lcd16x2_printf("V:%4dmV I:%3dmA", Vmpp, Impp);  // Display voltage and current

        lcd16x2_2ndLine();  // Move to the second line
        lcd16x2_printf("P: %3dmW E: %3d%%", Pmpp, Eff);  // Display power and efficiency

    } else if (displayMode == 2) {
        // Display ambient and panel temperatures, and light intensity
        int Ta = (int)Amb_temp;  // Integer part of Ambient temperature
        int Tp = (int)Pann_temp;  // Integer part of Panel temperature
        int lux = (int)(lightIntensity * 10000);  // Convert light intensity to integer

        lcd16x2_1stLine();  // Clear the display
        lcd16x2_printf("AMB:%3dC SP:%3dC", Ta, Tp);  // Display temperatures

        lcd16x2_2ndLine();  // Move to the second line
        lcd16x2_printf("LUX:%5d         ", lux);  // Display light sensor value

    } else if (displayMode == 3) {
        // Display current date and time
        RTC_TimeTypeDef sTime;
        RTC_DateTypeDef sDate;

        // Get the current time and date from the RTC
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        // Display date in DD-MM-YYYY format
        lcd16x2_1stLine();
        lcd16x2_printf("%02d/%02d/%04d          ", sDate.Date, sDate.Month, 2000 + sDate.Year);

        // Display time in HH:MM:SS format
        lcd16x2_2ndLine();
        lcd16x2_printf("%02d:%02d:%02d          ", sTime.Hours, sTime.Minutes, sTime.Seconds);
    }
}

/* Updates the LCD with live measuring data */
void lcdLiveMeasuring(int Vpv, int Ipv, int Ppv, int Eff) {
    // Clear the first line of the display
    lcd16x2_1stLine();
    lcd16x2_printf("V:%4dmV I:%3dmA", Vpv, Ipv);  // Display voltage and current

    // Move to the second line
    lcd16x2_2ndLine();
    lcd16x2_printf("P: %3dmW E: %3d%%", Ppv, Eff);  // Display power and efficiency
}


void toggleLedBlinking(ButtonLedPair *pair) {
    // Calculate the index of the pair for specific behavior if needed
    int index = pair - buttonLedPairs; // This assumes buttonLedPairs is an array

    // Toggle the LED state
    pair->State = !(pair->State);

    // Depending on the pair, handle the LED behavior
    switch (index) {
        case 0:
            // For pair 0, manage the timer and direct control of LED
            if (pair->State == 1) {
                // Start the timer to begin blinking
                HAL_TIM_Base_Start_IT(&htim2);
            } else {
                // Stop the timer and turn off the LED
                HAL_TIM_Base_Stop_IT(&htim2);
                HAL_GPIO_WritePin(pair->ledPort, pair->ledPin, GPIO_PIN_SET); // Ensure LED is off when stopping
            }
            break;
        case 1:
            // For pair 1, let's assume we also want to manage blinking with the timer
            if (pair->State == 1) {
                HAL_TIM_Base_Start_IT(&htim2); // Maybe use a different timer or setting if needed
            } else {
                HAL_TIM_Base_Stop_IT(&htim2);
                HAL_GPIO_WritePin(pair->ledPort, pair->ledPin, GPIO_PIN_SET); // Ensure LED is off when stopping
            }
            break;
        case 2:
            // For pair 1, let's assume we also want to manage blinking with the timer
            if (pair->State == 1) {
                HAL_TIM_Base_Start_IT(&htim2); // Maybe use a different timer or setting if needed
            } else {
                HAL_TIM_Base_Stop_IT(&htim2);
                HAL_GPIO_WritePin(pair->ledPort, pair->ledPin, GPIO_PIN_SET); // Ensure LED is off when stopping
            }
            break;
        case 3:
            // For pair 1, let's assume we also want to manage blinking with the timer
            if (pair->State == 1) {
                HAL_TIM_Base_Start_IT(&htim2); // Maybe use a different timer or setting if needed
            } else {
                HAL_TIM_Base_Stop_IT(&htim2);
                HAL_GPIO_WritePin(pair->ledPort, pair->ledPin, GPIO_PIN_SET); // Ensure LED is off when stopping
            }
            break;
        default:
            // For all other pairs, just toggle the state of the LED without using a timer
            if (pair->State == 1) {
                HAL_GPIO_WritePin(pair->ledPort, pair->ledPin, GPIO_PIN_SET); // Turn LED on
            } else {
                HAL_GPIO_WritePin(pair->ledPort, pair->ledPin, GPIO_PIN_RESET); // Turn LED off
            }
            break;
    }
}

/* Controls LED4 based on the efficiency value */
void ControlLED4BasedOnEfficiency(int Eff) {
    static uint32_t lastBlinkTime = 0;  // Last time the LED was toggled
    uint32_t currentTime = HAL_GetTick();
    const uint32_t blinkPeriod = 500;  // Blink period in milliseconds

    if (Eff < 80) {
        // Blink LED4 if efficiency is below 80%
        if (currentTime - lastBlinkTime >= blinkPeriod) {
            HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            lastBlinkTime = currentTime;
        }
    } else {
        // Ensure LED4 is on if efficiency is 80% or above
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    }
}

/* Sets the PWM duty cycle */
void SetPWM_DutyCycle(uint32_t dutyCyclePercentage) {
    if (dutyCyclePercentage > 100) {
        dutyCyclePercentage = 100;  // Cap the duty cycle at 100%
    }

    // Calculate the new compare value based on the percentage
    uint32_t pulse = (htim5.Init.Period + 1) * dutyCyclePercentage / 100;

    // Set the new compare value for TIM5 Channel 1
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
}

/* Selects ADC Channel 1 */
void ADC_Select_CH1(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* Selects ADC Channel 4 */
void ADC_Select_CH4(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* Selects ADC Channel 14 */
void ADC_Select_CH14(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* Selects ADC Channel 15 */
void ADC_Select_CH15(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
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
