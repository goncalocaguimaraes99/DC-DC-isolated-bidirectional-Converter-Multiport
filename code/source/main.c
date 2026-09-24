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
#include <stdarg.h>
#include <stdio.h>
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define V_MAX_P1  20.0f
#define V_MAX_P2  15.0f
#define V_MAX_P3  9.5f
volatile uint8_t cmd_stop = 0;
volatile uint8_t cmd_run  = 0;


uint8_t uart_rx_byte;           // buffer de 1 byte para receção
uint8_t test_mode = 0;          // modo ativo: 1, 2 ou 3
uint8_t system_running = 0;     // 0=parado, 1=a correr


float Kp_I_P2 = 0.5f;
float Ki_I_P2 = 0.05f;
float integral_I_P2 = 0.0f;
float I_ref_P2 = 0.0f;

float Kp_I_P3 = 0.5f;
float Ki_I_P3 = 0.05f;
float integral_I_P3 = 0.0f;
float I_ref_P3 = 0.0f;

float current_P1 = 0.0f;


int16_t phi12 = 0;
int16_t phi13 = 0;

float V_ref_P1 = 20.0f;   // tensão de saída desejada em P1

// PI Porto 2
float Kp_P2 = 0.1f;
float Ki_P2 = 0.01f;
float integral_P2 = 0.0f;
float V_ref_P2 = 15.0f;       // tensão de referência em Volts

// PI Porto 3
float Kp_P3 = 0.1f;
float Ki_P3 = 0.01f;
float integral_P3 = 0.0f;
float V_ref_P3 = 9.5f;

// Limites de phase shift
#define PHASE_MIN  -180.0f    // -90°
#define PHASE_MAX   180.0f    //  +90°

#define ADC_VOLTAGE_INDEX   0

volatile uint16_t V_SENSOR_ADC;
volatile uint16_t V_SENSOR_ADC2;
volatile uint16_t V_SENSOR_ADC3;
volatile uint16_t V_SENSOR_ADC4;
volatile uint16_t V_SENSOR_ADC5;
volatile uint8_t adc_channel = 0;  // To track which channel was converted
volatile uint8_t adc_ready = 0;
volatile uint16_t adc_values[5];  // Array to store
volatile float CURRENT_SENSOR_offset= 1.65;
#define DEG_TO_TICKS(deg) ((uint16_t)((deg) * 720.0f / 360.0f))
int callback_count=0;
uint8_t offset_captured = 0;
float float_adc_value1;
float float_adc_value2;
float float_adc_value3;
float float_adc_value4;
float float_adc_value5;
float current_P2;
float current_P3;
//float float_I_SENSOR_ADC;
//float current;
#define OFFSET_SAMPLES 100
float offset_sum = 0;
uint8_t offset_count = 0;
#define CURRENT_SENS_MV_PER_A  0.132f  // 185mV/A
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Set_PhaseShift_P2(int16_t ticks);
void Set_PhaseShift_P3(int16_t ticks);
float PI_Controller(float ref, float measured,float Kp, float Ki,float *integral,float out_min, float out_max);
float PI_Current_Controller(float I_ref, float I_measured,float Kp, float Ki, float *integral);
void Startup_Ramp(int16_t target_phi);
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 5);

  // Arrancar TIM2 e TIM3 com Base_Start primeiro
  // depois activar os canais PWM manualmente
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);

  // Activar outputs PWM directamente
  //TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);  // enable CH1 e CH2
  //TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);

  TIM2->CNT = 0;
  TIM3->CNT = 0;

  // TIM1 arranca por último — gera o trigger que activa TIM2 e TIM3
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
   uint32_t print_counter = 0;

   //Set_PhaseShift_P2(40);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (adc_ready)
	      {

	          float_adc_value1 = ((V_SENSOR_ADC  * 3.3f) / 4095.0f)*(30/3);  // adc port1
	          float_adc_value2 = ((V_SENSOR_ADC2  * 3.3f) / 4095.0f)*(V_MAX_P2/3); //volt port2
	          float_adc_value3 = ((V_SENSOR_ADC3  * 3.3f) / 4095.0f)*(V_MAX_P3/3); //voltagem port 3
	          float_adc_value4 = (V_SENSOR_ADC4 * 3.3f) / 4095.0f;
	          float_adc_value5 = (V_SENSOR_ADC5 * 3.3f) / 4095.0f;

	          current_P2 = (float_adc_value4 - CURRENT_SENSOR_offset) / CURRENT_SENS_MV_PER_A;
	          current_P3 = (float_adc_value5 - CURRENT_SENSOR_offset) / CURRENT_SENS_MV_PER_A;

	          if (cmd_stop)
	          {

	        	      cmd_stop = 0;
	        	      system_running = 0;

	        	      // Ramp down gradual do phase shift → evita glitch de corrente
	        	      int16_t current_phi = phi12;  // valor atual
	        	      while (current_phi != 0)
	        	      {
	        	          if (current_phi > 0) current_phi -= 2;
	        	          else                 current_phi += 2;

	        	          Set_PhaseShift_P2(current_phi);
	        	          Set_PhaseShift_P3(current_phi);
	        	          HAL_Delay(1);  // ~1ms por passo — ramp suave
	        	      }

	        	      // Phase shift a 0, corrente no trafo decaiu
	        	      HAL_Delay(10);

	        	      // Para TIM1
	        	      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	        	      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);

	        	      // Desliga outputs — replica estado inicial
	        	      TIM2->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
	        	      TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
	        	      TIM2->CNT = 0;
	        	      TIM3->CNT = 0;

	        	      integral_P2 = 0.0f; integral_P3 = 0.0f;
	        	      integral_I_P2 = 0.0f; integral_I_P3 = 0.0f;
	        	      phi12 = 0; phi13 = 0;

	        	      printf("STM32 STOPPED\r\n");
	          }

	          if (cmd_run)
	          {
	              cmd_run = 0;
	              system_running = 1;
	              TIM2->CNT = 0; TIM3->CNT = 0;
	              TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
	              TIM3->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
	              HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	              HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

	              if (test_mode == 0)       Startup_Ramp(40);
	              else if (test_mode == 1)  Startup_Ramp(-40);
	              else                      Startup_Ramp(10);  // modos 2 e 3


	              printf("RUN mode %d\r\n", test_mode);
	          }
	          if (system_running)
	              {
    	          if (++print_counter >= 1000)
    	          {
    	              printf("Mode: %i\nV1: %.3f V2: %.3f V3: %.3f I2:  %.3f I3:  %.3f\r\n", test_mode, float_adc_value1, float_adc_value2, float_adc_value3, current_P2, current_P3);
    	              print_counter = 0;
    	          }
	                  if (test_mode == 0)
	                  {
	                      // Teste 1 — PWM fixo sem controlo
	                      Set_PhaseShift_P2(40);
	                      Set_PhaseShift_P3(40);
	        	          // Só imprime 1 vez por cada 1000 amostras (~20Hz)
	                  }
	                  else if (test_mode == 1)
	                  {
	                      Set_PhaseShift_P2(-40);
	                      Set_PhaseShift_P3(-40);
	                  }
    	          else if (test_mode == 2)
	                  {
	                      // Teste 2 — PI tensão
	                      phi12 = (int16_t)PI_Controller(V_ref_P1, float_adc_value1, Kp_P2, Ki_P2, &integral_P2, PHASE_MIN, PHASE_MAX);
	                      phi13 = (int16_t)PI_Controller(V_ref_P1, float_adc_value1, Kp_P3, Ki_P3, &integral_P3, PHASE_MIN, PHASE_MAX);
	                      Set_PhaseShift_P2(phi12);
	                      Set_PhaseShift_P3(phi13);
	                  }
	                  else if (test_mode == 3)
	                  {
	                      // Teste 3 — PI cascata tensão + corrente
	                      I_ref_P2 = PI_Controller(V_ref_P1, float_adc_value1, Kp_P2, Ki_P2, &integral_P2, -5.0f, 5.0f);
	                      I_ref_P3 = PI_Controller(V_ref_P1, float_adc_value1, Kp_P3, Ki_P3, &integral_P3, -5.0f, 5.0f);
	                      phi12 = (int16_t)PI_Current_Controller(I_ref_P2, current_P2, Kp_I_P2, Ki_I_P2, &integral_I_P2);
	                      phi13 = (int16_t)PI_Current_Controller(I_ref_P3, current_P3, Kp_I_P3, Ki_I_P3, &integral_I_P3);
	                      Set_PhaseShift_P2(phi12);
	                      Set_PhaseShift_P3(phi13);
	                  }

	              }

	          adc_ready = 0;
	      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 719;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 360;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 719;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.Pulse = 360;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 719;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 360;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 719;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 38400;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Com Trigger Mode, Set_PhaseShift funciona assim:
void Set_PhaseShift_P2(int16_t ticks)
{
    uint16_t cnt_val;

    if (ticks >= 0)
        cnt_val = (uint16_t)ticks;
    else
        cnt_val = (uint16_t)(720 + ticks);

    // Não parar o timer — esperar pelo overflow e aplicar atomicamente
    // Usar EGR para forçar update imediato sem parar
    //__disable_irq();
    TIM2->CNT = cnt_val;
    //__enable_irq();
    // Sem parar/arrancar — escrita direta ao CNT é suficiente
}

void Set_PhaseShift_P3(int16_t ticks)
{
    uint16_t cnt_val;

    if (ticks >= 0)
        cnt_val = (uint16_t)ticks;
    else
        cnt_val = (uint16_t)(720 + ticks);

    //__disable_irq();
    //TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM3->CNT  = cnt_val;
    //TIM3->CR1 |= TIM_CR1_CEN;
    //__enable_irq();
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        // DMA automatically updates adc_values array
    	V_SENSOR_ADC = adc_values[0];
        V_SENSOR_ADC2 = adc_values[1];
        V_SENSOR_ADC3 = adc_values[2];
        V_SENSOR_ADC4 = adc_values[3];
        V_SENSOR_ADC5 = adc_values[4];
        adc_ready = 1;
        // No need to restart DMA if configured in circular mode
    }
}
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

float PI_Controller(float ref, float measured,float Kp, float Ki,float *integral,float out_min, float out_max)
{
    float error    = ref - measured;
    //float error    = measured-ref;  //possivel correçao
    *integral     += error;

    // Anti-windup — limitar integral
    if (*integral > out_max / Ki) *integral = out_max / Ki;
    if (*integral < out_min / Ki) *integral = out_min / Ki;

    float output = Kp * error + Ki * (*integral);

    // Saturar saída
    if (output > out_max) output = out_max;
    if (output < out_min) output = out_min;

    return output;
}

float PI_Current_Controller(float I_ref, float I_measured,float Kp, float Ki, float *integral)
{
    // I_measured positiva → P2 a enviar → φ deve ser negativo
    // Por isso o erro é invertido relativamente ao PI de tensão
    //float error = -(I_ref - I_measured);  // ← sinal invertido
    float error = (I_ref - I_measured);

    *integral  += error;

    if (*integral > PHASE_MAX / Ki) *integral = PHASE_MAX / Ki;
    if (*integral < PHASE_MIN / Ki) *integral = PHASE_MIN / Ki;

    float output = Kp * error + Ki * (*integral);

    if (output > PHASE_MAX) output = PHASE_MAX;
    if (output < PHASE_MIN) output = PHASE_MIN;

    return output;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (uart_rx_byte)
        {
            case 's': cmd_stop = 1;  break;
            case 'r': cmd_run  = 1;  break;
            case '1': test_mode = 1; break;
            case '2': test_mode = 2; break;
            case '3': test_mode = 3; break;
        }
        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}


void Startup_Ramp(int16_t target_phi)
{
    int16_t phi = 0;
    int16_t step = (target_phi >= 0) ? 1 : -1;

    while (phi != target_phi)
    {
        phi += step;
        Set_PhaseShift_P2(phi);
        Set_PhaseShift_P3(phi);
        HAL_Delay(5);
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
