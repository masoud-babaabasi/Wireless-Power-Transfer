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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */
#define RED 1
#define GREEN 0
#define ch1_2 1
#define ch3_4 3
#define INPHASE 1
#define OFF 0
#define ANTIPHASE -1
#define SHORTCIRCUIT 0
#define OPENCIRCUIT 1
	
TIM_TypeDef  *timers[7] = {TIM1,TIM2,TIM4,TIM4,TIM3,TIM3,TIM2};
const uint8_t channels[7] ={1,3,1,3,1,3,1};
uint16_t freq = 65;
uint8_t deadtime = 8;
float voltage = 4;
int8_t transmiter_state[7]={OFF,OFF,OFF,OFF,OFF,OFF,OFF};
uint16_t LEDs_state = 0;
uint8_t flags = 6;
//FATFS USBDISKFatFs;           /* File system object for USB disk logical drive */
//FIL MyFile;                   /* File object */
//char USBDISKPath[4];          /* USB Host logical drive path */
//USBH_HandleTypeDef hUSBHost; /* USB Host handle */
//static void MSC_Application(void);
//extern ApplicationTypeDef Appli_state;
char buff[5];
char received_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void timers_init();
void setRegVoltage(float Vo);
void resetLEDs();
void setLEDs(uint8_t COL, uint8_t NUM , uint8_t ON_OFF );
void set_frequency(int f);
void PWM_SET(TIM_HandleTypeDef* timer, uint8_t channel ,int8_t phase,int f,uint8_t deadtime);
void transmiter(uint8_t num,int8_t state , uint8_t HighZ);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	union transmit_data
    {
    uint16_t ADCValue[7];
    uint8_t TXdata[14];
    }TData;

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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	resetLEDs();
//	__HAL_RCC_TIM2_CLK_ENABLE();
//	TIM2->CR1 = (1<<0)|(1<<5)|(1<<6);
//	TIM2->CCMR1 = (1<<5)|(1<<6)|(1<<14)|(1<<13);
//	TIM2->CCER = (1<<0)|(1<<4);
//	TIM2->ARR = freq;
//	TIM2->CCR1 = (freq/2)-5;
//	TIM2->CCR2 = (freq/2)+5;
//	__HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = GPIO_PIN_3;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	//RCC->APB1ENR = RCC->APB1ENR |(uint32_t)(1<<0);
	
	
	
//	for(int i=1 ; i<=7 ; i++) {
//		if(i>1) setLEDs(RED,i-1,0);
//		setLEDs(RED,i,1);
//		HAL_Delay(100);
//	}
//	setLEDs(RED,7,0);
//	for(int i=1 ; i<=7 ; i++){
//		if(i>1) setLEDs(GREEN,i-1,0);
//		setLEDs(GREEN,i,1);
//		HAL_Delay(100);
//	}
//	setLEDs(GREEN,7,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//int i;
	timers_init();
	HAL_GPIO_WritePin(REG_ON_GPIO_Port,REG_ON_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(POT_CS_GPIO_Port,POT_CS_Pin,GPIO_PIN_SET);
	HAL_Delay(100);
	setRegVoltage(4);
	/*for(i=3;i<=12;i++) {
		setRegVoltage(i);
		HAL_Delay(500);
	}*/
	//PWM_SET(&htim4 ,ch3_4 , ANTIPHASE ,64 , 7);
	//PWM_SET(&htim2 ,ch3_4 , INPHASE ,64 , 7);
	/*for(int i =1 ; i <= 7 ; i++){
			transmiter(i,ANTIPHASE);
		if(i>1) transmiter(i-1,OFF);
		HAL_Delay(2000);
	}*/
	//HAL_TIM_Base_Stop(&htim1);
	//HAL_TIM_Base_Stop(&htim2);
	//HAL_TIM_Base_Stop(&htim3);
	//HAL_TIM_Base_Stop(&htim4);
	HAL_Delay(200);
	//transmiter(2,INPHASE);
	HAL_Delay(20);
	transmiter(6,ANTIPHASE,OPENCIRCUIT);
//	char tx_state = 'S';
//	buff[0] = 'S';
	set_frequency(102);
	//transmiter(6,ANTIPHASE);
	//transmiter(7,ANTIPHASE);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	
	//HAL_TIM_Base_Start(&htim1);
	TIM1->CR1 |= (1<<0);
	TIM5->CR1 |= (1<<0);
	
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	//HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);		

	float v = 4;
	
	uint16_t f=0;
	//unsigned int ADCValue[9];
	uint8_t i=0;
	//Appli_state = APPLICATION_START;
	HAL_UART_Receive_IT(&huart5,(uint8_t *)buff, 2);
	//void *packetADD = &(ADCValue[0]);
  while (1)
  {
		/*HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		HAL_Delay(500);*/
		/*setRegVoltage(v);
		if(f){
			v = v + 0.5;
			if(v==10) f=0;
		}
		else{
			v = v - 0.5;
			if(v==3) f=1;
		}
		HAL_Delay(500);*/
/*************** ADC start conversion*******************************/
		/*HAL_ADC_Start(&hadc1);
		for(i=0;i<10;i++){
			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
					TData.ADCValue[i] = HAL_ADC_GetValue(&hadc1);
			}
		}
		HAL_UART_Transmit (&huart5, TData.TXdata ,2*7,10);*/
		//HAL_Delay(10);

/**************UART input data interpatation*************************/		
		if(received_flag){
				received_flag = 0;
				if( buff[0] == 'f' || buff[0] == 'F' ){
					freq = ( (uint8_t)buff[1] );
					set_frequency(freq);
				}else if( buff[0] == 'v' || buff[0] == 'V' ){
					voltage = (( uint8_t)buff[1] ) * 10 / 255.0 + 1.9;
					setRegVoltage( voltage );
				}
				else if( buff[0] == 't' || buff[0] == 'T' ){
					flags = (( uint8_t)buff[1] )  & ( 0x0f );
					transmiter_state[ flags - 1 ] = (int8_t) ( ( uint8_t)buff[1]  & ( 0xf0 ) ) >> 4 ;
					transmiter( flags , transmiter_state[ flags - 1 ] , OPENCIRCUIT );
				}
				else if( buff[0] == 'S' && buff[1] == 'C' ){ //start conversion command
					HAL_ADC_Start(&hadc1);
					for(i=0;i<10;i++){
						if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
								TData.ADCValue[i] = HAL_ADC_GetValue(&hadc1);
						}
					}
					HAL_UART_Transmit (&huart5, TData.TXdata ,2*7,10);
				}
}
/*******************frequency setting**********************************/
//		f = TData.ADCValue[3] /(2047.0) * 80 + 50 ;
//		if( f != freq){
//			freq = f;
////			HAL_TIM_Base_Stop(&htim8);
////			HAL_TIM_Base_Stop(&htim5);
////			HAL_TIM_Base_Stop(&htim1);
////			HAL_TIM_Base_Stop(&htim2);
////			HAL_TIM_Base_Stop(&htim3);
////			HAL_TIM_Base_Stop(&htim4);
//			set_frequency(f);
////			HAL_TIM_Base_Start(&htim8);
////			HAL_TIM_Base_Start(&htim5);
//		}

/********************************voltage set****************************/
//		v = TData.ADCValue[4] / (2047.0)  * 8.0  + 4.0;
//		if(v != voltage ) {
//			voltage = v;
//			setRegVoltage(v);
//		}
		HAL_Delay(10);
		
/********************** transmiter number set *************************/
//		if( HAL_GPIO_ReadPin(PB2_GPIO_Port,PB2_Pin) == GPIO_PIN_RESET){
//			flags++;
//			if(flags>= 8 ) flags =6;
//			transmiter(flags,ANTIPHASE,OPENCIRCUIT);
//			/*if(flags == 0 ) {
//				transmiter(7,ANTIPHASE);
//				flags =1;
//			}
//			else if(flags == 1 ) {
//				transmiter(7,INPHASE);
//				flags =2;
//			}else if(flags == 2 ) {
//				transmiter(7,OFF);
//				flags =0;
//			}*/
//			/*flags++;
//			if(flags==8)flags = 1;
//			if(flags == 1) transmiter(7,OFF);
//			else transmiter(flags-1,OFF);
//			transmiter(flags,ANTIPHASE);*/
//			HAL_Delay(300);
//		}
/******************* transmiter state set*****************************/
//		if( HAL_GPIO_ReadPin(PB1_GPIO_Port,PB1_Pin) == GPIO_PIN_RESET){
//			int8_t st = transmiter_state[flags-1];
////			st++;
////			if(st == 2) st = -1;
//			st--;
//			if(st <= -2) st = 1;
//			transmiter(flags,st,OPENCIRCUIT);
//			HAL_Delay(300);	
//		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_1LINE;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_1LINE;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
	

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 65;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 64;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
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
  sConfigOC.Pulse = 27;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 44;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim3.Init.Period = 64;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
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
  sConfigOC.Pulse = 27;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim4.Init.Period = 65;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1;
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
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_GATED;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|REG_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(REG_clk_GPIO_Port, REG_clk_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, POT_CS_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2_Pin */
  GPIO_InitStruct.Pin = PB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 REG_ON_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12|REG_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : REG_clk_Pin */
  GPIO_InitStruct.Pin = REG_clk_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REG_clk_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1_Pin */
  GPIO_InitStruct.Pin = PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : POT_CS_Pin LED1_Pin */
  GPIO_InitStruct.Pin = POT_CS_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/********************************************************************************************/
void timers_init(){
	TIM1->CCER &= ~((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) );
	TIM2->CCER &= ~((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) );
	TIM3->CCER &= ~((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) );
	TIM4->CCER &= ~((1 << 0) | (1 << 4) | (1 << 8) | (1 << 12) );
	
	TIM1->CR1 |= (1<<0);
	TIM2->CR1 |= (1<<0);
	TIM3->CR1 |= (1<<0);
	TIM4->CR1 |= (1<<0);
}
/********************************************************************************************/
void setRegVoltage(float Vo){
	uint16_t pData;
	//pData = 0x11;
//	if( Vo >=2.717 && Vo <= 12) pData = 1.482/(Vo-1.235)*256.00;
//	else if( Vo < 2.717 ) pData = 255;
//	else if( Vo > 12 ) pData = 35;
	if( Vo >=1.9 && Vo <= 12) pData = 4.7*1.23*256.0/10.0/(Vo-1.23);
	else if( Vo < 1.9 ) pData = 255;
	else if( Vo > 12 ) pData = 14;
	pData = pData | (uint16_t)(0x11 << 8) ;
	HAL_GPIO_WritePin(POT_CS_GPIO_Port,POT_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,(uint8_t *)&pData,1,100);
	HAL_GPIO_WritePin(POT_CS_GPIO_Port,POT_CS_Pin,GPIO_PIN_SET);
  /* USER CODE END UART7_Init 2 */

}

/********************************************************************************************/
void resetLEDs(){
	uint8_t LED_data1 = 0x00;
	HAL_SPI_Transmit(&hspi3,(uint8_t *)&LED_data1,1,100);
	HAL_SPI_Transmit(&hspi3,(uint8_t *)&LED_data1,1,100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(REG_clk_GPIO_Port,REG_clk_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(REG_clk_GPIO_Port,REG_clk_Pin,GPIO_PIN_RESET);
}
/********************************************************************************************/
void setLEDs(uint8_t COL, uint8_t NUM , uint8_t ON_OFF ){
		uint16_t d = 0 ;
		if( NUM <= 7 && NUM >= 1 && (COL == RED || COL == GREEN)){
			d = 1 << (LED_POS[NUM-1]*2 + COL);
			if(ON_OFF == 1) LEDs_state |= d;
			else if(ON_OFF == 0) LEDs_state &= ~d;			 
			uint8_t LED_data1 = LEDs_state & 0xff;
			uint8_t LED_data2 = LEDs_state >> 8;
			HAL_SPI_Transmit(&hspi3,&LED_data1,1,100);
			HAL_SPI_Transmit(&hspi3,&LED_data2,1,100);
			HAL_Delay(1);
			HAL_GPIO_WritePin(REG_clk_GPIO_Port,REG_clk_Pin,GPIO_PIN_SET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(REG_clk_GPIO_Port,REG_clk_Pin,GPIO_PIN_RESET);
		}
}
/********************************************************************************************/
void set_frequency(int f){
	TIM1->ARR = f;
	TIM2->ARR = f;
	TIM3->ARR = f;
	TIM4->ARR = f;
	for(int i=0;i<7;i++){
		if( channels[i] == ch1_2){
			timers[i]->CCR1 = (uint16_t)(f / 2)-transmiter_state[i]*deadtime;
			timers[i]->CCR2 = (uint16_t)(f / 2)+ transmiter_state[i]*deadtime;
		}else if( channels[i] == ch3_4){
			timers[i]->CCR3 = (uint16_t)(f / 2)- transmiter_state[i]*deadtime;
			timers[i]->CCR4 = (uint16_t)(f / 2)+ transmiter_state[i]*deadtime;
		}
	}
}
/********************************************************************************************/
void PWM_SET(TIM_HandleTypeDef* timer,uint8_t channel ,int8_t phase ,int f,uint8_t deadtime)
{
//	timer->Init.Period = f;
//	HAL_TIM_PWM_Init(timer);
//	TIM_OC_InitTypeDef sConfigOC = {0};
//	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;

	//sConfigOC.Pulse = (uint16_t)(f / 2);

//	if( phase == INPHASE) sConfigOC.OCMode = TIM_OCMODE_PWM1;
//  else if( phase == ANTIPHASE) sConfigOC.OCMode = TIM_OCMODE_PWM2;
//  sConfigOC.Pulse = (int)(f / 2) - (phase)*deadtime;
//	if(channel == ch1_2) HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, TIM_CHANNEL_1);
//	else if(channel == ch3_4) HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, TIM_CHANNEL_3);
//  sConfigOC.Pulse = (int)(f / 2) + (phase)*deadtime;
//	if(channel == ch1_2) HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, TIM_CHANNEL_2);
//	else if(channel == ch3_4) HAL_TIM_PWM_ConfigChannel(timer, &sConfigOC, TIM_CHANNEL_4);
  	HAL_TIM_MspPostInit(timer);
}
/********************************************************************************************/
void transmiter(uint8_t num,int8_t state , uint8_t HighZ)
{
	uint16_t temp;
	if(state == INPHASE ){
			if(channels[num-1] == ch1_2){
			timers[num-1]->CCER &= ~((1 << 0) | (1 << 4));
			timers[num-1]->CCMR1 |= ((1<<5)|(1<<6)|(1<<13)|(1<<14));
			timers[num-1]->CCMR1 &= ~((1<<4)|(1<<12));//pwm mode1
			timers[num-1]->CCR1 = (uint16_t)(freq / 2)- state*deadtime;
			timers[num-1]->CCR2 = (uint16_t)(freq / 2)+ state*deadtime;
			timers[num-1]->CCER |= ((1 << 0) | (1 << 4));//output enable
			}else if(channels[num-1] == ch3_4){
			timers[num-1]->CCER &= ~((1 << 8) | (1 << 12));
			timers[num-1]->CCMR2 |= ((1<<5)|(1<<6)|(1<<13)|(1<<14));
			timers[num-1]->CCMR2 &= ~((1<<4)|(1<<12));//pwm mode1
			timers[num-1]->CCR3 = (uint16_t)(freq / 2)- state*deadtime;
			timers[num-1]->CCR4 = (uint16_t)(freq / 2)+ state*deadtime;
			timers[num-1]->CCER |= ((1 << 8) | (1 << 12));//output enable
			}
			setLEDs(RED , num , 1);
			setLEDs(GREEN , num , 0);
	}else if(state == ANTIPHASE ){
		 if(channels[num-1] == ch1_2){
			timers[num-1]->CCER &= ~((1 << 0) | (1 << 4));
			timers[num-1]->CCMR1 |= ((1<<4)|(1<<5)|(1<<6)|(1<<12)|(1<<13)|(1<<14));//pwm mode2
			timers[num-1]->CCR1 = (uint16_t)(freq / 2)- state*deadtime;
			timers[num-1]->CCR2 = (uint16_t)(freq / 2)+ state*deadtime;
			timers[num-1]->CCER |= ((1 << 0) | (1 << 4));//output enable
			}else if(channels[num-1] == ch3_4){
			timers[num-1]->CCER &= ~((1 << 8) | (1 << 12));
			timers[num-1]->CCMR2 |= ((1<<4)|(1<<5)|(1<<6)|(1<<12)|(1<<13)|(1<<14));//pwm mode2
			timers[num-1]->CCR3 = (uint16_t)(freq / 2)- state*deadtime;
			timers[num-1]->CCR4 = (uint16_t)(freq / 2)+ state*deadtime;
			timers[num-1]->CCER |= ((1 << 8) | (1 << 12));//output enable
			}
			setLEDs(GREEN , num , 1);
			setLEDs(RED , num , 0);
	}else if(state == OFF){
		if(channels[num-1] == ch1_2){
			timers[num-1]->CCER &= ~((1 << 0) | (1 << 4)); // output disable
			timers[num-1]->CCMR1 |= ((1<<6)|(1<<14));//output forced mode inactive level ch1 inactive level ch2(gate drive is inverted- nmos is ch2)
			timers[num-1]->CCMR1 &= ~((1<<4)|(1<<5)|(1<<12)|(1<<13));
			if( HighZ == SHORTCIRCUIT ) timers[num-1]->CCER |= ((1 << 0) | (1 << 4));//output enable
		}
		if(channels[num-1] == ch3_4){
			timers[num-1]->CCER &= ~((1 << 8) | (1 << 12));
			timers[num-1]->CCMR2 |= ((1<<6)|(1<<14));//output forced mode inactive level ch1 inactive level ch2(gate drive is inverted- nmos is ch2)
			timers[num-1]->CCMR2 &= ~((1<<4)|(1<<5)|(1<<12)|(1<<13));
			if( HighZ == SHORTCIRCUIT ) timers[num-1]->CCER |= ((1 << 8) | (1 << 12));//output enable
		}
		setLEDs(RED , num , 0);
		setLEDs(GREEN , num , 0);
	}
	transmiter_state[num-1] = state;
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//char buff2[5]="done\n";
  HAL_UART_Receive_IT(&huart5,(uint8_t *)buff, 2);
	received_flag = 1;
//	HAL_UART_Transmit (&huart5,(uint8_t *)buff ,5,500);	
//	if(buff[0] == 'I'){
//		freq = 92;
////		set_frequency(freq);
////		transmiter(6,ANTIPHASE,OPENCIRCUIT);
////		transmiter(7,ANTIPHASE,OPENCIRCUIT);
////		HAL_UART_Transmit (&huart5,(uint8_t *)buff2 ,5,50);		
//	}else if(buff[0] == 'A'){
//		freq = 97;
////		set_frequency(freq);
////		transmiter(6,ANTIPHASE,OPENCIRCUIT);
////		transmiter(7,INPHASE,OPENCIRCUIT);		
////		HAL_UART_Transmit (&huart5,(uint8_t *)buff2 ,5,50);	
//	}else if(buff[0] == 'S'){
//		freq = 95;
////		set_frequency(freq);
////		transmiter(6,ANTIPHASE,OPENCIRCUIT);
////		transmiter(7,OFF,OPENCIRCUIT);
////HAL_UART_Transmit (&huart5,(uint8_t *)buff2 ,5,50);			
//	}
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
	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
