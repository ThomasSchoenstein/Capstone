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
typedef enum PHlevels{
	 PH_VeryHigh,
	 PH_High,
	 PH_Good,
	 PH_Low,
	 PH_VeryLow,
}PHlevel;

typedef enum h2oSensors{
	dry,
	wet,
}h2oSensor;

typedef enum PumpM{
	on,
	off,
}MainPump;

typedef enum tempLevels{
	Temp_hot,
	Temp_good,
	Temp_cold,
}tempreture;

typedef enum humidLevels{
	Humid_high,
	Humid_good,
	Humid_low,
}humidity;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

PHlevel PH;
h2oSensor wetness;
tempreture temp;
humidity humid;

double setTemp_Level;
double setHumidity_Level;
double setLight_Time;
double setPH_Level;
double setPPM_Level;

double currentPH=20;        //set equal to values that should never be the case to determine if sensors aren't reading
double currentPPM=0;
double currentTemp=0;
double currentHumidity=200;

MainPump state=off;
MainPump pumpa=off;
MainPump pumpb=off;
MainPump pumpc=off;
MainPump light;
int s;
int Tickt;
int min3;

//Tempreture Variables
extern I2C_HandleTypeDef hi2c1;
uint8_t data[3];
uint8_t da[1] = {0xFE};
uint8_t read[2]= {0xE3,0xE5};
uint8_t data1[3];
double tempp;
double hu;
uint16_t sum_temp,sum_humid;
I2C_HandleTypeDef hi2c1;

double enviorment[2];
tempreture tempLevel;
humidity humidLevel;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

double getPH(void);
PHlevel PHtask(double PH_Set);
h2oSensor H2Otask(void);
MainPump PHpumps(void);
h2oSensor water(void);
void get_TempHumid(void);
tempreture TempTask(double setLevel);
humidity humidTask(double setLevel);

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
  MX_ADC1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1); //When it starts it turns on Lights
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,1); //When it starts it turns on Lights
  HAL_TIM_Base_Start_IT(&htim2);


  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  //preset for tomato plants
  setTemp_Level=73;      //degrees farenheight
  setHumidity_Level=90;  //percent humidity
  setLight_Time=8;       //hours of light
  setPH_Level=6;         //PH
  setPPM_Level=1500;     //Parts per million


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  wetness=H2Otask();           			 //Checks if Resivoir is full
	  currentPH=getPH();         			 //Gets the PH value
	  PH=PHtask(setPH_Level);     			 //Determines if PH is too High/low
	  PHpumps();                 			 //Pumps to amend PH
	  water();                    			 //Main pump operation
	  get_TempHumid();            			 //gets tempreture and humidity
	  temp=TempTask(setTemp_Level);          //determines if temp is too high/low
	  humid=humidTask(setHumidity_Level);    //determines if humidity is too high/low

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
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
  htim2.Init.Prescaler = 48000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 30000;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

double getPH(void){
	double PH=7;
	double Vnormalization=1227;
	double Vadc=0;
	double Vin=0;

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_8;      //PIN B0
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);        //Changes the ADC channel to PH sensor

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	HAL_Delay(10);  //delay for conversion time, if not it errors
	Vadc=HAL_ADC_GetValue(&hadc1);  //ADC results in a value that isn't directly tied to the voltage
	HAL_ADC_Stop(&hadc1);           //stop adc conversion
	Vin=Vadc/Vnormalization;        //devide the ADC value by the normalization constant to find the Vin, normilization constant found by compairing measured voltage with output
	PH=20.667-5.197*Vin;            //plug Vin into this function to find the PH This function was found by measuring the voltage at known PHs and plotting

	return PH;
}

PHlevel PHtask(double setLevel){
	PHlevel phLevel;
	double PH=7;

	PH=getPH();


	if(PH==setLevel+.25 || PH==setLevel-.25){   //if within .5 PH of chosen value everything is OK
		phLevel=PH_Good;
	}
	else if(PH>setLevel+.25){     //if slightly above the allowed range
		phLevel=PH_High;
	}
	else if(PH>setLevel+1){       //if very far above the allowed range
		phLevel=PH_VeryHigh;
	}
	else if(PH<setLevel-.25){   //if slightly below the allowed range
		phLevel=PH_Low;
	}
	else if(PH<setLevel-1){      //if very far below the allowed range
		phLevel=PH_VeryLow;
	}

	return phLevel;
}

h2oSensor H2Otask(void){
	double sensorInput;
	h2oSensor wetness;

	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_6;  //PIN A6
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);            //this switches the ADC channel to read the H2O sensor input

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	HAL_Delay(10);  //delay for conversion time, if not it errors
	sensorInput=HAL_ADC_GetValue(&hadc1);    //ADC results in a value that isn't directly tied to the voltage
	HAL_ADC_Stop(&hadc1);          //stop adc conversion

	if(sensorInput==0){
		wetness=dry;
	}
	else
		wetness=wet;


	return wetness;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim2){
		if(Tickt == 6){
			Tickt = 0;
			min3++;
		}
		if(min3 == 480){
			min3 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,1);
			light = on;
		}
		if(Tickt < 1 && state==off){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,1);
		state = on;
		}
		if(Tickt >= 1 && state==on){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,0);
		state=off;
		}
		Tickt++;
if(min3 == 160){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,0);
	light = off;
}
	}
}

MainPump PHpumps(void){
	if(PH==PH_Low || PH==PH_VeryLow ){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
		pumpb=on;
	}
	if(PH==PH_High || PH==PH_VeryHigh){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
			pumpa=on;
	}
if(PH==PH_Good){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	pumpb=off;
	pumpa=off;
}
return;
}

h2oSensor water(void){
	if(wetness==dry){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
		pumpc=on;
	}
	if(wetness==wet){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
		pumpc=off;
	}
}

void get_TempHumid(void){

	// HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	HAL_I2C_Master_Transmit(&hi2c1,0x80,&read[0],1,100);
	HAL_I2C_Master_Receive(&hi2c1,0x80,data,3,1000);
	sum_temp = (data[0]<<8) | data[1];
	tempp	= sum_temp;
	enviorment[0] = (-46.85 + (175.72*(tempp/65536)));
	HAL_Delay(20);
	HAL_I2C_Master_Transmit(&hi2c1,0x80,&read[1],1,100);
	HAL_I2C_Master_Receive(&hi2c1,0x80,data1,3,1000);
	sum_humid= (data1[0]<<8) | data1[1];
	hu= sum_humid;
	enviorment[1] = (-6+(125*(hu/65536)));
	HAL_Delay(20);
}

tempreture TempTask(double setLevel){
	tempreture relitiveTemp;

	if(enviorment[0]<setLevel-5)
		relitiveTemp=Temp_cold;
	else if(enviorment[0]>setLevel+5){
		relitiveTemp=Temp_hot;
	}
	else
		relitiveTemp=Temp_good;
	return relitiveTemp;
}

humidity humidTask(double setLevel){
	humidity relitiveHumid;

	if(enviorment[1]<setLevel-5){
		relitiveHumid=Humid_low;
	}
	else if(enviorment[1]>setLevel+5){
		relitiveHumid=Humid_high;
	}
	else
		relitiveHumid=Humid_good;

	return relitiveHumid;
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
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
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
