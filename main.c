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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	IDLE,
	MODO_TEMP,
	MODO_HUM
} State;

typedef struct{
	volatile State devState;
	volatile float temperature; //20bits
	volatile float pressure; //20bits
	volatile float humidity; //16bits
	volatile bool startMeasure;
}Device_Config;

typedef struct{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
}Calibration_Data;

typedef struct{
	uint8_t ID;
	bool deviceReady;
	uint8_t cmd;
	uint8_t rawData [8];
	Calibration_Data coeficientes;
	bool dataReady;
	uint32_t rawTemperature;
	uint32_t rawPressure;
	uint16_t rawHumidity;
}BME_Config;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME280_I2C_ADDRESS  (0x76<<1)

#define BME280_REG_CHIP_ID                        0xD0
#define BME280_REG_RESET                          0xE0

#define BME280_REG_CTRL_HUM                       0xF2
#define BME280_REG_STATUS                         0xF3
#define BME280_REG_CTRL_MEAS                      0xF4
#define BME280_REG_CONFIG                         0xF5
#define BME280_REG_DATA                           0xF7

#define BME280_REG_T1					          0x88
#define BME280_REG_T2					          0x8A
#define BME280_REG_T3					          0x8C
#define BME280_REG_P1					          0x8E
#define BME280_REG_P2					          0x90
#define BME280_REG_P3					          0x92
#define BME280_REG_P4					          0x94
#define BME280_REG_P5					          0x96
#define BME280_REG_P6					          0x98
#define BME280_REG_P7					          0x9A
#define BME280_REG_P8					          0x9C
#define BME280_REG_P9					          0x9E

#define BME280_REG_H1					          0xA1
#define BME280_REG_H2					          0xE1
#define BME280_REG_H3					          0xE3

#define BME280_REG_H4					          0xE4
#define BME280_REG_H5					          0xE5
#define BME280_REG_H6					          0xE7
#define BME280_TEMP_MSB                           0xFA

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
//#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
volatile State devState = {};
volatile BME_Config devBME280 = {};
volatile Device_Config dev = {};
volatile float my_temperature;
volatile uint8_t superBuffer[5];
volatile int32_t rawTemp;
volatile uint32_t rawPressure;
volatile uint16_t rawHumidity;
uint8_t min_temp_threshold = 20;
uint8_t max_temp_threshold = 25;
uint8_t minHumy = 20;
uint8_t maxHumy = 25;
uint8_t myID = 0;
bool myBool;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  dev.devState = IDLE;
  dev.temperature = 0;
  dev.humidity = 0;
  dev.pressure = 0;

  devBME280.ID = 0;



  HAL_I2C_Mem_Read(&hi2c2, BME280_I2C_ADDRESS, BME280_REG_CHIP_ID, 1, &devBME280.ID, 1, 1000);
  myID=devBME280.ID;
  //Config BME EN BLOCKING
  // LEEN LOS CALIBRation data blocking


  HAL_TIM_Base_Start_IT(&htim7);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (devState) {
	 		  case IDLE:
	 			  HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 0);
	 			  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 0);
	 			  HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);

	 			  break;
	 		  case MODO_TEMP:
	 			 if (dev.temperature < min_temp_threshold){
					 HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 0);
					 HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 0);
					 HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);
				  }else if ((dev.temperature > min_temp_threshold)&(dev.temperature < max_temp_threshold)){
					 HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 0);
					 HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 1);
					 HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);
				  }else if (dev.temperature > max_temp_threshold){
					 HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
					 HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 1);
					 HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);
				  }
	 			 break;


	 		 case MODO_HUM:
				  if (dev.humidity < minHumy){
					 HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 0);
					 HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 0);
					 HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);
				  }else if ((dev.humidity > minHumy)&(dev.humidity < maxHumy)){
					 HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 0);
					 HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 1);
					 HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);
				  }else if (dev.humidity > maxHumy){
					 HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
					 HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, 1);
					 HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 1);
				  }
				  break;
	 		 default:

	 		 	  break;
	 	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00C0EAFF;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_VERDE_Pin|LED_ROJO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_VERDE_Pin LED_ROJO_Pin */
  GPIO_InitStruct.Pin = LED_VERDE_Pin|LED_ROJO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_AMARILLO_Pin */
  GPIO_InitStruct.Pin = LED_AMARILLO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_AMARILLO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(USER_BTN_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USER_BTN_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == USER_BTN_Pin){
		if (devState == MODO_HUM){ //Si ya llegó al ultimo estado, vuelve al estado 0 IDLE.
			devState = IDLE;
		} else {
			devState++; //Aqui va cambiando el estado de manera secuencial,0, 1, 2, 3... 0, 1,...
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM7){ // Timer 1000 ms
		// Lectura coeficientes
		// Lectura Humedad
		HAL_I2C_Mem_Read_DMA(&hi2c2, BME280_I2C_ADDRESS, BME280_TEMP_MSB, 1, &superBuffer[0], 5);

	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{


	if(devBME280.ID == 0x60){
		devBME280.deviceReady = true;
		myID = devBME280.ID;
		myBool = devBME280.deviceReady;
	}else {
		devBME280.deviceReady = false;
		myID = devBME280.ID;
		myBool = devBME280.deviceReady;
	}
	rawTemp = ((uint32_t)superBuffer[0] << 12) | ((uint32_t)superBuffer[1] << 4)  | ((uint32_t)superBuffer[2] >> 4);
	//raw_Pressure =
	rawHumidity = ((uint16_t)superBuffer[3]<<8 | ((uint16_t)superBuffer[4]));

	processData(rawTemp, rawPressure, rawHumidity);
}

void processData(int32_t rawTemp, uint32_t rawPressure, uint16_t rawHumidity) {
	int32_t var1 = 0;
	int32_t var2 = 0;
	int32_t resultTemp = 0;
	int32_t t_fine = 0;

	int64_t var1p = 0;
	int64_t var2p = 0;
	int64_t resultPress = 0;

	int32_t v_x1_u32r = 0;
	int32_t resultHum = 0;

	//**** PROCESSING TEMPERATURE*****
	var1 = ((((rawTemp >> 3) - ((int32_t) devBME280.coeficientes.dig_T1 << 1)))
			* (int32_t) devBME280.coeficientes.dig_T2) >> 11;
	var2 = (((((rawTemp >> 4) - (int32_t) devBME280.coeficientes.dig_T1)
			* ((rawTemp >> 4) - (int32_t) devBME280.coeficientes.dig_T1)) >> 12)
			* (int32_t) devBME280.coeficientes.dig_T3) >> 14;

	resultTemp = var1 + var2;
	t_fine = resultTemp;
	resultTemp = (resultTemp*5+128)>>8;

	//**** PROCESSING PRESSURE*****
	var1p = (int64_t) t_fine - 128000;
	var2p = var1p * var1p * (int64_t) devBME280.coeficientes.dig_P6;
	var2p = var2p + ((var1p * (int64_t) devBME280.coeficientes.dig_P5) << 17);
	var2p = var2p + (((int64_t) devBME280.coeficientes.dig_P4) << 35);
	var1p = ((var1p * var1p * (int64_t) devBME280.coeficientes.dig_P3) >> 8)
			+ ((var1p * (int64_t) devBME280.coeficientes.dig_P2) << 12);
	var1p = (((int64_t) 1 << 47) + var1p) * ((int64_t) devBME280.coeficientes.dig_P1) >> 33;

	if (var1p == 0) {
		return;  // avoid exception caused by division by zero
	}

	resultPress = 1048576 - rawPressure;
	resultPress = (((resultPress << 31) - var2p) * 3125) / var1p;
	var1p = ((int64_t) devBME280.coeficientes.dig_P9 * (resultPress >> 13) * (resultPress >> 13)) >> 25;
	var2p = ((int64_t) devBME280.coeficientes.dig_P8 * resultPress) >> 19;

	resultPress = ((resultPress + var1p + var2p) >> 8) + ((int64_t) devBME280.coeficientes.dig_P7 << 4);

	//**** PROCESSING HUMIDITY*****
	resultHum = resultTemp - (int32_t) 76800;
	resultHum = ((((rawHumidity << 14) - ((int32_t) devBME280.coeficientes.dig_H4 << 20)
			- ((int32_t) devBME280.coeficientes.dig_H5 * resultHum)) + (int32_t) 16384) >> 15)
			* (((((((resultHum * (int32_t) devBME280.coeficientes.dig_H6) >> 10)
					* (((resultHum * (int32_t) devBME280.coeficientes.dig_H3) >> 11)
							+ (int32_t) 32768)) >> 10) + (int32_t) 2097152)
					* (int32_t) devBME280.coeficientes.dig_H2 + 8192) >> 14);
	resultHum = resultHum
			- (((((resultHum >> 15) * (resultHum >> 15)) >> 7)
					* (int32_t) devBME280.coeficientes.dig_H1) >> 4);
	resultHum = resultHum < 0 ? 0 : resultHum;
	resultHum = resultHum > 419430400 ? 419430400 : resultHum;
	resultHum = resultHum >> 12;

	dev.temperature = ((float)resultTemp/100);
	dev.pressure = ((float)resultPress/256/100);
	dev.humidity = ((float)resultHum/1024);
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
