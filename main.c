/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Development of driver for DHT22 temperature/humidity sensor
  * Using GPIO PB5 as data pin, and TIM16 for timing data duration.
  *
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum dht22ReturnType {
	DHT22_RX_OK,
	DHT22_RX_TIMEOUT,
	DHT22_CHECKSUM_ERROR
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT22_RX_TIME_MAX 150 // 2 x max expected RX bit duration (us)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DHT22_Init(void);
int DHT22_Read_Data(uint8_t*,uint8_t*,uint8_t*,uint8_t*);
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
uint8_t stringBuf[60],hum_dec,hum_int,temp_dec,temp_int;
enum dht22ReturnType r;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DHT22_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf((char*)stringBuf,"DHT22 test starting...\r\n");
  HAL_UART_Transmit(&huart2, stringBuf, strlen((char*)stringBuf), HAL_MAX_DELAY);
  HAL_TIM_Base_Start(&htim16);
  //HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Boot CPU2 */
  HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  r=DHT22_Read_Data(&hum_int, &hum_dec, &temp_int, &temp_dec);
	  // Note DHT22_Init is called before return, so measurements will continue if sensor
	  // connection is broken then re-connected without need for reset.
	  if(r==DHT22_RX_TIMEOUT)
		  sprintf((char*)stringBuf,"*** Timeout waiting for data *** \r\n");
	  else if(r==DHT22_CHECKSUM_ERROR)
		  sprintf((char*)stringBuf,"*** Checksum error ***\r\n");
	  else
		  sprintf((char*)stringBuf,"Humidity: %u.%u  Temperature %u.%u \r\n",hum_int, hum_dec, temp_int, temp_dec);
	  // Display on serial monitor
	  HAL_UART_Transmit(&huart2, stringBuf, strlen((char*)stringBuf), HAL_MAX_DELAY);
	  HAL_Delay(2000);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  * Timer clock = 32 MHz, pre-scaler = 32, giving count frequency = 1 MHz (T = 1 us)
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  * USART2 is also used by ST-LINK interface, so serial monitor in CubeIDE can be used.
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  * DHT22 data pin initially set to output Open Drain, with 4.7k ext resistor.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT22_data_GPIO_Port, DHT22_data_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DHT22_data_Pin */
  GPIO_InitStruct.Pin = DHT22_data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT22_data_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void DHT22_Init(void){
	  //Ensure still in output mode
	  // Set MODER REG for GPIO port C pin 2 to 01;
	  GPIOC->MODER&=0b11111111111111111111111111011111;
	  GPIOC->MODER|=0b00000000000000000000000000010000;
	  // Set data line high
	  HAL_GPIO_WritePin(DHT22_data_GPIO_Port, DHT22_data_Pin, 1);
	  // Allow enough time for DHT22 to recognise idle state
	  HAL_Delay(5);

}

int DHT22_Read_Data(uint8_t *hum_int, uint8_t *hum_dec, uint8_t *temp_int, uint8_t *temp_dec)
 {
	uint8_t data_frame[80], count, dht_param[5],i,j;
	uint16_t timer_val_low, timer_val_high, param_sum, humidity=0, temperature=0, timer_val;
	uint8_t byte_mask;


	  // TIM16 APB clock is configured in ioc file to 1 MHz
	  // Max duration for 16 bit timer = 65536 us which is ample for 1 data frame

	  // DHT22_data_pin is initially an output
	  // Pull data line low
	  HAL_GPIO_WritePin(DHT22_data_GPIO_Port, DHT22_data_Pin, 0);
	  // Reset timer each data frame to prevent rollover
	  __HAL_TIM_SetCounter(&htim16, 0);

	  // Wait at least 18ms, so 25 ms with some safety
	  while(__HAL_TIM_GET_COUNTER(&htim16)<25000);

	  // Pull data line high and wait for DHT2 to respond
	  HAL_GPIO_WritePin(DHT22_data_GPIO_Port, DHT22_data_Pin, 1);
	  // Switch data line to INPUT
	  // Set MODER REG for GPIO port C pin 2 to 01
	  GPIOC->MODER&=0b11111111111111111111111111001111;

	  timer_val=__HAL_TIM_GET_COUNTER(&htim16);
	  // Wait for DHT to pull line low (should be within 20 - 40 us)
	  while(HAL_GPIO_ReadPin(DHT22_data_GPIO_Port, DHT22_data_Pin)==1){
		  if(__HAL_TIM_GET_COUNTER(&htim16)-timer_val>DHT22_RX_TIME_MAX){
			  DHT22_Init(); // Ready to re-start
			  return DHT22_RX_TIMEOUT;
		  }

	  }
	  timer_val=__HAL_TIM_GET_COUNTER(&htim16);
	  // DHT pulls data line low for 80 us
	  while(HAL_GPIO_ReadPin(DHT22_data_GPIO_Port, DHT22_data_Pin)==0){
		  if(__HAL_TIM_GET_COUNTER(&htim16)-timer_val>DHT22_RX_TIME_MAX){
			  DHT22_Init(); // Ready to re-start
			  return DHT22_RX_TIMEOUT;
		  }

	  }
	  timer_val=__HAL_TIM_GET_COUNTER(&htim16);
	  // DHT pulls data line high or 80 us
	  while(HAL_GPIO_ReadPin(DHT22_data_GPIO_Port, DHT22_data_Pin)==1){
		  if(__HAL_TIM_GET_COUNTER(&htim16)-timer_val>DHT22_RX_TIME_MAX){
			  DHT22_Init(); // Ready to re-start
			  return DHT22_RX_TIMEOUT;
		  }

	  }


	  // Data frame now starts with 50us training bit followed by data bit
	  count=0;
	  __HAL_TIM_SetCounter(&htim16, 0);
	  timer_val_low=__HAL_TIM_GET_COUNTER(&htim16);
	  while(count<79){
		  while(HAL_GPIO_ReadPin(DHT22_data_GPIO_Port, DHT22_data_Pin)==0){
			  if(__HAL_TIM_GET_COUNTER(&htim16)-timer_val_low>DHT22_RX_TIME_MAX){
				  DHT22_Init(); // Ready to re-start
				  return DHT22_RX_TIMEOUT;
			  }

		  }
		  timer_val_high=__HAL_TIM_GET_COUNTER(&htim16);
		  data_frame[count++]=timer_val_high-timer_val_low;

		  while(HAL_GPIO_ReadPin(DHT22_data_GPIO_Port, DHT22_data_Pin)==1){
			  if(__HAL_TIM_GET_COUNTER(&htim16)-timer_val_high>DHT22_RX_TIME_MAX){
				  DHT22_Init(); // Ready to re-start
				  return DHT22_RX_TIMEOUT;
			  }

		  }
		  timer_val_low=__HAL_TIM_GET_COUNTER(&htim16);
		  data_frame[count++]=timer_val_low-timer_val_high;
	  }

	  for(i=0; i<5; i++){
		  dht_param[i]=0;
		  for(j=0; j<=14; j+=2){
			  if(data_frame[(i*16)+j+1]>data_frame[(i*16)+j])
				  byte_mask=1;
			  else
				  byte_mask=0;
			  dht_param[i]|=(byte_mask<<(14-j)/2);
		  }
	  }
/*
	  for(count=0;count<80;count+=2){
		  sprintf((char*)stringBuf,"%u   T: %u  D %u \r\n",count, data_frame[count], data_frame[count+1]);
		  HAL_UART_Transmit(&huart2, stringBuf, strlen((char*)stringBuf), HAL_MAX_DELAY);
	  }
*/

	  param_sum=dht_param[0]+dht_param[1]+dht_param[2]+dht_param[3];
	  param_sum&=0x00ff;
	  // Check for checksum error
	  if(param_sum != dht_param[4]){
		  DHT22_Init();
		  return DHT22_CHECKSUM_ERROR;
	  }

	  humidity=dht_param[0]; // MS byte
	  humidity<<=8;
	  humidity|=dht_param[1]; // LS byte
	  *hum_int=humidity/10;
	  *hum_dec=humidity-((*hum_int)*10);

	  temperature=dht_param[2]; // LS byte
	  temperature<<=8;
	  temperature|=dht_param[3];
	  *temp_int=temperature/10;
	  *temp_dec=temperature-((*temp_int)*10);

	  // Switch data pin back to OUTPUT
	  // Set MODER REG for GPIO port C pin 2 to 01;
	  GPIOC->MODER&=0b11111111111111111111111111011111;
	  GPIOC->MODER|=0b00000000000000000000000000010000;

	  // Assert data line back to high
	  HAL_GPIO_WritePin(DHT22_data_GPIO_Port, DHT22_data_Pin, 1);
	  return DHT22_RX_OK;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	uint8_t stringBuf[50];
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  sprintf((char*)stringBuf,"*** ERROR HANDLER ***\r\n");
	  HAL_UART_Transmit(&huart2, stringBuf, strlen((char*)stringBuf), HAL_MAX_DELAY);

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
