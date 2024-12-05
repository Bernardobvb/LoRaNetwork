/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "app_subghz_phy.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys_app.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
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

/* USER CODE BEGIN PV */
// Declaration of the node IDs

uint8_t id_node = 236;
uint16_t id_temperature = 556;
uint16_t id_humidity = 557;
uint16_t id_LDR = 558;
uint16_t id_frame = 0;


uint8_t my_appKey[4] = {5, 6, 7, 8}; // Security process for the LoRa transmission
uint8_t app_key_offset = sizeof(my_appKey);

// Declaration of the variables
uint8_t Rh_byte1 = 0, Rh_byte2 = 0, Temp_byte1 = 0, Temp_byte2 = 0;
uint16_t SUM = 0, RH = 0, TEMP = 0;
uint8_t Presence = 0;
uint16_t LDR = 0;
uint8_t counter;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// set a digital pin in output mode
void set_Pin_Output(GPIO_TypeDef *, uint16_t);
// set a digital pin in input mode
void set_Pin_Input(GPIO_TypeDef *, uint16_t);
// Start of the temperature/humidity DHT11 sensor
void DHT11_Start(void);
// Check the response of the DHT11 sensor
uint8_t DHT11_CheckResponse(void);
// read the data measured by the DHT11 sensor
uint8_t DHT11_ReadData(void);
// read the analog input of the LDR and return the LUX value of the light
uint16_t ReadLDR(void);
// delay function in microseconds units
void delay(uint32_t delay);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay(uint32_t delay)
{

	// Initialize Timer1 for 1 MHz clock (1 tick = 1 µs) 
    TIM_HandleTypeDef htim1;
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 47;  // (48 MHz / (47+1)) = 1 MHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;  // Max 16-bit count (65.535 ms max delay per cycle)
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    // implementation of the delay

    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        // Handle initialization error
        Error_Handler();
    }

    // Start the timer
    HAL_TIM_Base_Start(&htim1);

    // Handle delays larger than 65535 ticks (65.535 ms)
    while (delay > 65535)
    {
        __HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset counter
        while (__HAL_TIM_GET_COUNTER(&htim1) < 65535);  // Wait for timer to reach max count
        delay -= 65535;
    }

    // Handle the remaining delay if it's less than 65535 ticks
    if (delay > 0)
    {
        __HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset counter
        while (__HAL_TIM_GET_COUNTER(&htim1) < delay);  // Wait until counter reaches the remaining delay
    }

    // Stop the timer after the delay
    HAL_TIM_Base_Stop(&htim1);
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
  MX_SubGHz_Phy_Init();
  MX_ADC_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  MX_USART2_UART_Init();
  //HAL_TIM_Base_Start(&htim1);

  counter = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// Ensure that the LDR is initialized
	MX_ADC_Init();

	// SET LED 2 HIGH BEGINNING the loop
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	delay(1000000); // 1 s delay
	// SET LED 2 LOW
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

	// Starting the DHT11
	DHT11_Start ();

	// After the start check is response
	Presence = DHT11_CheckResponse();

	// Implement the protocol to read and store all the measures of the DHT11

	// Read the humidity data
	Rh_byte1 = DHT11_ReadData();

	Rh_byte2 = DHT11_ReadData();

	// Read the temperature data
	Temp_byte1 = DHT11_ReadData();

	Temp_byte2 = DHT11_ReadData();

	// Read the sum that is needed to check that all the data readed and store was corrected
	SUM = DHT11_ReadData();

	if (SUM == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2)) // if the read was correct
	{
			TEMP = Temp_byte1; // store the Temperature data
			RH = Rh_byte1; // store the humidity data
	}


	// a delay to start the read of the analogic LDR
	HAL_Delay(1000);
	LDR = ReadLDR(); // calls the function and store in LDR the number of LUX in the measure

	delay(20000000); // Delay 20 seconds before next reading

	// implement a counter to send via LoRa the readed data just after 9 iterations of the loop
	if (counter == 9){
		// will be send every 3min 10s
		MX_SubGHz_Phy_Process();
		counter = 0;
	}
	counter++;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t ReadLDR(void) {
	// function that read the analogic value and return at the End the LUX in the environment

	uint16_t V_out, Lux;
	// define value of the R resistance 1k ohm
	float R = 1000;
	// coefficient of the LDR
	float A = 56200;
	float alpha = 0.75;
	// variables to be calculate and the voltage alim of the carte
	float V_R, R_LDR, V_LDR;
	float vcc = 3.3;

	// start the analogic digital converter
    HAL_ADC_Start(&hadc);

    if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK) {
    	// Read and store the value of the analogic pin
    	V_out = HAL_ADC_GetValue(&hadc);
    	// Convert the digital value to the real value
        V_R = V_out * (vcc/4095.0);
        // Calculate the voltage tension in the LDR
        V_LDR = vcc - V_R;
        // Calculate the actual resistance value of the LDR
        R_LDR = V_LDR*R / V_R;
        // Calculate and return the LUX in the environment
        Lux = pow( (A/R_LDR),(1/alpha));
        return Lux;
    }else{
    	// in cas of the analog pin doesn't work properly send this debug information by the LED3 in the carte
    	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
    	delay(1000000); // 1 s delay
    	// SET LED 2 LOW
    	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);

    }
    return 0;
}

void DHT11_Start(void)
{
	//Following the instructions in the datasheet of the DHT11

	//Initialize with Data pin HIGH

  // Configure the pin as output
  set_Pin_Output(IN_T_GPIO_Port, IN_T_Pin);
  delay(20000);
  HAL_GPIO_WritePin(IN_T_GPIO_Port, IN_T_Pin, 1);

  //Pull the pin low for at least 18ms
  HAL_GPIO_WritePin(IN_T_GPIO_Port, IN_T_Pin, 0);
  delay(18000);

  // Pull the pin high for 20-40us
  HAL_GPIO_WritePin(IN_T_GPIO_Port, IN_T_Pin, 1);
  delay(20);
  // Configure the pin as input
  set_Pin_Input(IN_T_GPIO_Port, IN_T_Pin);
}

void set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

	// GPIO configuration to set the digital pin in the output mode
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx,&GPIO_InitStruct);
}

void set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){

	// GPIO configuration to set the digital pin in the input mode
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx,&GPIO_InitStruct);
}


uint8_t DHT11_CheckResponse(void)
{
  // Following the datasheet of the DHT11 implementing the check response of the sensor

  // Initialize the check response variable
  uint8_t Response = 0;
  delay(40);
  // If the read of the pin is equal to 0, enter and wait for 80us, now the DHT11 needs to be sent a positive signal showing that he is answer the starting call
  if (!HAL_GPIO_ReadPin(IN_T_GPIO_Port, IN_T_Pin))
  {
    delay(80);
    if (HAL_GPIO_ReadPin(IN_T_GPIO_Port, IN_T_Pin)) Response = 1;
    else Response = -1;
  }

  while (HAL_GPIO_ReadPin(IN_T_GPIO_Port, IN_T_Pin)); // wait for the pin to go low
  return Response;
}


uint8_t DHT11_ReadData(void)
{
	// Initialize the data variable of the read
	uint8_t data = 0;
	uint8_t j;
		// Now the sensor is going to send 8 bits in sequence, synchronize the time and read a high pin as 1 and a low as 0
		for (j=0;j<8;j++)
		{
			while (!(HAL_GPIO_ReadPin (IN_T_GPIO_Port, IN_T_Pin)));   // wait for the pin to go high
			delay(40);   // wait for 40 us
			if (!(HAL_GPIO_ReadPin (IN_T_GPIO_Port, IN_T_Pin)))   // if the pin is low
			{
				data&= ~(1<<(7-j));   // write 0
			}
			else{ // the pin is high
				data|= (1<<(7-j));  // write 1
			}
			while ((HAL_GPIO_ReadPin (IN_T_GPIO_Port, IN_T_Pin)));  // wait for the pin to go low
		}
		return data;
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
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
