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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned short counter = 0;

void RunningPairs(unsigned int delay_ms) {
	uint16_t Led_adresses[] = {0x2000,	0x4000,	0x8000,	0x1000,	0x2000};
	//						   Orange	Red		Blue	Green	Orange

	HAL_GPIO_WritePin(GPIOD, Led_adresses[3], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, Led_adresses[0], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, Led_adresses[1], GPIO_PIN_SET);
	HAL_Delay(delay_ms);

	for (unsigned int i = 1; i <= 3; i++) {
		HAL_GPIO_WritePin(GPIOD, Led_adresses[i-1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, Led_adresses[i+1], GPIO_PIN_SET);
		HAL_Delay(delay_ms);
	}
}

void CrossBlink(unsigned int delay_ms) {
	HAL_GPIO_WritePin(LD3_Orange_GPIO_Port, LD3_Orange_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD6_Blue_GPIO_Port, LD6_Blue_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD5_Red_GPIO_Port, LD5_Red_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD4_Green_GPIO_Port, LD4_Green_Pin, GPIO_PIN_RESET);

	HAL_Delay(delay_ms);

	HAL_GPIO_WritePin(LD3_Orange_GPIO_Port, LD3_Orange_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD6_Blue_GPIO_Port, LD6_Blue_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD5_Red_GPIO_Port, LD5_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD4_Green_GPIO_Port, LD4_Green_Pin, GPIO_PIN_SET);

	HAL_Delay(delay_ms);
}

void PingPong(unsigned int delay_ms) {
	uint16_t Led_adresses[] = {0x2000,	0x4000,	0x8000,	0x1000};
	//						   Orange	Red		Blue	Green

	//Going clockwise
	HAL_GPIO_WritePin(LD3_Orange_GPIO_Port, LD3_Orange_Pin, GPIO_PIN_SET);
	HAL_Delay(delay_ms);
	for (unsigned short i = 1; i <= 3; i++) {
		HAL_GPIO_WritePin(GPIOD, Led_adresses[i-1], GPIO_PIN_RESET);
		//HAL_Delay(delay_ms);
		HAL_GPIO_WritePin(GPIOD, Led_adresses[i], GPIO_PIN_SET);
		HAL_Delay(delay_ms);
	}
	// Going counter-clockwise
	for (unsigned short i = 3; i >= 2; i--) {
		HAL_GPIO_TogglePin(GPIOD, Led_adresses[i]);
		HAL_GPIO_WritePin(GPIOD, Led_adresses[i-1], GPIO_PIN_SET);
		HAL_Delay(delay_ms);
	}
	HAL_GPIO_WritePin(LD5_Red_GPIO_Port, LD5_Red_Pin, GPIO_PIN_RESET);
}

void AllBlink(unsigned int delay_ms) {
	HAL_GPIO_WritePin(LD3_Orange_GPIO_Port, LD3_Orange_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD5_Red_GPIO_Port, LD5_Red_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD6_Blue_GPIO_Port, LD6_Blue_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD4_Green_GPIO_Port, LD4_Green_Pin, GPIO_PIN_SET);

	HAL_Delay(delay_ms);

	HAL_GPIO_WritePin(LD3_Orange_GPIO_Port, LD3_Orange_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD5_Red_GPIO_Port, LD5_Red_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD6_Blue_GPIO_Port, LD6_Blue_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LD4_Green_GPIO_Port, LD4_Green_Pin, GPIO_PIN_RESET);

	HAL_Delay(delay_ms);

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
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  switch(counter % 4) {
		case 0:
			RunningPairs(250);
			break;
		case 1:
			CrossBlink(250);
			break;
		case 2:
			PingPong(100);
			break;
		case 3:
			AllBlink(250);
			break;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Green_Pin|LD3_Orange_Pin|LD5_Red_Pin|LD6_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Green_Pin LD3_Orange_Pin LD5_Red_Pin LD6_Blue_Pin */
  GPIO_InitStruct.Pin = LD4_Green_Pin|LD3_Orange_Pin|LD5_Red_Pin|LD6_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
