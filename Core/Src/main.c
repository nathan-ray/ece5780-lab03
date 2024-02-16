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

/*
 * TIM2 interrupt request handler.
 *
 */
void TIM2_IRQHandler() {
	
	// toggle pin
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);
	
	// clearing status register
	TIM2->SR &= ~(1 << 0);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
	
	// ENABLE LEDs
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	
	// Set up a configuration struct to pass to the initialization function
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC9 high
	
	// Initialize LEDs 
  // Blue LED (PC7)
	// set to Alternate Function
  GPIOC->MODER &= ~(1 << 14);
  GPIOC->MODER |= (1 << 15);
	
  GPIOC->OTYPER &= ~(1 << 7);
  GPIOC->OSPEEDR &= ~(1 << 14);
  GPIOC->OSPEEDR &= ~(1 << 15);
  GPIOC->PUPDR &= ~(1 << 14);
  GPIOC->PUPDR &= ~(1 << 15);

  // Red LED (PC6)
	// set to Alternate Function
  GPIOC->MODER &= ~(1 << 12);
  GPIOC->MODER |= (1 << 13);
	
  GPIOC->OTYPER &= ~(1 << 6);
  GPIOC->OSPEEDR &= ~(1 << 12);
  GPIOC->OSPEEDR &= ~(1 << 13);
  GPIOC->PUPDR &= ~(1 << 12); 
  GPIOC->PUPDR &= ~(1 << 13); 
	
	// ENABLE TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	// Frequency target = 4, Fclock = 8 000 000
	// 4 = 8 000 000 / ( (PSC + 1) * ARR )
	// PSC + 1 = 2000, PSC = 1999
	// ARR = 1000
	TIM2->PSC = 7999;	// = 1999
	TIM2->ARR = 250; // = 1000
	
	// ENABLE UPDATE INTERRUPT
	TIM2->DIER |= (1 << 0);
	
	// START TIMERS
	TIM2->CR1 |= (1 << 0); 
	
	// ENABLE INTERRUPT
	NVIC_EnableIRQ(15);
	
	// ENABLE TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Frequency target = 800, Fclock = 8 000 000
	// 800 = 8 000 000 / ( (PSC + 1) * ARR )
	// PSC + 1 = 4, PSC = 3
	// ARR = 2500
  TIM3->PSC = 3;
  TIM3->ARR = 2500;
	
	// SET OUTPUT for channel 1
	TIM3->CCMR1 &= ~(1 << 1);
	TIM3->CCMR1 &= ~(1 << 0);
	
	// PWM Mode 2 for channel 1
	TIM3->CCMR1 |= (1 << 6);
	TIM3->CCMR1 |= (1 << 5);
	TIM3->CCMR1 |= (1 << 4);
	
	// ENABLE OUTPUT for channel 1
	TIM3->CCER |= (1 << 0);
	
	// ENABLE PRELOAD for channel 1
	TIM3->CCMR1 |= (1 << 3);
	
	// SET OUTPUT for channel 2
	TIM3->CCMR1 &= ~(1 << 9);
	TIM3->CCMR1 &= ~(1 << 8);
	
	// PWM Mode 1 for channel 2
	TIM3->CCMR1 |= (1 << 14);
	TIM3->CCMR1 |= (1 << 13);
	TIM3->CCMR1 &= ~(1 << 12);
	
	// ENABLE OUTPUT for channel 2
	TIM3->CCER |= (1 << 4);
	
	// ENABLE PRELOAD for channel 2
	TIM3->CCMR1 |= (1 << 11);
	
	// experimenting with different CCRx values, 100% and 20%
  TIM3->CCR1 = 50; 
  TIM3->CCR2 = 50; 
	
	// ENABLE TIMER 3
	TIM3->CR1 |= (1 << 0);

  while (1)
  {
		
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
