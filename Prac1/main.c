//PRACTICAL 1
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include "stm32f0xx.h"
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
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
volatile uint8_t currentPattern = 0;
volatile uint32_t period = 1000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);

void setPattern(uint8_t pattern);
void changePeriod(uint32_t newPeriod);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  setPattern(currentPattern);
  // TODO: Start timer TIM16
  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay
    if (LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin) == 0) //check if PA0 is pressed, creating 0.5s delay
    {
    	changePeriod(500);
    } else if (LL_GPIO_IsInputPinSet(Button1_GPIO_Port, Button1_Pin) == 0) //check if PA1 is pressed, creating a 2s delay
    {
    	changePeriod(2000);
    } else if (LL_GPIO_IsInputPinSet(Button2_GPIO_Port, Button2_Pin) == 0) //check if PA2 is pressed, creating a 1s delay
    {
    	changePeriod(1000);
    } else if (LL_GPIO_IsInputPinSet(Button3_GPIO_Port, Button3_Pin) == 0) //check if PA3 is pressed, setting pattern sequence back to beginning
    {
    	currentPattern = 0;
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = period-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Timer rolled over
void TIM16_IRQHandler(void)
{
	// Acknowledge interrupt
	HAL_TIM_IRQHandler(&htim16);

	// TODO: Change LED pattern
	setPattern(currentPattern);
	currentPattern = (currentPattern + 1) % 9;


  
}

/* USER CODE END 4 */
void setPattern(uint8_t pattern)
{
	switch (pattern)
	{
	case 0: //11101001
		LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
	    LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
	    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
	    LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
	    LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
	    LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
	    LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin);
	    break;
	case 1: //11010010
		LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 2: //10100100
		LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 3: //01001000
		LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 4: //10010000
		LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 5: //00100000
		LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 6: //01000000
		LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 7: //10000000
		LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	case 8: //00000000
		LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	default: //All LEDs set to LOW
		LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
		LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
		LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
		LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
		LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
		LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
		LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
		LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
	}
}

void changePeriod(uint32_t newPeriod)
{
	period = newPeriod;
	__HAL_TIM_SET_AUTORELOAD(&htim16, period-1);
	HAL_TIM_Base_Stop_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim16);
}
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
