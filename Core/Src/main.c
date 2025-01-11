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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PMSM_FUNC.h"
#include "board_config_V1.h"
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
#ifdef ENABLE_UART_DEBUG
char stringToUART[100] = "buffer here\r\n";//{'\0',};
#endif
uint16_t ADCBuffer[6]={0,};
extern uint32_t globalTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
//  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  initGPIO();
	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc,(uint32_t*)&ADCBuffer,6);

	PMSM_Init();
	initController();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //sendToUART("HELLO\r\n");
	  //snprintf(stringToUART,100,"ADCBuffer=%d\r\n",ADCBuffer[0]);
	  //sendToUART(stringToUART);

	  if ((ADCBuffer[0] & 0xFFF0) > getThrottleStartValue()) {
		  // If Motor Is not running
		  if (!isMotorRunning()) {
			  // Start motor
			  // Check Reverse button
			  if (isReverseButtonPressed()) {
				  // Reverse
				  setMotorSpinDirection(PMSM_CCW);
			  } else {
				  // Forward
				  setMotorSpinDirection(PMSM_CW);
			  }
			  BLDC_MotorCommutation(getRotorSector());
			  setMotorRunningState(true);
		  }

		  PMSM_updatePMSMPWMVariable(PMSM_ADCToPWM(ADCBuffer[0] & 0xFFF0));
		  __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);//start timer 1 interrupt
#ifdef ENABLE_UART_DEBUG
		  snprintf(stringToUART,100,"PMSM_PWM=%d\r\n",PMSM_ADCToPWM(ADCBuffer[0] & 0xFFF8));
		  //snprintf(stringToUART,100,"GT=%d CNT=%d\r\n",globalTime,counter);
		  sendToUART(stringToUART);
#endif
		  GPIO_SET_PIN(GPIOB, GREEN_LED);//set green LED
	  }else {
		  __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_UPDATE);//stop timer 1 interrupt
		  PMSM_SetPWMWidthToYGB(0);
		  GPIO_SET_PIN(GPIOB, GREEN_LED);//reset green LED
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
//  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/* USER CODE BEGIN 4 */
//*********************************************handling system clk***************//
void SystemClock_Config(void)
{
  // Enable HSI (High Speed Internal) clock
  RCC->CR |= RCC_CR_HSION;

  // Wait till HSI is ready
  while((RCC->CR & RCC_CR_HSIRDY) == 0);

  // Configure the Flash latency and enable prefetch buffer
  FLASH->ACR |= FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE;

  // Configure the AHB and APB bus clocks
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;   // HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_PPRE_DIV1;  // PCLK1 = HCLK

  // Select HSI as the system clock source
  RCC->CFGR &= ~RCC_CFGR_SW;          // Clear SW bits
  RCC->CFGR |= RCC_CFGR_SW_HSI;       // Set HSI as system clock

  // Wait till HSI is used as the system clock source
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

  // Configure the USART1 clock source to PCLK1
  RCC->CIR &= ~RCC_CFGR3_USART1SW;  // Clear USART1 clock source selection bits
  RCC->CIR |= RCC_CFGR3_USART1SW_0; // Set PCLK1 as USART1 clock source
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */