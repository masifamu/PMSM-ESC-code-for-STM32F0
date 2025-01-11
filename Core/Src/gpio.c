/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "board_config_V1.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void initGPIO(void){
	//1. Enable Periph clock
	RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN);

	//********************************************************LEDs***************************
	//2. Configure GPIOA-pin and GPIOB-pin default output level
	GPIO_CLEAR_PIN(GPIOA, GPIO_PIN_12);
	GPIO_CLEAR_PIN(GPIOB, GPIO_PIN_3);
	GPIO_CLEAR_PIN(GPIOB, GPIO_PIN_4);

	//3. Configure mode for A12 pins, PB3,PB4
	//(00: Input, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode)
	GPIOA->MODER &= ~GPIO_MODER_MODER12; //reset first
	GPIOA->MODER |= GPIO_MODER_MODER12_0;

	GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4);
	GPIOB->MODER |= (GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0);

	//4. Set output type to push-pull (0: Output push-pull, 1: Output open-drain)
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_12;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_3 | GPIO_OTYPER_OT_4);

	//5. Set pin speed to low (00: Low speed, 01: Medium speed, 10: High speed, 11: Very high speed)
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR12;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_0;

	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4);
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3_0 | GPIO_OSPEEDER_OSPEEDR4_0);

	//6. Set no pull-up/pull-down (00: No pull-up, pull-down, 01: Pull-up, 10: Pull-down, 11: Reserved)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR12;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12 | GPIO_PUPDR_PUPDR12);

	//********************************************************HALL SENSOR***************************
	//    Enable the clock for the SYSCFG peripheral.
	//    Enable the clock for the GPIO port(s) used by the EXTI lines.
	//    Configure the GPIO pins as input or alternate function.
	//    Enable the interrupt in the EXTI IMR (Interrupt Mask Register).
	//    Configure the EXTI trigger (rising/falling edge, or both).
	//    Configure the EXTI lines in the SYSCFG controller.
	//    Enable the interrupt in the NVIC (Nested Vectored Interrupt Controller).

	//1. Enable Periph clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

	//3. Configure mode for PB5,PB6,PB7
	//(00: Input, 01: General purpose output mode, 10: Alternate function mode, 11: Analog mode)
	GPIOB->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7); //reset first

	//6. Set no pull-up/pull-down (00: No pull-up, pull-down, 01: Pull-up, 10: Pull-down, 11: Reserved)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR5_0 | GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR7_0);

	// Enable EXTI lines interrupt at rising edge for PB5, PB6, and PB7
	EXTI->IMR |= (EXTI_IMR_IM5 | EXTI_IMR_IM6 | EXTI_IMR_IM7);
	EXTI->RTSR |= (EXTI_RTSR_RT5 | EXTI_RTSR_RT6 | EXTI_RTSR_RT7);

	SYSCFG->EXTICR[1] &= ~(SYSCFG_EXTICR2_EXTI5 | SYSCFG_EXTICR2_EXTI6 | SYSCFG_EXTICR2_EXTI7);
	SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI5_PB | SYSCFG_EXTICR2_EXTI6_PB | SYSCFG_EXTICR2_EXTI7_PB);

	// Enable EXTI interrupt in NVIC
	NVIC_EnableIRQ(EXTI4_15_IRQn);

}
// This function will be called at every hall sensor trigger.
void EXTI4_15_IRQHandler(void)
{
    // Check which EXTI line triggered the interrupt and clear the flag
    if (EXTI->PR & (1 << 5)) {
        EXTI->PR = (1 << 5); // Clear interrupt pending bit for PB5
        // Handle interrupt for PB5
    }
    if (EXTI->PR & (1 << 6)) {
        EXTI->PR = (1 << 6); // Clear interrupt pending bit for PB6
        // Handle interrupt for PB6
    }
    if (EXTI->PR & (1 << 7)) {
        EXTI->PR = (1 << 7); // Clear interrupt pending bit for PB7
        // Handle interrupt for PB7
    }
    GPIO_TOGGLE_PIN(GPIOB, YELLOW_LED);
}
/* USER CODE END 2 */
