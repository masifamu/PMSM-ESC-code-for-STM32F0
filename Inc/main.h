/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define throttlePin_Pin GPIO_PIN_0
#define throttlePin_GPIO_Port GPIOA
#define battVoltagePin_Pin GPIO_PIN_1
#define battVoltagePin_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_2
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_3
#define RX_GPIO_Port GPIOA
#define currentPin_Pin GPIO_PIN_4
#define currentPin_GPIO_Port GPIOA
#define heatSinkVoltage_Pin GPIO_PIN_5
#define heatSinkVoltage_GPIO_Port GPIOA
#define BH_Pin GPIO_PIN_8
#define BH_GPIO_Port GPIOA
#define GH_Pin GPIO_PIN_9
#define GH_GPIO_Port GPIOA
#define YH_Pin GPIO_PIN_10
#define YH_GPIO_Port GPIOA
#define ledB_Pin GPIO_PIN_12
#define ledB_GPIO_Port GPIOA
#define F_R_Pin GPIO_PIN_15
#define F_R_GPIO_Port GPIOA
#define ledY_Pin GPIO_PIN_3
#define ledY_GPIO_Port GPIOB
#define ledG_Pin GPIO_PIN_4
#define ledG_GPIO_Port GPIOB
#define hsB_Pin GPIO_PIN_5
#define hsB_GPIO_Port GPIOB
#define hsB_EXTI_IRQn EXTI4_15_IRQn
#define hsG_Pin GPIO_PIN_6
#define hsG_GPIO_Port GPIOB
#define hsG_EXTI_IRQn EXTI4_15_IRQn
#define hsY_Pin GPIO_PIN_7
#define hsY_GPIO_Port GPIOB
#define hsY_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
