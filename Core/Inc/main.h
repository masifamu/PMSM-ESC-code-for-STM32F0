/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define punchThrottlePin_Pin GPIO_PIN_0
#define punchThrottlePin_GPIO_Port GPIOA
#define punchBattVoltagePin_Pin GPIO_PIN_1
#define punchBattVoltagePin_GPIO_Port GPIOA
#define punchCurrentPin_Pin GPIO_PIN_4
#define punchCurrentPin_GPIO_Port GPIOA
#define punchHeatSinkTemp_Pin GPIO_PIN_5
#define punchHeatSinkTemp_GPIO_Port GPIOA
#define punchBreak_Pin GPIO_PIN_6
#define punchBreak_GPIO_Port GPIOA
#define punchBH_Pin GPIO_PIN_8
#define punchBH_GPIO_Port GPIOA
#define punchGH_Pin GPIO_PIN_9
#define punchGH_GPIO_Port GPIOA
#define punchYH_Pin GPIO_PIN_10
#define punchYH_GPIO_Port GPIOA
#define punchLedB_Pin GPIO_PIN_12
#define punchLedB_GPIO_Port GPIOA
#define punchF_R_Pin GPIO_PIN_15
#define punchF_R_GPIO_Port GPIOA
#define punchLedY_Pin GPIO_PIN_3
#define punchLedY_GPIO_Port GPIOB
#define punchLedG_Pin GPIO_PIN_4
#define punchLedG_GPIO_Port GPIOB
#define punchHSB_Pin GPIO_PIN_5
#define punchHSB_GPIO_Port GPIOB
#define punchHSG_Pin GPIO_PIN_6
#define punchHSG_GPIO_Port GPIOB
#define punchHSY_Pin GPIO_PIN_7
#define punchHSY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
