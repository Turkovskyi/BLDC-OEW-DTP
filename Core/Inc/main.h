/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define SD_A_3_Pin GPIO_PIN_5
#define SD_A_3_GPIO_Port GPIOA
#define SD_B_3_Pin GPIO_PIN_6
#define SD_B_3_GPIO_Port GPIOA
#define SD_C_3_Pin GPIO_PIN_7
#define SD_C_3_GPIO_Port GPIOA
#define SD_B_1_Pin GPIO_PIN_0
#define SD_B_1_GPIO_Port GPIOB
#define SD_C_1_Pin GPIO_PIN_1
#define SD_C_1_GPIO_Port GPIOB
#define SD_A_1_Pin GPIO_PIN_2
#define SD_A_1_GPIO_Port GPIOB
#define Toggle_pin_Pin GPIO_PIN_10
#define Toggle_pin_GPIO_Port GPIOB
#define SD_A_2_Pin GPIO_PIN_13
#define SD_A_2_GPIO_Port GPIOB
#define SD_B_2_Pin GPIO_PIN_14
#define SD_B_2_GPIO_Port GPIOB
#define SD_C_2_Pin GPIO_PIN_15
#define SD_C_2_GPIO_Port GPIOB
#define SD_A_4_Pin GPIO_PIN_8
#define SD_A_4_GPIO_Port GPIOA
#define SD_B_4_Pin GPIO_PIN_11
#define SD_B_4_GPIO_Port GPIOA
#define SD_C_4_Pin GPIO_PIN_12
#define SD_C_4_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define HS4_Pin GPIO_PIN_10
#define HS4_GPIO_Port GPIOC
#define HS5_Pin GPIO_PIN_11
#define HS5_GPIO_Port GPIOC
#define HS6_Pin GPIO_PIN_12
#define HS6_GPIO_Port GPIOC
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define HS1_Pin GPIO_PIN_4
#define HS1_GPIO_Port GPIOB
#define HS2_Pin GPIO_PIN_5
#define HS2_GPIO_Port GPIOB
#define HS3_Pin GPIO_PIN_6
#define HS3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
