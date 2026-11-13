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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_4
#define COL2_GPIO_Port GPIOC
#define COL2_EXTI_IRQn EXTI4_IRQn
#define SEGA_Pin GPIO_PIN_5
#define SEGA_GPIO_Port GPIOC
#define SEGB_Pin GPIO_PIN_1
#define SEGB_GPIO_Port GPIOB
#define SEGC_Pin GPIO_PIN_2
#define SEGC_GPIO_Port GPIOB
#define DIGIT1_Pin GPIO_PIN_10
#define DIGIT1_GPIO_Port GPIOB
#define DIGIT2_Pin GPIO_PIN_12
#define DIGIT2_GPIO_Port GPIOB
#define SEGD_Pin GPIO_PIN_13
#define SEGD_GPIO_Port GPIOB
#define SEGE_Pin GPIO_PIN_14
#define SEGE_GPIO_Port GPIOB
#define COL3_Pin GPIO_PIN_15
#define COL3_GPIO_Port GPIOB
#define COL3_EXTI_IRQn EXTI15_10_IRQn
#define COL1_Pin GPIO_PIN_7
#define COL1_GPIO_Port GPIOC
#define COL1_EXTI_IRQn EXTI9_5_IRQn
#define SEGF_Pin GPIO_PIN_8
#define SEGF_GPIO_Port GPIOC
#define COL4_Pin GPIO_PIN_9
#define COL4_GPIO_Port GPIOC
#define COL4_EXTI_IRQn EXTI9_5_IRQn
#define DIGIT4_Pin GPIO_PIN_8
#define DIGIT4_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_9
#define ROW1_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_10
#define ROW2_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_11
#define ROW3_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_12
#define ROW4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SEGG_Pin GPIO_PIN_10
#define SEGG_GPIO_Port GPIOC
#define SEGDP_Pin GPIO_PIN_11
#define SEGDP_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DIGIT3_Pin GPIO_PIN_4
#define DIGIT3_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
