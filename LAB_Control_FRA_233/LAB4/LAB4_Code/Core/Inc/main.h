/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define Joy_rl_Pin GPIO_PIN_1
#define Joy_rl_GPIO_Port GPIOC
#define Lead3_Pin GPIO_PIN_2
#define Lead3_GPIO_Port GPIOC
#define PWM_Dir_Pin GPIO_PIN_3
#define PWM_Dir_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Grip1_Pin GPIO_PIN_4
#define Grip1_GPIO_Port GPIOC
#define Grip2_Pin GPIO_PIN_5
#define Grip2_GPIO_Port GPIOC
#define Switch_Mode_Pin GPIO_PIN_0
#define Switch_Mode_GPIO_Port GPIOB
#define Joy_rr_Pin GPIO_PIN_1
#define Joy_rr_GPIO_Port GPIOB
#define Joy_Blue_Pin GPIO_PIN_2
#define Joy_Blue_GPIO_Port GPIOB
#define Lead2_Pin GPIO_PIN_12
#define Lead2_GPIO_Port GPIOB
#define Lead1_Pin GPIO_PIN_6
#define Lead1_GPIO_Port GPIOC
#define light1_Pin GPIO_PIN_8
#define light1_GPIO_Port GPIOC
#define light2_Pin GPIO_PIN_9
#define light2_GPIO_Port GPIOC
#define Joy_red_Pin GPIO_PIN_8
#define Joy_red_GPIO_Port GPIOA
#define Joy_Black_Pin GPIO_PIN_9
#define Joy_Black_GPIO_Port GPIOA
#define Prox_Pin GPIO_PIN_10
#define Prox_GPIO_Port GPIOA
#define Prox_EXTI_IRQn EXTI15_10_IRQn
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define Joy_Mode_Pin GPIO_PIN_15
#define Joy_Mode_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define Joy_rlB6_Pin GPIO_PIN_6
#define Joy_rlB6_GPIO_Port GPIOB
#define Joy_soft_stop_Pin GPIO_PIN_7
#define Joy_soft_stop_GPIO_Port GPIOB
#define Joy_Yellow_Pin GPIO_PIN_9
#define Joy_Yellow_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
