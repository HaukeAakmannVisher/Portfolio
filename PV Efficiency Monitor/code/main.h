/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Active_load_Pin GPIO_PIN_0
#define Active_load_GPIO_Port GPIOA
#define Sol1_Pin GPIO_PIN_1
#define Sol1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Sol2_Pin GPIO_PIN_4
#define Sol2_GPIO_Port GPIOA
#define B_Button_Pin GPIO_PIN_5
#define B_Button_GPIO_Port GPIOA
#define B_Button_EXTI_IRQn EXTI9_5_IRQn
#define M_Button_Pin GPIO_PIN_7
#define M_Button_GPIO_Port GPIOA
#define M_Button_EXTI_IRQn EXTI9_5_IRQn
#define Light_sensor_Pin GPIO_PIN_4
#define Light_sensor_GPIO_Port GPIOC
#define Analog_temp_Pin GPIO_PIN_5
#define Analog_temp_GPIO_Port GPIOC
#define D4_Pin GPIO_PIN_1
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_2
#define D5_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_12
#define D6_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_13
#define RS_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_14
#define D7_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_15
#define E_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_10
#define LED4_GPIO_Port GPIOA
#define Digital_temp_Pin GPIO_PIN_12
#define Digital_temp_GPIO_Port GPIOA
#define Digital_temp_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB
#define R_Button_Pin GPIO_PIN_6
#define R_Button_GPIO_Port GPIOB
#define R_Button_EXTI_IRQn EXTI9_5_IRQn
#define L_Button_Pin GPIO_PIN_8
#define L_Button_GPIO_Port GPIOB
#define L_Button_EXTI_IRQn EXTI9_5_IRQn
#define T_Button_Pin GPIO_PIN_9
#define T_Button_GPIO_Port GPIOB
#define T_Button_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
