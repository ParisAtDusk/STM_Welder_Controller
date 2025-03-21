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
#define GAS_Pin GPIO_PIN_13
#define GAS_GPIO_Port GPIOC
#define IGNITION_Pin GPIO_PIN_14
#define IGNITION_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOC
#define PEDAL_Pin GPIO_PIN_0
#define PEDAL_GPIO_Port GPIOA
#define CURR_SENSE_Pin GPIO_PIN_1
#define CURR_SENSE_GPIO_Port GPIOA
#define LED_COL_0_Pin GPIO_PIN_12
#define LED_COL_0_GPIO_Port GPIOB
#define LED_COL_1_Pin GPIO_PIN_13
#define LED_COL_1_GPIO_Port GPIOB
#define LED_COL_2_Pin GPIO_PIN_14
#define LED_COL_2_GPIO_Port GPIOB
#define LED_ROW_0_Pin GPIO_PIN_15
#define LED_ROW_0_GPIO_Port GPIOB
#define LED_ROW_1_Pin GPIO_PIN_8
#define LED_ROW_1_GPIO_Port GPIOA
#define ENC0_Pin GPIO_PIN_15
#define ENC0_GPIO_Port GPIOA
#define ENC1_Pin GPIO_PIN_3
#define ENC1_GPIO_Port GPIOB
#define ENC_BUTTON_Pin GPIO_PIN_4
#define ENC_BUTTON_GPIO_Port GPIOB
#define ENC_BUTTON_EXTI_IRQn EXTI4_IRQn
#define TORCH_BUTTON_Pin GPIO_PIN_5
#define TORCH_BUTTON_GPIO_Port GPIOB
#define TORCH_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define PWM_CURR_Pin GPIO_PIN_8
#define PWM_CURR_GPIO_Port GPIOB
#define H_BRIDGE_Pin GPIO_PIN_9
#define H_BRIDGE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
 
extern I2C_HandleTypeDef hi2c1;

// extern SPI_HandleTypeDef hspi1;
 
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
