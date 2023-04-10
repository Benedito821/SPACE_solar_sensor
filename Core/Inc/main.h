/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define I2C_EN_Pin GPIO_PIN_14
#define I2C_EN_GPIO_Port GPIOC
#define ADC_Rad_Pin GPIO_PIN_4
#define ADC_Rad_GPIO_Port GPIOA
#define ISence1_Pin GPIO_PIN_5
#define ISence1_GPIO_Port GPIOA
#define ISence2_Pin GPIO_PIN_6
#define ISence2_GPIO_Port GPIOA
#define ADC_Ilum_Pin GPIO_PIN_0
#define ADC_Ilum_GPIO_Port GPIOB
#define Coil_Up_Pin GPIO_PIN_8
#define Coil_Up_GPIO_Port GPIOA
#define Dwn_In_Pin GPIO_PIN_9
#define Dwn_In_GPIO_Port GPIOA
#define Up_in_Pin GPIO_PIN_10
#define Up_in_GPIO_Port GPIOA
#define Coil_Dwn_Pin GPIO_PIN_15
#define Coil_Dwn_GPIO_Port GPIOA
#define NRST_Pin GPIO_PIN_3
#define NRST_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
