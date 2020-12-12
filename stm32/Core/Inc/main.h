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
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_dma.h"

#include "stm32f0xx_ll_exti.h"

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
#define HMI_ERROR_EXT_Pin GPIO_PIN_14
#define HMI_ERROR_EXT_GPIO_Port GPIOC
#define HMI_STATUS_EXT_Pin GPIO_PIN_15
#define HMI_STATUS_EXT_GPIO_Port GPIOC
#define RADIO_DIO1_Pin GPIO_PIN_2
#define RADIO_DIO1_GPIO_Port GPIOA
#define RADIO_DIO1_EXTI_IRQn EXTI2_3_IRQn
#define RADIO_BUSY_Pin GPIO_PIN_3
#define RADIO_BUSY_GPIO_Port GPIOA
#define RADIO_NRST_Pin GPIO_PIN_4
#define RADIO_NRST_GPIO_Port GPIOA
#define RADIO_DIO2_Pin GPIO_PIN_0
#define RADIO_DIO2_GPIO_Port GPIOB
#define RADIO_TXEN_Pin GPIO_PIN_1
#define RADIO_TXEN_GPIO_Port GPIOB
#define RADIO_RXEN_Pin GPIO_PIN_2
#define RADIO_RXEN_GPIO_Port GPIOB
#define RADIO_NSS_Pin GPIO_PIN_12
#define RADIO_NSS_GPIO_Port GPIOB
#define HMI_ERROR_Pin GPIO_PIN_8
#define HMI_ERROR_GPIO_Port GPIOB
#define HMI_STATUS_Pin GPIO_PIN_9
#define HMI_STATUS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
