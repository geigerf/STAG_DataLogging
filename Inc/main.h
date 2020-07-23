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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DATA_SIZE 1024

#define SDRAM_FRAME_OFFSET  ((uint32_t)0x0800)
  
#define SDRAM_BANK_ADDR                 ((uint32_t)0xC0000000)

#define SDRAM_TIMEOUT     ((uint32_t)0xFFFF)

#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_13
#define LED_RED_GPIO_Port GPIOJ
#define Dmux_sel1_Pin GPIO_PIN_8
#define Dmux_sel1_GPIO_Port GPIOC
#define Mux_sel0_Pin GPIO_PIN_7
#define Mux_sel0_GPIO_Port GPIOC
#define Mux_sel1_Pin GPIO_PIN_6
#define Mux_sel1_GPIO_Port GPIOC
#define Dmux_sel2_Pin GPIO_PIN_7
#define Dmux_sel2_GPIO_Port GPIOF
#define Mux_sel3_Pin GPIO_PIN_6
#define Mux_sel3_GPIO_Port GPIOF
#define Dmux_EN_Pin GPIO_PIN_4
#define Dmux_EN_GPIO_Port GPIOJ
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOJ
#define BUTTON_Pin GPIO_PIN_0
#define BUTTON_GPIO_Port GPIOA
#define BUTTON_EXTI_IRQn EXTI0_IRQn
#define Dmux_sel3_Pin GPIO_PIN_3
#define Dmux_sel3_GPIO_Port GPIOJ
#define P_EN_Pin GPIO_PIN_6
#define P_EN_GPIO_Port GPIOH
#define Dmux_sel0_Pin GPIO_PIN_0
#define Dmux_sel0_GPIO_Port GPIOJ
#define Mux_sel2_Pin GPIO_PIN_1
#define Mux_sel2_GPIO_Port GPIOJ
/* USER CODE BEGIN Private defines */
// Uncomment the one that corresponds to the physical readout circuit
// So far there is a readout board with 3k and one with 1k feedback resistor
//#define FB_RES_1K
#define FB_RES_3K
// Uncomment if you want to send the pressure values as float instead of uint16_t
//#define FLOAT_Pressure
// Uncomment if you want to test how well the threshold works for detecting object contacts
#define TEST_THRESH
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
