/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define ON 1
#define OFF 0
#define LED_GREEN(x) ((x)? (SET_BIT (GPIOA->BSRR, GPIO_BSRR_BS2 )) : (SET_BIT (GPIOA->BSRR, GPIO_BSRR_BR2)))
#define TOOGLE_LED_GREEN ((READ_BIT (GPIOA->IDR, GPIO_IDR_IDR2)) ? (LED_RED(0)) : (LED_RED(1)))

#define LED_RED(x) ((x)? (SET_BIT (GPIOA->BSRR, GPIO_BSRR_BS3 )) : (SET_BIT (GPIOA->BSRR, GPIO_BSRR_BR3)))
#define TOOGLE_LED_RED ((READ_BIT (GPIOA->IDR, GPIO_IDR_IDR3)) ? (LED_RED(0)) : (LED_RED(1)))
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

