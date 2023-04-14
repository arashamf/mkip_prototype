/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

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
typedef union _MY_FLAGS
{
	unsigned int Value;
	struct //������� ����
	{
		unsigned CAN_Fail				: 1;	//��� unsigned, ����� ���� 1 ���, ������ CAN	( ��� ������ ����������� ��������� C2 )
		unsigned UPS_state			: 4; //��� unsigned, ����� ���� 4 ����, ������ RS-232
	};
}TMyFlags;

extern TMyFlags g_MyFlags ; //������������� �������� ���� (UPS_NO_LINK , CAN_Fail == 1) 
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin LL_GPIO_PIN_2
#define LED1_GPIO_Port GPIOA
#define LED2_Pin LL_GPIO_PIN_3
#define LED2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define UPS_OK 				0
#define UPS_NO_LINK 	1
#define UPS_BAT_MODE 	2
#define UPS_LOW_BAT 	3
#define UPS_FAULT 		8
#define UPS_BAT_FAULT 9

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */