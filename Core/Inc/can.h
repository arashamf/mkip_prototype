/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "megatec.h"
#include "usart.h"
#include "lcd1602.h"
#include "stdio.h"
#include "gpio.h"
#include "lib_delay.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define ID_C1 0b010000
#define ID_C1_mask 0b01000000000
#define ID_C2 0b010000
#define ID_adress 0b01010
#define ID_adress_mask 0b00000001010

#define CAN_MSG_TYPE_A1_ID	0x01
#define CAN_MSG_TYPE_B_ID	0x08
#define CAN_MSG_TYPE_C_ID	0x10
#define CAN_MSG_TYPE_D_ID	0x20

#define MY_MODULE_TYPE 0x15	// Код типа модуля - МКИП

typedef struct 
{
	uint8_t flag_RX; //флаг принятого сообщения
	uint8_t RxData [8]; //буффер принятого сообщения по CAN

}CAN_RX_msg;
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void Send_Message_C2 ( TUPS_PROTOCOL_ITEMS *ups_msg_items);
void CAN1_Send_Message (CAN_TxHeaderTypeDef * , uint8_t * );
static void Task_ProcCANRequests_And_CheckCANCondition( void );
void TaskCAN( void );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

