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

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
typedef union 
{
	struct
	{
		unsigned int data_type		: 3; 
		unsigned int module_type	: 5; //тип модуля отправителя
		unsigned int 				: 4;
		unsigned int state			: 4; //код состояния UPS
	};

	uint8_t bytes[ 8 ];

}CAN_MSG_TYPE_C_MKIP ;

typedef struct
{
	uint8_t flag_RX;
	uint8_t RxData[8];
}
CAN_RX_msg;

typedef enum { RX_NONE = 0, RX_C1, RX_OWN_C2, RX_UNKNOWN}  TRxResult; //статусы полученных сообщений CAN

#define CAN_MSG_TYPE_A1_ID	0x01
#define CAN_MSG_TYPE_B_ID	0x08
#define CAN_MSG_TYPE_C_ID	0x10
#define CAN_MSG_TYPE_D_ID	0x20

#define MY_MODULE_TYPE 0x15	// Код типа модуля - МКИП

#define MAKE_FRAME_ID( msg_type_id, board_addr) ((((uint32_t)msg_type_id) << 5) | (board_addr))
#define GET_MODULE_ADDR( frame_id) (((frame_id) >> 18) & 0x1F)
#define GET_MSG_TYPE( frame_id) (((frame_id) >> 23) & 0x3F)

//extern uint32_t ID_C2;
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
static TRxResult ReadMsgCAN(void);
uint32_t Send_Message_C2 ();
uint32_t CAN1_Send_Message (CAN_TxHeaderTypeDef * , uint8_t * );
static void Task_ProcCANRequests_And_CheckCANCondition( void );
void TaskCAN( void );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

