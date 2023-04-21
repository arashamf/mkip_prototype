
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

//Includes -------------------------------------------------------------------------------------------//
#include "main.h"

//Exported types --------------------------------------------------------------------------------------//
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

//Private variables -----------------------------------------------------------------------------------//
extern CAN_HandleTypeDef hcan;

//Private defines--------------------------------------------------------------------------------------//
#define MY_CAN	((CAN_TypeDef *)CAN1_BASE)

#define CAN_RX_PIN		GPIO_PIN_11
#define CAN_RX_PORT		GPIOA
#define CAN_TX_PIN		GPIO_PIN_12
#define CAN_TX_PORT		GPIOA

#define CAN_MSG_TYPE_A1_ID	0x01
#define CAN_MSG_TYPE_B_ID	0x08
#define CAN_MSG_TYPE_C_ID	0x10
#define CAN_MSG_TYPE_D_ID	0x20

#define MY_MODULE_TYPE 0x15	// Код типа модуля - МКИП

//Macro -----------------------------------------------------------------------------------------------//
#define MAKE_FRAME_ID( msg_type_id, board_addr) ((((uint32_t)msg_type_id) << 5) | (board_addr))
#define GET_MODULE_ADDR( frame_id) ((frame_id) & 0x1F) //адрес модуля (5 бит) из заголовка CAN сообщения
#define GET_MSG_TYPE( frame_id) (((frame_id) >> 5) & 0x3F) //тип сообщения (6 бит) из заголовка CAN сообщения

//Prototypes-------------------------------------------------------------------------------------------//
void init_CAN (void);
void MX_CAN_Init(void);
void HAL_CAN_MspInit(CAN_HandleTypeDef* );
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* );
static TRxResult ReadMsgCAN(void);
uint32_t Send_Message_C2 ();
uint32_t CAN1_Send_Message (CAN_TxHeaderTypeDef * , uint8_t * );
static void Task_ProcCANRequests_And_CheckCANCondition( void );
void TaskCAN( void );
//-----------------------------------------------------------------------------------------------------//

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

