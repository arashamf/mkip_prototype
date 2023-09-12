
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
		unsigned int module_type	: 5; //тип модул€ отправител€
		unsigned int 				: 4;
		unsigned int state			: 4; //код состо€ни€ UPS
	};

	uint8_t bytes[ 8 ];

}CAN_MSG_TYPE_C_MKIP ; //структура сообщени€ типа —

typedef struct
{
	uint8_t flag_RX;
	uint8_t RxData[8];
}
CAN_RX_msg; //структура с данными прин€того кадра CAN

typedef enum { RX_NONE = 0, RX_C1, RX_OWN_C2, RX_UNKNOWN}  TRxResult; //статусы полученных сообщений CAN

//Private variables -----------------------------------------------------------------------------------//
extern CAN_HandleTypeDef hcan;

//Private defines--------------------------------------------------------------------------------------//
#define MSG_TYPE_A0                                  0x000 // 000000
#define MSG_TYPE_A1                                  0x001 // 000001
#define MSG_TYPE_A2                                  0x002 // 000010
#define MSG_TYPE_A3                                  0x003 // 000011
#define MSG_TYPE_B                                   0x008 // 001000
#define MSG_TYPE_C                                   0x010 // 010000
#define MSG_TYPE_D                                   0x020 // 100000

#define MODULE_TYPE_MKIP                             0x15	//  од типа модул€ - ћ »ѕ

#define CAN_OK 				0
#define CAN_ERROR 	1

//Macro -----------------------------------------------------------------------------------------------//
#define MAKE_FRAME_ID( msg_type_id, board_addr) ((((uint32_t)msg_type_id) << 5) | (board_addr))
#define MAKE_MSG_DATA0(module_id, data_type) ( (module_id << 3) | data_type ) //старшие 5 бит 1 байта сообщени€ тип модул€-отправител€ или тип модул€ получател€, младшие 3 бита - код вида данных
#define GET_MODULE_ADDR( frame_id) ((frame_id) & 0x1F) //адрес модул€ (5 бит) из заголовка CAN сообщени€
#define GET_MSG_TYPE( frame_id) (((frame_id) >> 5) & 0x3F) //тип сообщени€ (6 бит) из заголовка CAN сообщени€

//Prototypes-------------------------------------------------------------------------------------------//
void init_CAN (void);
void MX_CAN_Init(void);
void HAL_CAN_MspInit(CAN_HandleTypeDef* );
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* );
void CAN_Reinit (void);
void TaskCAN(void);
//-----------------------------------------------------------------------------------------------------//

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

