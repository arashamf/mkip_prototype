
#ifndef __MEGATEC_H__
#define __MEGATEC_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
 	SM_WAIT_PARENTHESIS = 0, //ожидание символа '('
	SM_GET_DATA //получение данных

}TUPS_PROTOCOL_STATES;

typedef struct 
{
	uint32_t Count; //счётчик полученных слов
	char *ptr_Items[25]; //указатели на принятые слова

}TUPS_PROTOCOL_ITEMS;

//struct TUPS_PROTOCOL;
typedef struct TUPS_PROTOCOL* TUPS_PROTOCOL_HANDLE;

//extern TUPS_PROTOCOL_ITEMS ups_msg_items;
extern TMyFlags g_MyFlags;
//extern const char status[];

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TaskCommUPS( void );
uint8_t UPS_PROTOCOL_Process( TUPS_PROTOCOL_HANDLE Handle, char smb, TUPS_PROTOCOL_ITEMS *Items );
TUPS_PROTOCOL_HANDLE UPS_PROTOCOL_Create( void );
void UPS_PROTOCOL_Destroy(TUPS_PROTOCOL_HANDLE Handle);
void UPS_PROTOCOL_Reset( TUPS_PROTOCOL_HANDLE );

#endif /* __MEGATEC_H__ */

