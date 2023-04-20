
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "megatec.h"
#include "usart.h"
//#include "can.h"
#include "lib_delay.h"

#define MAX_UPS_PROTOCOL_HANDLES 1 //максимальное количество создаваемых структур
#define UPS_PROTOCOL_BUFFER_SIZE 200 //размер буффера

static enum //режимы обмена с UPS
{
	SM_HOME = 0,
	SM_WAIT_RESPONCE, 
	SM_CHECK_RESPONCE,
	SM_SHORT_DELAY,
	SM_INIT
} state = SM_INIT; //начальная стадия - инициализация

static enum //статус полученного сообщения от UPS
{
	NO_GET_MSG = 0,
	GET_MSG, 
	ECHO_MSG
} result = NO_GET_MSG; 

struct TUPS_PROTOCOL
{
	TUPS_PROTOCOL_STATES StateMachine;	//режим обмена с UPS
  unsigned int index; //счётчик полученных символов
  unsigned int items_count; //счётчик полученных слов

	char Buffer[ UPS_PROTOCOL_BUFFER_SIZE ]; //буффер принятого сообщения

	TUPS_PROTOCOL_ITEMS Items; //структура с принятыми словами
};

static struct TUPS_PROTOCOL m_UPS_PROTOCOL_Instances[ MAX_UPS_PROTOCOL_HANDLES ]; //массив структур TUPS_PROTOCOL для соединения с UPS
static bool m_Handle_In_Use[ MAX_UPS_PROTOCOL_HANDLES ];

char *msg_flags;
const char status[] = {"Q1\r"};
//----------------------------------------------------------------------------------------------------//

//----------------------------------------------------------------------------------------------------//
void TaskCommUPS( void )
{
	static uint32_t ticks; //тип static 
	static TUPS_PROTOCOL_HANDLE ups_h; //указатель на структуру TUPS_PROTOCOL
	static TUPS_PROTOCOL_ITEMS ups_msg_items; //инициализация структуры TUPS_PROTOCOL_ITEMS
	
	switch( state )
	{
		case SM_HOME:			//стадия начала обмена с UPSом
			check_status_UPS(); //отправка запроса Q1 UPSу
			ticks = HAL_GetTick(); //получение текущего количества тиков
			state = SM_WAIT_RESPONCE; //переход в режим ожидания получения ответа от UPSа
			break;

		case SM_WAIT_RESPONCE: //стадия ожидания получения ответа от UPSа			
			if( HAL_GetTick() - ticks > (TICKS_PER_SECOND  * 4) ) //если нет ответа от UPSа больше 4 с
			{
				g_MyFlags.UPS_state = UPS_NO_LINK; //установка статуса отсутствия линка с UPSом
				ticks = HAL_GetTick(); //получение текущего количества тиков
				state = SM_SHORT_DELAY; //тайм-аут на 3 с
			}
			else 
			{
				if((result = UPS_PROTOCOL_Process(ups_h, UartGetc(ups_h->index), &ups_msg_items)) == GET_MSG ) //если сообщение от UPSа успешно получено 
					{state = SM_CHECK_RESPONCE;} //установка режима проверки ответа
				else
				{
					if(result == ECHO_MSG) //если получено эхо-ответ
					{
						result = NO_GET_MSG;
						check_status_UPS();
					}
				}
			}
		break;

		case SM_CHECK_RESPONCE: //стадия проверки полученного сообщения			
			msg_flags = ups_msg_items.ptr_Items[ 7 ]; //указатель на восьмое слово
		
			if( ups_msg_items.Count == 8 && strlen( msg_flags ) == 8 ) //если принятых слов 8 и в восьмом слове 8 символов
			{
				g_MyFlags.UPS_state = UPS_OK; //статус UPS_OK
				//---msg_flags[0] сответствует bit№7 протокола мегатек---//
				if( msg_flags[6] != '1' ) //1: ИБП выключен, 0: ИБП включён
				{
					if( msg_flags[0] == '1' ) //1: работа от АКБ, 0: работа от сети
						g_MyFlags.UPS_state = UPS_BAT_MODE;
					else if( msg_flags[1] == '1' ) //1: низкий уровень заряда АКБ, 0: АКБ в норме
						g_MyFlags.UPS_state = UPS_LOW_BAT;
					else if( msg_flags[3] == '1' ) // 1: авария ИБП, 0: ИБП в норме
						g_MyFlags.UPS_state = UPS_FAULT;
				}
			}
			result = NO_GET_MSG; 
			ticks = HAL_GetTick(); //получение текущего количества тиков
			state = SM_SHORT_DELAY; //тайм-аут на 3 с
		break;

		case SM_SHORT_DELAY: //стадия тайм-аута					
			if((HAL_GetTick() - ticks) > (TICKS_PER_SECOND*3)) //если режим тайм-аута длится 3 с и более
			{	
				state = SM_HOME; //установка стадии начала обмена с UPSом 
			}
		break;

		case SM_INIT:		//стадия инициализации
			UartRxClear();
			ups_h = UPS_PROTOCOL_Create(); //получение адреса структуры соединения
			if( !ups_h )
				g_MyFlags.UPS_state = UPS_NO_LINK;
			else
				state = SM_HOME;
			break;
			
		default:	
			state = SM_HOME;
		
		break;
	}
}
	
//----------------------------------------------------------------------------------------------------//
uint8_t UPS_PROTOCOL_Process( TUPS_PROTOCOL_HANDLE Handle, char smb, TUPS_PROTOCOL_ITEMS *Items )
//----------------------------------------------------------------------------------------------------//
{
  unsigned int index; //счётчик полученных символов
  unsigned int items_count; //счётчик полученных слов

  index = Handle->index; //получение счётчика полученных символов
	items_count = Handle->items_count; //получение счётчика полученных слов

	switch( Handle->StateMachine ) //проверка состояния
	{
		case SM_WAIT_PARENTHESIS: // стадия ожидания получения ответа от UPSа		

			if( smb == '(' ) //если получен символ '('
			{
				index = 0; //счётчик полученных символов
				items_count = 0;  //счётчик полученных слов
				Handle->Items.ptr_Items[0] = Handle->Buffer; //копирование указателя на 1 слово
				Handle->StateMachine = SM_GET_DATA; //перевод в режим получения данных
			}
			else
			{
				if (smb == 'Q') //если UPS вернул начало посланого запроса Q1
				{
					result = ECHO_MSG; //получен эхо ответ
					return result;
				}
			}
		  break;

		case SM_GET_DATA: // стадия получения ответа от UPSа

			if( smb == '\r' ) //если сообщение окончено
			{
				items_count++; //увеличение счётчика полученных слов
				Handle->Buffer[index++] = 0; //запись '\0' вместо '\r'
				memcpy((void *)Items, (void *)&Handle->Items, sizeof(TUPS_PROTOCOL_ITEMS)); //копирование указателя на полученную структуру ups_h в указатель ups_msg_items
				Items->Count = items_count; //сохранение счётчика полученных слов в указатель ups_msg_items
				Handle->StateMachine = SM_WAIT_PARENTHESIS; //установка статуса ожидания символа '('
				result = GET_MSG; //сообщение получено успешно
			}
			else
			{
				if( index < (sizeof (Handle->Buffer) - 1) ) //проверка размера сообщения
				{
					if(smb == ' ') //если слово окончено
					{
            if(items_count < (sizeof(Handle->Items.ptr_Items) / sizeof(Handle->Items.ptr_Items[0]))) //если счётчик слов не превысил максимальное количество слов в буффере
						{
            	Handle->Items.ptr_Items[ ++items_count ] = &Handle->Buffer[index + 1]; //сохранение указателя на начало следующего слова
						}
						else
						{							
							Handle->StateMachine = SM_WAIT_PARENTHESIS; // слишком много слов в сообщении
						}
						smb = 0; //запись '\0' вместо пробела
						Handle->Buffer[ index++ ] = smb; //сохранение символа
					}
					else
					{
						if (smb == 0) //число 0 не сохраняется
						{
							break;
						}
						else 
						{
							Handle->Buffer[ index++ ] = smb; //сохранение символа
						}
					}
				}
				else //если слишком длинное сообщение
				{					
					Handle->StateMachine = SM_WAIT_PARENTHESIS; //режим ожидания получения ответа от UPSа
				}
			}
			break;

		default:
			Handle->StateMachine = SM_WAIT_PARENTHESIS; //режим ожидания получения ответа от UPSа
		  break;
	}

  Handle->index = index; //сохранение счётчика полученных символов
	Handle->items_count = items_count; //сохранение счётчика полученных слов
	return result;
}

//----------------------------------------------------------------------------------------------------//
TUPS_PROTOCOL_HANDLE UPS_PROTOCOL_Create( void )
//----------------------------------------------------------------------------------------------------//
{
  TUPS_PROTOCOL_HANDLE handle;
  static bool need_init = true; //статус инициализации структуры соединения
	uint8_t i;
	
	if( need_init ) //если требуется инициализация структур
	{
		for(i = 0 ; i < MAX_UPS_PROTOCOL_HANDLES ; i++ ) //определение количества необходимых структур
			{m_Handle_In_Use[ i ] = false;} //необходимо получить адрес структуры

		need_init = false;
	}

	handle = NULL;
	for(i = 0 ; i < MAX_UPS_PROTOCOL_HANDLES ; i++ ) //получение адреса структуры
	{	
		if( !m_Handle_In_Use[i] ) //если необходимо получить адрес структуры
		{
			handle = &m_UPS_PROTOCOL_Instances[i]; //указатель на элемент массива со структурой
			m_Handle_In_Use[ i ] = true; //адрес структуры получен
			break;
		}	
	}
  if( handle ) //если указатель проинициализирован
  	UPS_PROTOCOL_Reset( handle ); //активация режима ожидания получения сообщения
	
  return handle;
}

//----------------------------------------------------------------------------------------------------//
void UPS_PROTOCOL_Reset( TUPS_PROTOCOL_HANDLE Handle )
//----------------------------------------------------------------------------------------------------//
{
	Handle->StateMachine = SM_WAIT_PARENTHESIS; //активация режима ожидания получения сообщения
	Handle->Items.Count = 0; //сброс счётчика полученных слов 
}

//----------------------------------------------------------------------------------------------------//
void UPS_PROTOCOL_Destroy( TUPS_PROTOCOL_HANDLE Handle )
//----------------------------------------------------------------------------------------------------//
{
	for(int i = 0 ; i < MAX_UPS_PROTOCOL_HANDLES ; i++ )
	{	
		if( m_Handle_In_Use[i] == true && Handle == &m_UPS_PROTOCOL_Instances[i]) //если указатель проинициализирован
		{
			m_Handle_In_Use[i] = false; //указателю необходима инициализация
			break;
		}	
	}
}

//----------------------------------------------------------------------------------------------------//
