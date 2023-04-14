
#include "megatec.h"

#define MAX_UPS_PROTOCOL_HANDLES 1 //������������ ���������� ����������� ��������
#define UPS_PROTOCOL_BUFFER_SIZE 150 //������ �������

static enum //������ ������ � UPS
{
	SM_HOME = 0,
	SM_WAIT_RESPONCE, 
	SM_CHECK_RESPONCE,
	SM_SHORT_DELAY,
	SM_INIT
} state = SM_INIT; //��������� ������ - �������������

struct TUPS_PROTOCOL
{
	TUPS_PROTOCOL_STATES StateMachine;	//����� ������ � UPS
  unsigned int index; //������� ���������� ��������
  unsigned int items_count; //������� ���������� ����

	char Buffer[ UPS_PROTOCOL_BUFFER_SIZE ]; //������ ��������� ���������

	TUPS_PROTOCOL_ITEMS Items; //��������� � ��������� �������
};

//static TUPS_PROTOCOL_HANDLE ups_h;
//static TUPS_PROTOCOL_ITEMS ups_msg_items;
TUPS_PROTOCOL_ITEMS ups_msg_items; //������������� ��������� TUPS_PROTOCOL_ITEMS

static struct TUPS_PROTOCOL m_UPS_PROTOCOL_Instances[ MAX_UPS_PROTOCOL_HANDLES ]; //������ �������� TUPS_PROTOCOL ��� ���������� � UPS
static bool m_Handle_In_Use[ MAX_UPS_PROTOCOL_HANDLES ];

char *msg_flags;
const char status[] = {"Q1\r"};
//----------------------------------------------------------------------------------------------------//

//----------------------------------------------------------------------------------------------------//
void TaskCommUPS( void )
{
	static uint32_t ticks; //��� static 
	static TUPS_PROTOCOL_HANDLE ups_h; //��������� �� ��������� TUPS_PROTOCOL
	
	switch( state )
	{
		case SM_HOME:			//������ ������ ������ � UPS��
			check_status_UPS(); //�������� ������� Q1 UPS�
			ticks = HAL_GetTick(); //��������� �������� ���������� �����
			state = SM_WAIT_RESPONCE; //������� � ����� �������� ��������� ������ �� UPS�
			break;

		case SM_WAIT_RESPONCE: //������ �������� ��������� ������ �� UPS�			
			if( HAL_GetTick() - ticks > (TICKS_PER_SECOND  * 4) ) //���� ��� ������ �� UPS� ������ 4 �
			{
				g_MyFlags.UPS_state = UPS_NO_LINK; //��������� ������� ���������� ����� � UPS��
				ticks = HAL_GetTick(); //��������� �������� ���������� �����
				state = SM_SHORT_DELAY; //��������� ������ ����-����
			}
			else 
			{
				if( UPS_PROTOCOL_Process( ups_h, UartGetc(ups_h->index), &ups_msg_items ) == true ) //���� ��������� �� UPS� �������� ���������
					{state = SM_CHECK_RESPONCE;} //��������� ������ �������� ������
			}
		break;

		case SM_CHECK_RESPONCE: //������ �������� ����������� ���������			
			msg_flags = ups_msg_items.ptr_Items[ 7 ]; //��������� �� ������� �����
		
			if( ups_msg_items.Count == 8 && strlen( msg_flags ) == 8 ) //���� �������� ���� 8 � � ������� ����� 8 ��������
			{
				g_MyFlags.UPS_state = UPS_OK; //������
				/*	if( msg_flags[ 6 ] != '1' ) // Shutdown not active
				{
					if( msg_flags[ 0 ] == '1' ) // Utility Fail 
						g_MyFlags.UPS_state = UPS_BAT_MODE;
					else if( msg_flags[ 1 ] == '1' ) // Battery low
						g_MyFlags.UPS_state = UPS_LOW_BAT;
					else if( msg_flags[ 3 ] == '1' ) // UPS failed
						g_MyFlags.UPS_state = UPS_FAULT;
				}*/
			}
			UartRxClear();
			ticks = HAL_GetTick(); //��������� �������� ���������� �����
			state = SM_SHORT_DELAY; //����-��� �� 3 �
		break;

		case SM_SHORT_DELAY: //������ ����-����					
			if((HAL_GetTick() - ticks) > (TICKS_PER_SECOND*3)) //���� ����� ����-���� ������ ������ 3 �
			{	
				state = SM_HOME; //��������� ������ ������ ������ � UPS�� 
			}
		break;

		case SM_INIT:		//������ �������������
			UartRxClear();
			ups_h = UPS_PROTOCOL_Create(); //��������� ������ ��������� ����������
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
bool UPS_PROTOCOL_Process( TUPS_PROTOCOL_HANDLE Handle, char smb, TUPS_PROTOCOL_ITEMS *Items )
//----------------------------------------------------------------------------------------------------//
{
  unsigned int index; //������� ���������� ��������
  unsigned int items_count; //������� ���������� ����
  bool result = false;

  index = Handle->index; //��������� �������� ���������� ��������
	items_count = Handle->items_count; //��������� �������� ���������� ����

	switch( Handle->StateMachine ) //�������� ���������
	{
		case SM_WAIT_PARENTHESIS: // ������ �������� ��������� ������ �� UPS�		

			if( smb == '(' ) //���� ������� ������ '('
			{
				index = 0; //������� ���������� ��������
				items_count = 0;  //������� ���������� ����
				Handle->Items.ptr_Items[0] = Handle->Buffer; //����������� ��������� �� 1 �����
				Handle->StateMachine = SM_GET_DATA; //������� � ����� ��������� ������
			}
		  break;

		case SM_GET_DATA: // ������ ��������� ������ �� UPS�

			if( smb == '\r' ) //���� ��������� ��������
			{
				items_count++; //���������� �������� ���������� ����
				Handle->Buffer[index++] = 0; //������ '\0' ������ '\r'
				memcpy((void *)Items, (void *)&Handle->Items, sizeof(TUPS_PROTOCOL_ITEMS)); //����������� ��������� �� ���������� ��������� ups_h � ��������� ups_msg_items
				Items->Count = items_count; //���������� �������� ���������� ���� � ��������� ups_msg_items
				Handle->StateMachine = SM_WAIT_PARENTHESIS; //��������� ������� �������� ������� '('
				result = true;
			}
			else
			{
				if( index < (sizeof (Handle->Buffer) - 1) ) //�������� ������� ���������
				{
					if(smb == ' ') //���� ����� ��������
					{
            if(items_count < (sizeof(Handle->Items.ptr_Items) / sizeof(Handle->Items.ptr_Items[0]))) //���� ������� ���� �� �������� ������������ ���������� ���� � �������
						{
            	Handle->Items.ptr_Items[ ++items_count ] = &Handle->Buffer[index + 1]; //���������� ��������� �� ������ ���������� �����
						}
						else
						{							
							Handle->StateMachine = SM_WAIT_PARENTHESIS; // ������� ����� ���� � ���������
						}
						smb = 0; //������ '\0' ������ �������
					}
					Handle->Buffer[ index++ ] = smb; //���������� �������
				}
				else //���� ������� ������� ���������
				{					
					Handle->StateMachine = SM_WAIT_PARENTHESIS; //����� �������� ��������� ������ �� UPS�
				}
			}
			break;

		default:
			Handle->StateMachine = SM_WAIT_PARENTHESIS; //����� �������� ��������� ������ �� UPS�
		  break;
	}

  Handle->index = index; //���������� �������� ���������� ��������
	Handle->items_count = items_count; //���������� �������� ���������� ����
	return result;
}

//----------------------------------------------------------------------------------------------------//
TUPS_PROTOCOL_HANDLE UPS_PROTOCOL_Create( void )
//----------------------------------------------------------------------------------------------------//
{
  TUPS_PROTOCOL_HANDLE handle;
  static bool need_init = true;
	int i;
	
	if( need_init ) //���� ��������� ������������� ��������
	{
		for(i = 0 ; i < MAX_UPS_PROTOCOL_HANDLES ; i++ ) //����������� ���������� ����������� ��������
			{m_Handle_In_Use[ i ] = false;} //���������� �������� ����� ���������

		need_init = false;
	}

	handle = NULL;
	for(i = 0 ; i < MAX_UPS_PROTOCOL_HANDLES ; i++ ) //��������� ������ ���������
	{	
		if( !m_Handle_In_Use[i] ) //���� ���������� �������� ����� ���������
		{
			handle = &m_UPS_PROTOCOL_Instances[i]; //��������� �� ������� ������� �� ����������
			m_Handle_In_Use[ i ] = true; //����� ��������� �������
			break;
		}	
	}
  if( handle ) //���� ��������� ������������������
  	UPS_PROTOCOL_Reset( handle ); //��������� ������ �������� ��������� ���������
  return handle;
}

//----------------------------------------------------------------------------------------------------//
void UPS_PROTOCOL_Reset( TUPS_PROTOCOL_HANDLE Handle )
//----------------------------------------------------------------------------------------------------//
{
	Handle->StateMachine = SM_WAIT_PARENTHESIS; //��������� ������ �������� ��������� ���������
	Handle->Items.Count = 0; //����� �������� ���������� ���� 
}

//----------------------------------------------------------------------------------------------------//
void UPS_PROTOCOL_Destroy( TUPS_PROTOCOL_HANDLE Handle )
//----------------------------------------------------------------------------------------------------//
{
	for(int i = 0 ; i < MAX_UPS_PROTOCOL_HANDLES ; i++ )
	{	
		if( m_Handle_In_Use[i] == true && Handle == &m_UPS_PROTOCOL_Instances[i]) //���� ��������� ������������������
		{
			m_Handle_In_Use[i] = false; //��������� ���������� �������������
			break;
		}	
	}
}

//----------------------------------------------------------------------------------------------------//
