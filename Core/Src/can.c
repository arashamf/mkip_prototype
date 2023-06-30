// Includes -------------------------------------------------------------------------------------------//
#include "can.h"
#include <stdio.h>
#include <stdbool.h>
#include "usart.h"
#include "pins.h"
#include "lib_delay.h"
#include "HW_Profile.h"

//Private variables -----------------------------------------------------------------------------------//
static CAN_TxHeaderTypeDef CAN_TxHeader; //структура TxHeader отвечает за отправку кадров
static CAN_RxHeaderTypeDef CAN_RxHeader; //структура для приёма сообщения CAN1

static CAN_RX_msg CAN1_RX;
static uint32_t TxMailbox = 0;//номер почтового ящика для отправки 
static uint32_t ID_C2 = 0; //CAN заголовок сообщения типа С2
uint32_t MyModuleAddress = 0;	// адрес в кроссе

CAN_HandleTypeDef hcan;

//-----------------------------------------------------------------------------------------------------//
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
   // __HAL_RCC_CAN1_CLK_ENABLE(); //CAN1 clock enable 
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CAN1EN);

    __HAL_RCC_GPIOA_CLK_ENABLE();
     
    GPIO_InitStruct.Pin = CAN_RX_PIN; //PA11--> CAN_RX 
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CAN_RX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CAN_TX_PIN; //PA12--> CAN_TX  
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CAN_TX_PORT, &GPIO_InitStruct);

		//ремап пинов GPIO для CAN1
		if (CAN_TX_PORT == GPIOA)
			{__HAL_AFIO_REMAP_CAN1_1();} //CAN_RX mapped to PA11, CAN_TX mapped to PA12
		else
			if (CAN_TX_PORT == GPIOB)
				{__HAL_AFIO_REMAP_CAN1_2();} //CAN_RX mapped to PB8,  CAN_TX mapped to PB9
			else
					if (CAN_TX_PORT == GPIOD)
						{__HAL_AFIO_REMAP_CAN1_3();} //CAN_RX mapped to PD0,  CAN_TX mapped to PD1
					
    // CAN1 interrupt Init //
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn); //включение прервания по приёму в буффер FIFO0
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn); //включение прервания для ошибок  
  }
}
//-----------------------------------------------------------------------------------------------------//
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();  // Peripheral clock disable /

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12); //PA11-> CAN_RX , PA12-> CAN_TX

    // CAN1 interrupt Deinit //
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  }
}


//-----------------------------------------------------------------------------------------------------//
void init_CAN (void)
{
	hcan.Instance = MY_CAN;
  hcan.Init.Prescaler = 18; //предделитель CAN
  hcan.Init.Mode = CAN_MODE_NORMAL; //режим работы CAN
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE; //если включить, тогда узел превращается в Time Master и с определённым интервалом начинает посылать в сеть сообщения, по которым другие узлы синхронизируются
  hcan.Init.AutoBusOff = ENABLE; //если Automatic Bus-Off включён, то CAN, будет автоматически восстанавливаться
  hcan.Init.AutoWakeUp = ENABLE; //если включено, то активность на шине разбудит спящий узел
  hcan.Init.AutoRetransmission = DISABLE; //при включёнии, узел будет повторять попытки отправить сообщение если не получает подтверждения приёма
  hcan.Init.ReceiveFifoLocked = DISABLE; //Если отключён, тогда если все mailbox FIFO заполнены, а сообщения не вычитываются, последнее сообщение будет перезаписываться новым
  hcan.Init.TransmitFifoPriority = ENABLE; //Если режим включён, тогда сообщения отправляются из mailbox по принципу FIFO — первым пришёл, первым вышел. Если отключён, тогда первыми улетают сообщения с более высоким приоритетом
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

	//-----------настройка фильтра----------//
	CAN_FilterTypeDef can_FIFO0_filter;
	
	can_FIFO0_filter.FilterBank = 0; //номер фильтра
	can_FIFO0_filter.FilterIdHigh = 0x0000; // старшая часть первого регистра фильтра
	can_FIFO0_filter.FilterIdLow = 0x0000; // младшая часть первого регистра фильтра
	can_FIFO0_filter.FilterMaskIdHigh = 0x0000; // старшая часть маски фильтра
	can_FIFO0_filter.FilterMaskIdLow = 0x0000; // младшая часть маски фильтра
	can_FIFO0_filter.FilterFIFOAssignment = CAN_RX_FIFO0; //настройка фильтра для приёмного буфера CAN_RX_FIFO0
	can_FIFO0_filter.FilterMode = CAN_FILTERMODE_IDMASK; //режим работы фильтра
	can_FIFO0_filter.FilterScale =  CAN_FILTERSCALE_32BIT; //размерность фильтра, 32 бита - фильтроваться могут либо стандартные (11 бит) идентификаторы, либо расширенные (29 бит)
	can_FIFO0_filter.FilterActivation = ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan, &can_FIFO0_filter) != HAL_OK)
		{Error_Handler();}
	
	HAL_CAN_Start(&hcan); 
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING 
	| CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE); //настройка прерываний CAN
	
	CAN1_RX.flag_RX = RX_NONE; //установка статуса приёма CAN: сообщение не принято
	MyModuleAddress = Get_Module_Address(); //получение адреса в кросс-плате
	ID_C2 = MAKE_FRAME_ID(CAN_MSG_TYPE_C_ID, MyModuleAddress); //формирование и сохранение ID CAN-сообщения

}

//-------------------------------------------------------------------------------------------------------------//
void CAN_Reinit (void)
{
	HAL_CAN_DeInit (&hcan);
	init_CAN ();
}


//--------------------------------------коллбэк для буфера приёма FIFO №0--------------------------------------//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //если пришло прерывание получения пакета в буфер FIFO0 CAN1
	{
		CAN1_RX.flag_RX = RX_UNKNOWN; //установка статуса приёма CAN: принято неиндентифицированное сообщение 
	//	sprintf (buffer_TX_UART3, (char *)"FIFO0_id=%x,msg=%x_%x,my_id=%x\r\n", CAN_RxHeader.StdId, CAN1_RX.RxData[0], CAN1_RX.RxData[1], ID_C2);
	//	UART3_PutString (buffer_TX_UART3);
	}	
}

//---------------------------------коллбек ошибки по переполнению Fifo0---------------------------------//
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	g_MyFlags.CAN_Fail = 1;
	sprintf (buffer_TX_UART3, (char *)"CAN_FIFO0_Full");
	UART3_PutString (buffer_TX_UART3);
}

//----------------------------------коллбэк для буфера приёма FIFO №1----------------------------------//
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO1, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //если пришло прерывание получения пакета в буфер FIFO0 CAN1
	{
		CAN1_RX.flag_RX = RX_UNKNOWN; //установка статуса приёма CAN: принято неиндентифицированное сообщение 
		//sprintf (buffer_TX_UART3, (char *)"FIFO1_id=%x, msg=%x_%x\r\n", CAN_RxHeader.StdId, CAN1_RX.RxData[0], CAN1_RX.RxData[1]);
		//UART3_PutString (buffer_TX_UART3);
	}	
}

//---------------------------------коллбек ошибки по переполнению Fifo1---------------------------------//
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
	g_MyFlags.CAN_Fail = 1;
	sprintf (buffer_TX_UART3, (char *)"CAN_FIFO1_Full");
	UART3_PutString (buffer_TX_UART3);
}

//------------------------------------------коллбек ошибок CAN------------------------------------------//
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t errorcode = 0;
	if ((errorcode = HAL_CAN_GetError(hcan)) != HAL_OK)
	{
		g_MyFlags.CAN_Fail = 1;
		sprintf (buffer_TX_UART3, (char *)"CAN1_ERROR=%u\r\n", errorcode);
		UART3_PutString (buffer_TX_UART3);
	}
}	

//---------------------------Чтение сообщений C1 (от МИУ) и C2 (собственных)---------------------------//
static TRxResult ReadMsgCAN(void)
{
	if (CAN1_RX.flag_RX == RX_NONE)
		return CAN1_RX.flag_RX;

	if( CAN_RxHeader.RTR == CAN_RTR_REMOTE) //если в полученном сообщении установлен бит RTR
	{
		if ((CAN_RxHeader.StdId & MyModuleAddress) == MyModuleAddress)
			{return (CAN1_RX.flag_RX = RX_C1);} //получено сообщение С1
	}
	else
	{
		// Если не установлен RTR - проверка, что это наше собственное сообщение
		if( (CAN_RxHeader.DLC == 8) && ( CAN_RxHeader.RTR == CAN_RTR_DATA) && ((CAN_RxHeader.StdId & MyModuleAddress) == MyModuleAddress))
			{return (CAN1_RX.flag_RX = RX_OWN_C2);}
	}
	return CAN1_RX.flag_RX;
}

//-----------------------------------------------------------------------------------------------------//
void TaskCAN( void )
{
	Task_ProcCANRequests_And_CheckCANCondition();
}

//-----------------------------------------------------------------------------------------------------//
static void Task_ProcCANRequests_And_CheckCANCondition( void )
{
	static uint32_t errorcode; //код ошибки CAN
  static bool need_init = true;
  static uint32_t last_c2_tx_ticks; //static - значения сохраняются между вызовами 
  uint32_t current_ticks;

	current_ticks = HAL_GetTick();

	if( need_init )
	{
		last_c2_tx_ticks = current_ticks; //сохранения текущего значения времени
		need_init = false;
	}
   	
	switch(ReadMsgCAN() )
	{
		case RX_C1: //получен запрос C1  

			if ((errorcode = Send_Message_C2()) != HAL_OK ) //отправление сообщения C2
				{g_MyFlags.CAN_Fail = 1;} //если отправка не удалась - установка флага отказа CAN
			else
				{g_MyFlags.CAN_Fail = 0;}  // сброс флага отказа CAN
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			last_c2_tx_ticks = current_ticks; //запоминание времени отправки сообщения С2
			break;

		case RX_OWN_C2: //получено собственное сообщение C2 

			g_MyFlags.CAN_Fail = 0;  // сброс флага отказа CAN
			//last_c2_rx_ticks = current_ticks; //сохранение текущего количества тиков
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			break;

		case RX_NONE:
		break;
		
		case RX_UNKNOWN:
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			break;
		
		default:
		break;
	}
	
	if( current_ticks - last_c2_tx_ticks > 4*TICKS_PER_SECOND ) //если долго (4с) не отправляли С2 (в ответ на С1)
	{		
		if ((errorcode = Send_Message_C2 ()) != HAL_OK ) //отправка сообщения C2 для контроля работоспособности CAN, получение статуса отправки
		{
			g_MyFlags.CAN_Fail = 1; // установка флага отказа CAN
			CAN_Reinit ();	
		} 
		last_c2_tx_ticks = current_ticks; //сохранение текущего количества тиков
	}
}

//--------------------------------------------------------------------------------------------------------------//
uint32_t Send_Message_C2 ()
{
	uint32_t errorcode; //код ошибки CAN
	uint8_t *msg_flags;
	uint8_t CAN_Tx_buffer[8];
	CAN_MSG_TYPE_C_MKIP my_can_msg = {0, 0};
	
	my_can_msg.data_type = 0; //младшие 3 бита 1 байта сообщения С2 равны 0
	my_can_msg.module_type = MY_MODULE_TYPE; //старшие 5 битов 1 байта сообщения С2 равны 0х15 ( тип модуля-отправителя - МКИП)
	my_can_msg.state = g_MyFlags.UPS_state; //в старшие 4 бита 2 байта передаётся статус UPSa
	
	msg_flags = my_can_msg.bytes; 
	for (uint8_t count = 0; count < 8; count++)
	{
		CAN_Tx_buffer[count] = *(msg_flags+count); 
	}
	//формирование CAN - заголовка
	CAN_TxHeader.StdId = ID_C2; //ID стандартного заголовка
	CAN_TxHeader.ExtId = 0;
	CAN_TxHeader.RTR = CAN_RTR_DATA; //тип сообщения (CAN_RTR_Data - передача данных)
	CAN_TxHeader.IDE = CAN_ID_STD;   //формат кадра Standard
	CAN_TxHeader.DLC = 8; //количество байт в сообщении
	CAN_TxHeader.TransmitGlobalTime = 0;
	
//	sprintf (buffer_TX_UART3, (char *)"id=%x, msg=%x_%x\r\n", CAN_TxHeader.StdId, CAN_Tx_buffer[0], CAN_Tx_buffer[1]);
//	UART3_PutString (buffer_TX_UART3);
	
	{
		return (errorcode = CAN1_Send_Message (&CAN_TxHeader, CAN_Tx_buffer));
	}
}

//--------------------------------------------------------------------------------------------------------------//
uint32_t CAN1_Send_Message (CAN_TxHeaderTypeDef * TxHeader, uint8_t * CAN_TxData)
{
	uint32_t errorcode; //код ошибки CAN
	uint32_t uwCounter = 0;
	
	
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) && (uwCounter != 0xFFFF)) //ожидание освобождения TxMailbox
		{uwCounter++;} 
	
	if (uwCounter == 0xFFFF)	//выход по тайм-ауту
		{return (errorcode = HAL_TIMEOUT);}
	
	if (READ_BIT (CAN1->TSR, CAN_TSR_TME0)) 
		{TxMailbox = 0;}
	else
	{
		if (READ_BIT (CAN1->TSR, CAN_TSR_TME1)) 
			{TxMailbox = 1;}
		else
		{
			if (READ_BIT (CAN1->TSR, CAN_TSR_TME2)) 
				{TxMailbox = 2;}
		}
	}
	return (errorcode = HAL_CAN_AddTxMessage(&hcan, TxHeader, CAN_TxData, &TxMailbox)); //Добавление сообщений в первый свободный Mailboxe Tx и активация запроса на передачу  
}

//-------------------------------------------------------------------------------------------------------------//

