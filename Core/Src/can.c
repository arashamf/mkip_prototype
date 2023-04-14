/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
static CAN_TxHeaderTypeDef CAN_TxHeader; //структура TxHeader отвечает за отправку кадров
static CAN_RxHeaderTypeDef CAN_RxHeader; //структура для приёма сообщения CAN1

static uint32_t TxMailbox = 0;//номер почтового ящика для отправки 

typedef enum { RX_NONE = 0, RX_C1, RX_OWN_C2, RX_UNKNOWN} TRxResult;

static CAN_RX_msg CAN1_RX;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	//настройка фильтра
	CAN_FilterTypeDef can0_filter;
	
	can0_filter.FilterBank = 0; //номер фильтра
	can0_filter.FilterIdHigh = 0x0000; // старшая часть первого регистра фильтра
	can0_filter.FilterIdLow = 0x0000; // младшая часть первого регистра фильтра
	can0_filter.FilterMaskIdHigh = 0x0000; // старшая часть маски фильтра
	can0_filter.FilterMaskIdLow = 0x0000; // младшая часть маски фильтра
	can0_filter.FilterFIFOAssignment = CAN_RX_FIFO0; //настройка фильтра для приёмного буфера CAN_RX_FIFO0
	can0_filter.FilterMode = CAN_FILTERMODE_IDMASK; //режим работы фильтра
	can0_filter.FilterScale =  CAN_FILTERSCALE_32BIT; //размерность фильтра, 32 бита - фильтроваться могут либо стандартные (11 бит) идентификаторы, либо расширенные (29 бит)
	can0_filter.FilterActivation = ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan, &can0_filter) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING |  CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
	
	CAN1_RX.flag_RX = RX_NONE; //сообщение не принято
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//--------------------------------------коллбэк для буфера приёма FIFO №0--------------------------------------//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //если пришло прерывание получения пакета в буфер FIFO0 CAN1
	{
		LED_GREEN (OFF); LED_RED (ON);
		CAN1_RX.flag_RX = RX_UNKNOWN;
	/*	if ((CAN1_Rx_buf.StdId & ID_adress_mask) == ID_adress)	//проверка адреса отправителя
		{
			if ((CAN1_Rx_buf.StdId >> 5) == ID_C1) //проверка типа сообщения
			{	
				if (CAN1_Rx_buf.RTR == CAN_RTR_REMOTE) //если запрос типа Remote
				{
					if (g_MyFlags.UPS_state != UPS_NO_LINK)
					{
						Send_Message_C2 (&ups_msg_items);
						LED_GREEN (ON); LED_RED (OFF);
					}
				}
			}
		}*/
	}	
}

//------------------------------------коллбек ошибки по переполнению Fifo0------------------------------------//
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	//sprintf (buffer_TX_UART1, (char *)"CAN_Fifo0_Full");
	//UART1_PutString (buffer_TX_UART1);
}

//--------------------------------------коллбэк для буфера приёма FIFO №0--------------------------------------//
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO1, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //если пришло прерывание получения пакета в буфер FIFO0 CAN1
	{
		LED_GREEN (OFF); LED_RED (ON);
		CAN1_RX.flag_RX = RX_UNKNOWN;
	}	
}

//------------------------------------коллбек ошибки по переполнению Fifo0------------------------------------//
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
	//sprintf (buffer_TX_UART1, (char *)"CAN_Fifo0_Full");
	//UART1_PutString (buffer_TX_UART1);
}

//---------------------------------------------коллбек ошибок CAN---------------------------------------------//
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t errorcode = 0;
	if ((errorcode = HAL_CAN_GetError(hcan)) != HAL_OK)
	{
		//sprintf (buffer_TX_UART1, (char *)"CAN1_ERROR=%u\r\n", errorcode);
		//UART1_PutString (buffer_TX_UART1);
	}
}	

//--------------------------------Чтение сообщений C1 (от МИУ) и C2 (собственных)--------------------------------//
static TRxResult RxMsgC(void)
{
	if (CAN1_RX.flag_RX == RX_NONE)
		return RX_NONE;

	if( CAN_RxHeader.RTR == CAN_RTR_REMOTE) //если установлен RTR
	{
		return (CAN1_RX.flag_RX = RX_C1);
	}
	else
	{
		// Если не установлен RTR - проверим, что это наше собственное сообщение
		if( (CAN_RxHeader.DLC == 8) && ( CAN_RxHeader.RTR == CAN_RTR_DATA) && ((CAN_RxHeader.StdId & MY_MODULE_TYPE) == MY_MODULE_TYPE))
			return (CAN1_RX.flag_RX = RX_OWN_C2);
		else
			return RX_NONE;
	}
}

//-------------------------------------------------------------------------------------------------------------//
void Send_Message_C2 (TUPS_PROTOCOL_ITEMS *ups_msg_items)
{
	char *msg_flags;
	msg_flags = ups_msg_items->ptr_Items[7]; //указатель на 8 слово
	uint8_t CAN_Tx_buffer[8];
	
	
	for (uint8_t count = 0; count < 8; count++)
	{
		CAN_Tx_buffer[count] = *(msg_flags+count); 
	}
	
	//формирование CAN - заголовка
	CAN_TxHeader.StdId = ((ID_C2 << 5) | (ID_adress)); //ID стандартного заголовка
	CAN_TxHeader.ExtId = 0;
	CAN_TxHeader.RTR = CAN_RTR_DATA; //тип сообщения (CAN_RTR_Data - передача данных)
	CAN_TxHeader.IDE = CAN_ID_STD;   //формат кадра Standard
	CAN_TxHeader.DLC = 8; //количество байт в сообщении
	CAN_TxHeader.TransmitGlobalTime = 0;
	
	if (g_MyFlags.UPS_state == UPS_OK)
	{
		CAN1_Send_Message (&CAN_TxHeader, CAN_Tx_buffer);
	}
}

//-------------------------------------------------------------------------------------------------------------//
void CAN1_Send_Message (CAN_TxHeaderTypeDef * TxHeader, uint8_t * CAN_TxData)
{
	uint32_t errorcode;
	uint32_t uwCounter = 0;
	
	
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) && (uwCounter != 0xFFFF)) //ожидание освобождения TxMailbox
	{
		uwCounter++;
	} 
	
	if (uwCounter == 0xFFFF)	//выход по тайм-ауту
	{
	//	sprintf (buffer_TX_UART1, "CAN1_TX_time_out\r\n");
	//	UART1_PutString (buffer_TX_UART1);
		return;
	}
	
	if (READ_BIT (CAN1->TSR, CAN_TSR_TME0)) {
		TxMailbox = 0;}
	else
	{
		if (READ_BIT (CAN1->TSR, CAN_TSR_TME1)) {
		TxMailbox = 1;}
		else
		{
			if (READ_BIT (CAN1->TSR, CAN_TSR_TME2)) {
			TxMailbox = 2;}
		}
	}
	
	if ((errorcode = HAL_CAN_AddTxMessage(&hcan, TxHeader, CAN_TxData, &TxMailbox)) != HAL_OK) //Добавление сообщений в первый свободный Mailboxe Tx и активация запроса на передачу
	{	 
		//sprintf (buffer_TX_UART1, "CAN1_TX_ERROR=%u", errorcode);
		//UART1_PutString (buffer_TX_UART1);
	}
}

//-------------------------------------------------------------------------------------------------------------//
static void Task_ProcCANRequests_And_CheckCANCondition( void )
{
  static bool need_init = true;
  static uint32_t last_c2_tx_ticks, last_c2_rx_ticks; //static - значения сохраняются между вызовами
  uint32_t current_ticks;

	current_ticks = HAL_GetTick();

	if( need_init )
	{
		last_c2_tx_ticks = last_c2_rx_ticks = current_ticks; //сохранения текущего количества тиков
		need_init = false;
	}
   	
	switch( RxMsgC() )
	{
		case RX_C1:

			Send_Message_C2 (&ups_msg_items); //получен запрос C1 - отправление сообщения C2
			last_c2_tx_ticks = current_ticks; //сохранение текущего количества тиков
			break;

		case RX_OWN_C2: 

			g_MyFlags.CAN_Fail = 0;  // Получено собственное сообщение C2, сброс флага отказа CAN
			last_c2_rx_ticks = current_ticks; //сохранение текущего количества тиков
			break;

		case RX_NONE:
		break;
		
		default:
		break;
	}
	
	CAN1_RX.flag_RX = RX_NONE;
	
	if( current_ticks - last_c2_tx_ticks > 4 * TICKS_PER_SECOND ) //если долго (4с) не отправляли С2 (в ответ на С1),
	{
		Send_Message_C2 (&ups_msg_items); 	// отправка C2 с целью контроля работоспособности CAN
		last_c2_tx_ticks = current_ticks; //сохранение текущего количества тиков
	}

	if( current_ticks - last_c2_rx_ticks > 5 * TICKS_PER_SECOND ) //если собственное сообщение C2 не получено в течение 5с
	{
		g_MyFlags.CAN_Fail = 1; // установка флага отказа CAN
	}
}

//-------------------------------------------------------------------------------------------------------------//
void TaskCAN( void )
{

	Task_ProcCANRequests_And_CheckCANCondition();
}
/* USER CODE END 1 */
