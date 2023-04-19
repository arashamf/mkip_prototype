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
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "usart.h"
//#include "lcd1602.h"
#include "pins.h"
#include "lib_delay.h"

static CAN_TxHeaderTypeDef CAN_TxHeader; //��������� TxHeader �������� �� �������� ������
static CAN_RxHeaderTypeDef CAN_RxHeader; //��������� ��� ����� ��������� CAN1

static CAN_RX_msg CAN1_RX;
static uint32_t TxMailbox = 0;//����� ��������� ����� ��� �������� 
static uint32_t ID_C2 = 0; //CAN ��������� 
uint32_t MyModuleAddress = 0;	// ����� � ������
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
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	//��������� �������
	CAN_FilterTypeDef can_FIFO0_filter;
	
	can_FIFO0_filter.FilterBank = 0; //����� �������
	can_FIFO0_filter.FilterIdHigh = 0x0000; // ������� ����� ������� �������� �������
	can_FIFO0_filter.FilterIdLow = 0x0000; // ������� ����� ������� �������� �������
	can_FIFO0_filter.FilterMaskIdHigh = 0x0000; // ������� ����� ����� �������
	can_FIFO0_filter.FilterMaskIdLow = 0x0000; // ������� ����� ����� �������
	can_FIFO0_filter.FilterFIFOAssignment = CAN_RX_FIFO0; //��������� ������� ��� �������� ������ CAN_RX_FIFO0
	can_FIFO0_filter.FilterMode = CAN_FILTERMODE_IDMASK; //����� ������ �������
	can_FIFO0_filter.FilterScale =  CAN_FILTERSCALE_32BIT; //����������� �������, 32 ���� - ������������� ����� ���� ����������� (11 ���) ��������������, ���� ����������� (29 ���)
	can_FIFO0_filter.FilterActivation = ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan, &can_FIFO0_filter) != HAL_OK)
  {
    Error_Handler();
  }
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING |  CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
	
	CAN1_RX.flag_RX = RX_NONE; //��������� ������� ����� CAN: ��������� �� �������
	MyModuleAddress = Get_Module_Address(); //��������� ������ � �����-�����
	ID_C2 = MAKE_FRAME_ID(CAN_MSG_TYPE_C_ID, MyModuleAddress); //������������ � ���������� ID CAN-���������
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
//--------------------------------------������� ��� ������ ����� FIFO �0--------------------------------------//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //���� ������ ���������� ��������� ������ � ����� FIFO0 CAN1
	{
		CAN1_RX.flag_RX = RX_UNKNOWN; //��������� ������� ����� CAN: ������� ��������������������� ��������� 
	//	sprintf (buffer_TX_UART3, (char *)"FIFO0_id=%x,msg=%x_%x,my_id=%x\r\n", CAN_RxHeader.StdId, CAN1_RX.RxData[0], CAN1_RX.RxData[1], ID_C2);
	//	UART3_PutString (buffer_TX_UART3);
	}	
}

//------------------------------------������� ������ �� ������������ Fifo0------------------------------------//
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	g_MyFlags.CAN_Fail = 1;
	sprintf (buffer_TX_UART3, (char *)"CAN_FIFO0_Full");
	UART3_PutString (buffer_TX_UART3);
}

//--------------------------------------������� ��� ������ ����� FIFO �1--------------------------------------//
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO1, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //���� ������ ���������� ��������� ������ � ����� FIFO0 CAN1
	{
		CAN1_RX.flag_RX = RX_UNKNOWN; //��������� ������� ����� CAN: ������� ��������������������� ��������� 
		//sprintf (buffer_TX_UART3, (char *)"FIFO1_id=%x, msg=%x_%x\r\n", CAN_RxHeader.StdId, CAN1_RX.RxData[0], CAN1_RX.RxData[1]);
		//UART3_PutString (buffer_TX_UART3);
	}	
}

//------------------------------------������� ������ �� ������������ Fifo1------------------------------------//
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
	g_MyFlags.CAN_Fail = 1;
	sprintf (buffer_TX_UART3, (char *)"CAN_FIFO1_Full");
	UART3_PutString (buffer_TX_UART3);
}

//---------------------------------------------������� ������ CAN---------------------------------------------//
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

//--------------------------------������ ��������� C1 (�� ���) � C2 (�����������)--------------------------------//
static TRxResult ReadMsgCAN(void)
{
	if (CAN1_RX.flag_RX == RX_NONE)
		return CAN1_RX.flag_RX;

	if( CAN_RxHeader.RTR == CAN_RTR_REMOTE) //���� ���������� RTR
	{
		return (CAN1_RX.flag_RX = RX_C1);
	}
	else
	{
		// ���� �� ���������� RTR - ��������, ��� ��� ���� ����������� ���������
		if( (CAN_RxHeader.DLC == 8) && ( CAN_RxHeader.RTR == CAN_RTR_DATA) && ((CAN_RxHeader.StdId & MY_MODULE_TYPE) == MY_MODULE_TYPE))
			return (CAN1_RX.flag_RX = RX_OWN_C2);
		else
			return CAN1_RX.flag_RX;
	}
}

//-------------------------------------------------------------------------------------------------------------//
void TaskCAN( void )
{

	Task_ProcCANRequests_And_CheckCANCondition();
}

//-------------------------------------------------------------------------------------------------------------//
static void Task_ProcCANRequests_And_CheckCANCondition( void )
{
	static uint32_t errorcode; //��� ������ CAN
  static bool need_init = true;
  static uint32_t last_c2_tx_ticks; //static - �������� ����������� ����� �������� 
	//static uint32_t last_c2_rx_ticks; 
  uint32_t current_ticks;

	current_ticks = HAL_GetTick();

	if( need_init )
	{
		last_c2_tx_ticks = current_ticks; //���������� �������� �������� �������
		need_init = false;
	}
   	
	switch(ReadMsgCAN() )
	{
		case RX_C1: //������� ������ C1  

			errorcode = Send_Message_C2 (); //����������� ��������� C2
			g_MyFlags.CAN_Fail = 0;  // ����� ����� ������ CAN
		//	LED_GREEN (ON); LED_RED (OFF);
			last_c2_tx_ticks = current_ticks; //����������� ������� �������� ��������� �2
			break;

		case RX_OWN_C2: //�������� ����������� ��������� C2 

			g_MyFlags.CAN_Fail = 0;  // ����� ����� ������ CAN
		//	LED_GREEN (ON); LED_RED (OFF);
			//last_c2_rx_ticks = current_ticks; //���������� �������� ���������� �����
			break;

		case RX_NONE:
		break;
		
		case RX_UNKNOWN:
		break;
		
		default:
		break;
	}
	
	CAN1_RX.flag_RX = RX_NONE; //����� ����� ����������� CAN ���������
	
	if( current_ticks - last_c2_tx_ticks > 4 * TICKS_PER_SECOND ) //���� ����� (4�) �� ���������� �2 (� ����� �� �1),
	{		
		if ((errorcode = Send_Message_C2 ()) != HAL_OK ) //�������� C2 � ����� �������� ����������������� CAN, ��������� ������� ��������
		{
			g_MyFlags.CAN_Fail = 1; // ��������� ����� ������ CAN
		}
		last_c2_tx_ticks = current_ticks; //���������� �������� ���������� �����
	}
}

//-------------------------------------------------------------------------------------------------------------//
uint32_t Send_Message_C2 ()
{
	uint32_t errorcode; //��� ������ CAN
	uint8_t *msg_flags;
	uint8_t CAN_Tx_buffer[8];
	CAN_MSG_TYPE_C_MKIP my_can_msg = {0, 0};
	
	my_can_msg.data_type = 0; //������� 3 ���� 1 ����� ��������� �2 ����� 0
	my_can_msg.module_type = MY_MODULE_TYPE; //������� 5 ����� 1 ����� ��������� �2 ����� 10101
	my_can_msg.state = g_MyFlags.UPS_state; //� ������� 4 ���� 2 ����� ��������� ������ UPSa
	
	msg_flags = my_can_msg.bytes; 
	for (uint8_t count = 0; count < 8; count++)
	{
		CAN_Tx_buffer[count] = *(msg_flags+count); 
	}
	//������������ CAN - ���������
	CAN_TxHeader.StdId = ID_C2; //ID ������������ ���������
	CAN_TxHeader.ExtId = 0;
	CAN_TxHeader.RTR = CAN_RTR_DATA; //��� ��������� (CAN_RTR_Data - �������� ������)
	CAN_TxHeader.IDE = CAN_ID_STD;   //������ ����� Standard
	CAN_TxHeader.DLC = 8; //���������� ���� � ���������
	CAN_TxHeader.TransmitGlobalTime = 0;
	
	sprintf (buffer_TX_UART3, (char *)"id=%x, msg=%x_%x\r\n", CAN_TxHeader.StdId, CAN_Tx_buffer[0], CAN_Tx_buffer[1]);
	UART3_PutString (buffer_TX_UART3);
	
	//if (g_MyFlags.UPS_state == UPS_OK) //�������� ������� ���������� � UPS��
	{
		return (errorcode = CAN1_Send_Message (&CAN_TxHeader, CAN_Tx_buffer));
	}
}

//-------------------------------------------------------------------------------------------------------------//
uint32_t CAN1_Send_Message (CAN_TxHeaderTypeDef * TxHeader, uint8_t * CAN_TxData)
{
	uint32_t errorcode; //��� ������ CAN
	uint32_t uwCounter = 0;
	
	
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) && (uwCounter != 0xFFFF)) //�������� ������������ TxMailbox
	{
		uwCounter++;
	} 
	
	if (uwCounter == 0xFFFF)	//����� �� ����-����
	{
		return (errorcode = HAL_TIMEOUT);
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
	return (errorcode = HAL_CAN_AddTxMessage(&hcan, TxHeader, CAN_TxData, &TxMailbox)); //���������� ��������� � ������ ��������� Mailboxe Tx � ��������� ������� �� ��������  
}
/* USER CODE END 1 */
