/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "HW_Profile.h"

typedef struct 
{
  char buffer_RX_UART [BUFFER_SIZE];
	uint8_t head_count; //счётчик полученных байтов
	uint8_t tail_count; //счётчик переданных байтов
} uart_Rx_data;

static uart_Rx_data UPS_rx_data;

char buffer_TX_UART1 [10];
char buffer_TX_UART3 [35];
/* USER CODE END 0 */

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* USER CODE BEGIN 1 */

//-----------------------------------------USART init function-----------------------------------------//
void RS232_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	if( UPS_UART == ((USART_TypeDef *)USART1_BASE) ) //USART clock enable
	{
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
	}
	else 
		if( UPS_UART == ((USART_TypeDef *)USART2_BASE))
		{
			LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
		}
		else 
			if( UPS_UART == ((USART_TypeDef *)USART3_BASE))
			{
				LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
			}
	 
	if( UPS_UART_PORT == GPIOA ) //GPIO clock enable
		{LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);}
	else 
		if(UPS_UART_PORT == GPIOB)
			{LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);}
		else 
			if(UPS_UART_PORT == GPIOC)
				{LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);}
			else 
				if(UPS_UART_PORT == GPIOD)
					{LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);}

  GPIO_InitStruct.Pin = UPS_UART_TX_PIN; 
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE; //альтернативная функция
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(UPS_UART_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = UPS_UART_RX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(UPS_UART_PORT, &GPIO_InitStruct);

  USART_InitStruct.BaudRate = 2400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UPS_UART, &USART_InitStruct);

  /* USART interrupt Init */
	if( UPS_UART == ((USART_TypeDef *)USART1_BASE) )
	{
		NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
		NVIC_EnableIRQ(USART1_IRQn);
	}
	else 
		if( UPS_UART == ((USART_TypeDef *)USART2_BASE))
		{
			NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
			NVIC_EnableIRQ(USART2_IRQn);
		}
		else 
			if( UPS_UART == ((USART_TypeDef *)USART3_BASE))
			{
				NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
				NVIC_EnableIRQ(USART3_IRQn);
			}
			
	LL_USART_ClearFlag_RXNE (UPS_UART); // сброс флага прерывания по приёму
	LL_USART_EnableIT_RXNE (UPS_UART); // разрешение прерываний по приёму от USART
	LL_USART_EnableIT_ERROR (UPS_UART); // разрешение прерываний при ошибках USART1

	LL_USART_ConfigAsyncMode(UPS_UART);
  LL_USART_Enable(UPS_UART);
}

//-------------------------------передача символа по RS232-----------------------------------//
void RS232_PutByte(char c)
{
while(!(UPS_UART->SR & USART_SR_TC)) {}; 
UPS_UART->DR = c; 
}

//-------------------------------передача строки по RS232-----------------------------------//
void RS232_PutString(const char *str)
{
	char c;
	while((c = *str++))
	{
		RS232_PutByte(c);
	}
}

//-------------------------------получение символа по UART1-----------------------------------//
void RS232_CharReception_Callback (void)
{
	auto uint8_t smb;
	auto uint16_t i;
	smb = LL_USART_ReceiveData8(UPS_UART);
	i = (UPS_rx_data.head_count + 1) % BUFFER_SIZE; //остаток от деления 
	if(i != UPS_rx_data.tail_count) //если количество полученных байтов не равно количеству переданных
	{
			UPS_rx_data.buffer_RX_UART [UPS_rx_data.head_count] = smb;
			UPS_rx_data.head_count = i;
	}
}

//---------------------------------------------------------------------------------------------//
uint8_t UartGetc(uint8_t count)
{
  uint8_t smb;
	if (UPS_rx_data.tail_count == UPS_rx_data.head_count) //если в буффере нет новых символов
		{smb = 0;}
	else
	{
		smb = UPS_rx_data.buffer_RX_UART [UPS_rx_data.tail_count];
		UPS_rx_data.tail_count = (UPS_rx_data.tail_count + 1) % BUFFER_SIZE; //увеличение кол-ва переданных байт на 1 и сохранение
	}

	return smb;
}

//---------------------------------------------------------------------------------------------//
uint16_t uart_available(void)
{
	//возвращает 0, если количество полученных и переданных байтов равно
	return ((uint16_t)(BUFFER_SIZE + UPS_rx_data.head_count - UPS_rx_data.tail_count)) % BUFFER_SIZE; 
}

//---------------------------------------------------------------------------------------------//
void UartRxClear( void )
{
	LL_USART_DisableIT_RXNE(USART1);
	UPS_rx_data.head_count = 0; //обнуление всех счётчиков кольцевого буффера
	UPS_rx_data.tail_count = 0;
	LL_USART_EnableIT_RXNE (USART1);
}

//-------------------------------передача символа по UART3-----------------------------------//
void UART3_PutByte(char c)
{
while(!(USART3->SR & USART_SR_TC)) {}; 
USART3->DR = c; 
}

//-------------------------------передача строки по UART3-----------------------------------//
void UART3_PutString(const char *str)
{
	char c;
	while((c = *str++))
	{
		UART3_PutByte(c);
	}
}
/* USER CODE END 1 */
