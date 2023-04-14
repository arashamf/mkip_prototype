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

typedef struct 
{
  char buffer_RX_UART [BUFFER_SIZE];
	uint8_t first_count; //счётчик полученных байтов
	uint8_t tail_count; //счётчик переданных байтов
} uart_Rx_data;

static uart_Rx_data UPS_rx_data;
char buffer_TX_UART1 [25];
/* USER CODE END 0 */

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 2400;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
	LL_USART_ClearFlag_RXNE (USART1); // сброс флага прерывания по приёму
	LL_USART_EnableIT_RXNE (USART1); // разрешение прерываний по приёму от USART1
	LL_USART_EnableIT_ERROR (USART1); // разрешение прерываний при ошибках USART1
  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 1 */
//-------------------------------передача символа по RS232-----------------------------------//
void RS232_PutByte(char c)
{
while(!(USART1->SR & USART_SR_TC)) {}; 
USART1->DR = c; 
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
	
	smb = LL_USART_ReceiveData8(USART1);
	UPS_rx_data.buffer_RX_UART [UPS_rx_data.first_count] = smb;
	UPS_rx_data.first_count++;
	
	if (UPS_rx_data.first_count >= (BUFFER_SIZE))
	{
		UPS_rx_data.first_count = 0;
	}
}

//---------------------------------------------------------------------------------------------//
uint8_t UartGetc(uint8_t count)
{
  uint8_t smb;
	smb = UPS_rx_data.buffer_RX_UART [UPS_rx_data.tail_count];
	UPS_rx_data.tail_count++;
	if (UPS_rx_data.tail_count >= (BUFFER_SIZE))
	{
		UPS_rx_data.tail_count = 0;
	}
	return smb;
}

//---------------------------------------------------------------------------------------------//
void UartRxClear( void )
{
	LL_USART_DisableIT_RXNE(USART1);
	UPS_rx_data.first_count = 0; //обнуление всех счётчиков
	UPS_rx_data.tail_count = 0;
	LL_USART_EnableIT_RXNE (USART1);
}
/* USER CODE END 1 */
