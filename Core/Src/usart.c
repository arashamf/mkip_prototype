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
#include "can.h"
#include "megatec.h"
#include <string.h>
#include <stdio.h>

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

UART_HandleTypeDef huart3;

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
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
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
	auto uint16_t i;
	smb = LL_USART_ReceiveData8(USART1);
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
	{
		smb = 0;
	}
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
	UPS_rx_data.head_count = 0; //обнуление всех счётчиков
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
