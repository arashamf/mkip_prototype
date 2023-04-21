/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 100 //размер буффера UPS_UART
#define UPS_UART		((USART_TypeDef *)USART1_BASE)

#define UPS_UART_PORT			GPIOA
#define UPS_UART_TX_PIN		LL_GPIO_PIN_9
#define UPS_UART_TX_PIN_FUNCTION	PORT_FUNC_OVERRID

#define UPS_UART_RX_PIN		LL_GPIO_PIN_10
#define UPS_UART_RX_PIN_FUNCTION	PORT_FUNC_ALTER
/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void RS232_Init(void);
void RS232_PutByte(char );
void RS232_PutString(const char *);
void RS232_CharReception_Callback(void);
uint16_t uart_available(void);
uint8_t UartGetc(uint8_t );
void UartRxClear( void );

void UART3_PutByte(char c);
void UART3_PutString(const char *str);

extern char buffer_TX_UART1 [];
extern char buffer_TX_UART3 [];
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

