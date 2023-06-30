#ifndef _HW_Profile_H_
#define _HW_Profile_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported types ------------------------------------------------------------*/
typedef union _MY_FLAGS
{
	unsigned int Value;
	struct //Битовые поля
	{
		unsigned CAN_Fail				: 1;	//тип unsigned, длина поля 1 бит, статус CAN	( нет приема собственных сообщений C2 )
		unsigned UPS_state			: 4; //тип unsigned, длина поля 4 бита, статус RS-232
	};
}TMyFlags;


/* Exported constants --------------------------------------------------------*/
#define CPU_CLOCK_VALUE	(80000000UL)	/* Частота контроллера */

//--------------------------------пины LED
#define LED_RED_PIN		LL_GPIO_PIN_3        
#define LED_RED_PORT	GPIOA  

#define LED_GREEN_PIN		LL_GPIO_PIN_2        
#define LED_GREEN_PORT	GPIOA  

//---------------------------------пины CAN1
#define MY_CAN	((CAN_TypeDef *)CAN1_BASE)

#define CAN_RX_PIN		GPIO_PIN_11
#define CAN_RX_PORT		GPIOA
#define CAN_TX_PIN		GPIO_PIN_12
#define CAN_TX_PORT		GPIOA

//-------------------------------Связь с ИБП
#define UPS_UART		((USART_TypeDef *)USART1_BASE)

#define UPS_UART_PORT							GPIOA
#define UPS_UART_TX_PIN						LL_GPIO_PIN_9
#define UPS_UART_TX_PIN_FUNCTION	PORT_FUNC_OVERRID

#define UPS_UART_RX_PIN		LL_GPIO_PIN_10
#define UPS_UART_RX_PIN_FUNCTION	PORT_FUNC_ALTER

//----------------------Адрес модуля в кроссе
#define MY_BACKPLANE_ADDR0_PIN		LL_GPIO_PIN_6
#define MY_BACKPLANE_ADDR0_PORT		GPIOC
                              
#define MY_BACKPLANE_ADDR1_PIN		LL_GPIO_PIN_7        
#define MY_BACKPLANE_ADDR1_PORT		GPIOC             
                              
#define MY_BACKPLANE_ADDR2_PIN		LL_GPIO_PIN_8        
#define MY_BACKPLANE_ADDR2_PORT		GPIOC             
                             
#define MY_BACKPLANE_ADDR3_PIN		LL_GPIO_PIN_9       
#define MY_BACKPLANE_ADDR3_PORT		GPIOC             
                                        
#define MY_BACKPLANE_ADDR4_PIN		LL_GPIO_PIN_8       
#define MY_BACKPLANE_ADDR4_PORT		GPIOA           

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//-------------------------------------------------------------------------------------------------
	
#endif
