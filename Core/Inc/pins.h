/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PINS_H__
#define __PINS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines */

//пины адреса модуля в кросс-плате
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

#define LED_RED_PIN		LL_GPIO_PIN_3        
#define LED_RED_PORT	GPIOA  

#define LED_GREEN_PIN		LL_GPIO_PIN_2        
#define LED_GREEN_PORT	GPIOA  

#define ON 1
#define OFF 0

#define LED_GREEN(x) ((x)? (LL_GPIO_SetOutputPin (LED_GREEN_PORT, LED_GREEN_PIN)) : (LL_GPIO_ResetOutputPin(LED_GREEN_PORT, LED_GREEN_PIN)))
#define LED_RED(x) ((x)? (LL_GPIO_SetOutputPin (LED_RED_PORT, LED_RED_PIN)) : (LL_GPIO_ResetOutputPin(LED_RED_PORT, LED_RED_PIN)))

typedef struct 
{
	GPIO_TypeDef *PORTx;
	uint32_t Pin;		
} TPortPin;

/*Prototypes */

void Pins_LEDs_Init(void);
void Pins_Address_Init(void);
uint32_t Get_Module_Address( void );
void Task_Control_LEDs( void );

/*Prototypes*/

#ifdef __cplusplus
}
#endif
#endif /*__ PINS_H__ */

