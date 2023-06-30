
#ifndef __LIB_DELAY_H__
#define __LIB_DELAY_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PCLK1_CLOCK 36000000U
#define SYSCLOCK 72000000U

/* Exported macro ------------------------------------------------------------*/
#define TIMER_HIGH_BITS_CNT 18 // Сколько старших битов счетчика 24-хразрядного таймера использовать
//#define TICKS_PER_SECOND			(SYSCLOCK / (1<<(24-TIMER_HIGH_BITS_CNT)))
#define TICKS_PER_SECOND  (TICKS_PER_MILLISECOND*1000)	
//#define TICKS_PER_MILLISECOND		(TICKS_PER_SECOND / 1000)
#define TICKS_PER_MILLISECOND		(HAL_GetTickFreq())

/* Exported functions ------------------------------------------------------- */
void GetClocksFreq (LL_RCC_ClocksTypeDef *SysClock);
void delay_ms(uint16_t );
void delay_us(uint16_t );

void SysTick_Init(void);
static void GetRawXTick( uint32_t *, uint32_t *);
uint32_t GetXTick(void);
uint32_t GetXTickDiv1024(void);
void XDelayMs( uint32_t );

extern volatile uint32_t InternalTicks;

#endif /* __LIB_DELAY_H__ */

