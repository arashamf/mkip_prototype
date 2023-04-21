// Includes -------------------------------------------------------------------------------------------//
#include "lib_delay.h"

//Private defines -------------------------------------------------------------------------------------//
#define GetHWTimerVal() (0xFFFFFFUL - SysTick->VAL) 
#define XTICK_TIMER_IRQn			SysTick_IRQn
#define XTICK_TIMER_IRQ_Handler()	void SysTick_Handler( void )
#define IsXTickTimerOver()			READ_BIT (SysTick->CTRL, SysTick_CTRL_COUNTFLAG_Msk) //Возвращает 1, если таймер отсчитал до 0 

//Private variables -----------------------------------------------------------------------------------//
volatile uint32_t InternalTicks = 0;

//-----------------------------------------------------------------------------------------------------//
void GetClocksFreq (LL_RCC_ClocksTypeDef *SysClock)
{
	LL_RCC_GetSystemClocksFreq(SysClock);
}

//-----------------------------------------------------------------------------------------------------//
void delay_ms(uint16_t delay)
{

	SET_BIT (RCC->APB1ENR, RCC_APB1ENR_TIM4EN); //тактирование таймера 4
	CLEAR_REG(TIM4->CNT);
	WRITE_REG (TIM4->PSC, (uint16_t)((PCLK1_CLOCK/1000) - 1)); //GetPCLK1Freq - частота шины таймера 10 						
	WRITE_REG (TIM4->ARR, (2*delay)-1); //время задержки
	SET_BIT (TIM4->EGR, TIM_EGR_UG); //запись новых настроек в таймер 
	CLEAR_BIT (TIM4->SR, TIM_SR_UIF);// сброс флага обновления
	
	SET_BIT (TIM4->CR1, TIM_CR1_CEN); //включение таймера
	while (!READ_BIT(TIM4->SR, TIM_SR_UIF)) {} //ожидание установки флага обновления таймера 
	CLEAR_BIT (TIM4->CR1, TIM_CR1_CEN);  //отключение таймера
	CLEAR_BIT (TIM4->SR, TIM_SR_UIF);
}

//-----------------------------------------------------------------------------------------------------//
void delay_us(uint16_t delay)
{
	SET_BIT (RCC->APB1ENR, RCC_APB1ENR_TIM4EN ); //тактирование таймера 4
	CLEAR_REG(TIM4->CNT);
	WRITE_REG (TIM4->PSC, (uint16_t)((PCLK1_CLOCK/1000000) - 1));  	 
	WRITE_REG (TIM4->ARR, 2*delay-1); //время задержки
	SET_BIT (TIM4->EGR, TIM_EGR_UG); //запись новых настроек в таймер 
	CLEAR_BIT (TIM4->SR, TIM_SR_UIF);// сброс флага обновления
	
	SET_BIT (TIM4->CR1, TIM_CR1_CEN); //включение таймера
	while (!READ_BIT(TIM4->SR, TIM_SR_UIF)) {} //ожидание установки флага обновления таймера 
	CLEAR_BIT (TIM4->CR1, TIM_CR1_CEN);  //отключение таймера
	CLEAR_BIT (TIM4->SR, TIM_SR_UIF);
}

//-----------------------------------------------------------------------------------------------------//
void SysTick_Init(void)
{
  WRITE_REG (SysTick->LOAD,0xFFFFFFUL); //значения от которого считает SysTick
  SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk); //в качестве источника SysTick системная частота SYSCLOCK 
	SET_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT_Msk); //разрешение прерываний
	SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE_Msk); //включение счётчика
	
	NVIC_SetPriority( SysTick_IRQn, 3 );
}

//-----------------------------------------------------------------------------------------------------//
__weak XTICK_TIMER_IRQ_Handler()
{
  InternalTicks++; // счетчик переполнений
}

//-----------------------------------------------------------------------------------------------------//
static void GetRawXTick( uint32_t *HighBits, uint32_t *LowBits )
{
	do
	{
		NVIC_EnableIRQ( XTICK_TIMER_IRQn );
		__NOP();
		__NOP();
		NVIC_DisableIRQ( XTICK_TIMER_IRQn );
		
		*LowBits = GetHWTimerVal(); //младшие биты содержат значение TIMER_HIGH_BITS_CNT старших битов текущего счетчика таймера
		*HighBits = InternalTicks; //старшие биты содержат значение (24-TIMER_HIGH_BITS_CNT) младших битов счетчика переполнений
	} 
	while( IsXTickTimerOver()); //ожидание пока таймер отсчитает до нуля

	NVIC_EnableIRQ( XTICK_TIMER_IRQn );
}

//-----------------------------------------------------------------------------------------------------//
uint32_t GetXTick( void )
{
  uint32_t highbits, lowbits;
  uint32_t result;

	GetRawXTick( &highbits, &lowbits );

	result = lowbits >> (24-TIMER_HIGH_BITS_CNT); //сдвиг на 18 разрядов влево
	result |= highbits << TIMER_HIGH_BITS_CNT; //сдвиг на 18 разрядов вправо

	return result;
}

//-----------------------------------------------------------------------------------------------------//
uint32_t GetXTickDiv1024( void )
{
  uint32_t highbits, lowbits;
  uint32_t result;

	GetRawXTick( &highbits, &lowbits );

	result = lowbits >> (24-(TIMER_HIGH_BITS_CNT-10));
	result |= highbits << (TIMER_HIGH_BITS_CNT-10);

	return result;
}

//-----------------------------------------------------------------------------------------------------//
void XDelayMs( uint32_t delay )
{
  uint32_t ticks;

	ticks = GetXTick();
	while( ( GetXTick() - ticks ) < TICKS_PER_MILLISECOND * delay );
}

//-----------------------------------------------------------------------------------------------------//

