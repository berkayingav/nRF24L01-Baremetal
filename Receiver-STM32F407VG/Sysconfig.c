#include "Sysconfig.h"
#include <stm32f407xx.h>

void SysClockConfig (void)
{
	#define PLL_M 8
	#define PLL_N 168
	#define PLL_P 2
	
	RCC->CR |= RCC_CR_HSEON;
	
	while(!(RCC->CR & RCC_CR_HSERDY));
	
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;
	
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV4;
	
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16 | RCC_PLLCFGR_PLLSRC_HSE);
		
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	
	
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	
}

void TIM6Config(void)
{
	RCC->APB1ENR |= (1<<4);
	
	TIM6->PSC = 90-1;
	TIM6->ARR = 0xffff - 1;
	
	TIM6->CR1 |= (1<<0);
	while(!(TIM6->SR & (1<<0)));
}

void delay_us (uint16_t us)
{
	TIM6->CNT = 0;
	while(TIM6->CNT < us);
}

void TIM7Config(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	TIM7->PSC = 0;
	TIM7->ARR = 0xffffffff;
	TIM7->CR1 |= (1<<0);
	while(!(TIM7->SR & (1<<0)));
}

void TIM7SetCounter(uint32_t value){
	TIM7->SR = 0;
	TIM7->CNT = value;

};

int TIM7GetCounter(){
	return TIM7->CNT;
}

