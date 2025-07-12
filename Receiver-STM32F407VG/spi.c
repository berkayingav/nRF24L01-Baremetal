#include "spi.h"

void SPI_Config(void)
{
	RCC->APB1ENR |= (1<<15);
	SPI3->CR1 = 0;

	SPI3->CR1 |=  (1<<2);
	SPI3->CR1 &= ~(1<<1);
	SPI3->CR1 &= ~(1<<0);
	SPI3->CR1 |= (2<<3);  // 42/8;
	SPI3->CR1 &= ~(1<<7);
	SPI3->CR1 |= (1<<8) | (1<<9);
	SPI3->CR1 &=~(1<<10);
	SPI3->CR1 &=~(1<<11);
}

void GPIO_Config(void)
{
	pinSet(GPIOC,alternateFunction,veryHigh,SPI3_,pushpull,noPP,10); // SCK Pin
	pinSet(GPIOC,alternateFunction,veryHigh,SPI3_,pushpull,noPP,11);//MISO
	pinSet(GPIOC,alternateFunction,veryHigh,SPI3_,pushpull,noPP,12);//MOSI
	pinSet(GPIOA,generalOutput,veryHigh,NotFunction,pushpull,noPP,15); // NSS

}

void SPIEnable()
{
	SPI3->CR1 |= (1<<6);
}
	
void SPIDisable()
{
	SPI3->CR1 &= ~(1<<6);
}	
	
void CSEnable()
{
	GPIOA->BSRR = (1<<(16 + 15));
}
	
void CSDisable()
{
	GPIOA->BSRR = (1<<(15));
}

void SPI_Transmit(uint8_t *data,int size)
{	
	int i = 0;
	while (i<size)
	{
			while (!(SPI3->SR & (1<<1))) {};
			SPI3->DR = data[i];
			i++;
	}
	
	
	while(!(SPI3->SR & (1<<1))) {};  // TXE = 1 olana kadar bekle
	while((SPI3->SR & (1<<7))) {};   //BSY = 1 oldugu sürece bekle
	uint8_t temp = SPI3->DR;
					temp = SPI3->SR;
}

void SPI_Receive(uint8_t *data, int size)
{
	while(size){
		while((SPI3->SR & (1<<7))) {};
		SPI3->DR = 0;
		while (!(SPI3->SR & (1<<0))) {};
		*data++ = (SPI3->DR);
		size--;
	}
}

void SPI_SendData(uint8_t data)
{
	SPI3->DR = data;
}

uint8_t SPI_ReceiveData(void)
{
	return ((uint8_t)SPI3->DR);
}

uint8_t SPI_RW(uint8_t data)
{
	while(!(SPI3->SR & SPI_SR_TXE)){};
	SPI_SendData(data);
	while(!(SPI3->SR & SPI_SR_RXNE)){};
	return SPI3->DR;
}