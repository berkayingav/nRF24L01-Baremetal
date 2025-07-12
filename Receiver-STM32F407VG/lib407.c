#include "lib407.h"

uint8_t getDmaChannel(enum mcuFunction_e func,DMA_Stream_TypeDef *stream)
{
	for(int i = 0; i < DMA_MAP_LEN;i++){
		if(dmamap[i].func == func && dmamap[i].str == stream){
			return dmamap[i].channel;
		}
	}
	return DMA_CH_INVALID;	
}


void pinSet(GPIO_TypeDef *gpio,enum gpioMode gm,enum speedValues sv,enum mcuFunction mf,enum outputMode om,enum pullMode pm,uint8_t pin)
{
	if(gpio == GPIOA){
		RCC->AHB1ENR |= (1<<0);
	}
	else if(gpio == GPIOB){
		RCC->AHB1ENR |= (1<<1);
	}
	else if(gpio == GPIOC){
		RCC->AHB1ENR |= (1<<2);
	}
	else if(gpio == GPIOD){
		RCC->AHB1ENR |= (1<<3);
	}
	else if(gpio == GPIOE){
		RCC->AHB1ENR |= (1<<4);
	}
	else if(gpio == GPIOF){
		RCC->AHB1ENR |= (1<<5);
	}
	else if(gpio == GPIOG){
		RCC->AHB1ENR |= (1<<6);
	}
	else if(gpio == GPIOH){
		RCC->AHB1ENR |= (1<<7);
	}
	else if(gpio == GPIOI){
		RCC->AHB1ENR |= (1<<8);
	}
	
	gpio->MODER &= ~(3U << (pin*2));        // önce 00 yap
	gpio->MODER |= (gm<<(pin*2));
	gpio->OSPEEDR &= ~(3U << (pin*2));
	gpio->OSPEEDR |= (sv<<(pin*2));
	
	if(gm == alternateFunction){
		
		if(pin>7){
			gpio->AFR[1] = (gpio->AFR[1] & ~(0xF << ((pin-8)*4))) | (mf << ((pin-8)*4));   // (mf << ((pin-8)*4));
		}
		else{
			gpio->AFR[0] = (gpio->AFR[0] & ~(0xF << (pin*4))) |(mf<<((pin)*4));
		}	
	}
	
	
	/*if(gm == generalOutput || gm==alternateFunction)
	{
		if(om == pushpull || om == openDrain){
      gpio->OTYPER &= ~(1U << pin);			
			gpio->OTYPER |= (om<<pin);
		}
		
	}*/
	
	if(om == openDrain){
		gpio->OTYPER  &= ~(om << pin);
		gpio->OTYPER  |= (om << pin);   // push-pull
	}
	else{
		gpio->OTYPER  &= ~(1 << pin);   // push-pull
	}
	
	if(pm != NotPullMode){
		  gpio->PUPDR = (gpio->PUPDR & ~(3U <<(pin*2))) | (pm << (pin*2));
	}
	
	// sadece analogta göz ardi ediliyor pull up-pull down 
	
}

void pinHigh(GPIO_TypeDef *gpio,uint32_t pin){
		gpio->BSRR = (1<<pin);
}

void pinLow(GPIO_TypeDef *gpio,uint32_t pin){
	 gpio->BSRR = (1<<(pin + 16));
}

void ussartConfig(USART_TypeDef *usart,enum interruptFeature inf,enum dmaFeature df,enum dmadataDirection dd,uint32_t baudrate,uint16_t mcuFreq)
{
	if(usart == USART1){
	  RCC->APB2ENR |= (1<<4);
    pinSet(GPIOA,alternateFunction,high,USART1_,pushpull,noPP,9);
		pinSet(GPIOA,alternateFunction,high,USART1_,pushpull,noPP,10);
		NVIC_EnableIRQ(USART1_IRQn);
	}
	else if(usart == USART2){
		RCC->APB1ENR |= (1<<17);
	  pinSet(GPIOA,alternateFunction,high,USART2_,pushpull,noPP,2);
		pinSet(GPIOA,alternateFunction,high,USART2_,pushpull,noPP,3);
		NVIC_EnableIRQ(USART2_IRQn);
	}
	else if(usart == USART3){
		RCC->APB1ENR |= (1<<18);
		pinSet(GPIOB,alternateFunction,high,USART3_,pushpull,noPP,10);
		pinSet(GPIOB,alternateFunction,high,USART3_,pushpull,noPP,11);
		NVIC_EnableIRQ(USART3_IRQn);
	}
	else if(usart == UART4){
	  RCC->APB1ENR |= (1<<19);
		pinSet(GPIOA,alternateFunction,high,UART4_,pushpull,noPP,0);
		pinSet(GPIOA,alternateFunction,high,UART4_,pushpull,noPP,1);
		NVIC_EnableIRQ(UART4_IRQn);
	}
	else if(usart == UART5){
		RCC->APB1ENR |= (1<<20);
		pinSet(GPIOC,alternateFunction,high,UART5_,pushpull,noPP,12);
		pinSet(GPIOD,alternateFunction,high,UART5_,pushpull,noPP,2);
		NVIC_EnableIRQ(UART5_IRQn);
	}
	else if(usart == USART6){
		RCC->APB2ENR |= (1<<5);
		pinSet(GPIOC,alternateFunction,high,USART6_,pushpull,noPP,6);
		pinSet(GPIOC,alternateFunction,high,USART6_,pushpull,noPP,7);
		NVIC_EnableIRQ(USART6_IRQn);
	}
	
  usart->CR1 &= ~((1 << 12) | (1 << 13));  
	usart->CR1 &= ~(1<<12);  // M bit
	//For interrupt
	
	if(inf){
		usart->CR1 |= (1<<5) | (1<<7);
	}
	else{
		usart->CR1 &= ~(1<<5);
		usart->CR1 &= ~(1<<7);
	}
	
	if(df){
	if(dd == Receiver){
		usart->CR3 |= (1<<6);
	}
	else if(dd == Transmitter){
		usart->CR3 |= (1<<7);
	}
	}
	else{
		usart->CR3 &= ~(3<<6);
	}
	
	volatile float ftemp = (mcuFreq*1000000.0f)/(baudrate*16.0f);
	volatile uint32_t mantissa  = ftemp;
	volatile uint32_t fraction = ((ftemp - mantissa)*16.0f);
	usart->BRR = (fraction<<0) | (mantissa<<4);
	
	usart->CR1 |= (1<<6) | (1<<3) | (1<<2) ;
	usart->CR1 |= (1<<13);   // UE bit  

	
}

void usartSendBuffer(USART_TypeDef *usart,uint8_t *data,uint16_t size){
	for(uint16_t i = 0; i<size; i++){
		while(!(usart->SR & (1<<7)));
		usart->DR = data[i];
		
	}
	while(!(usart->SR & (1<<6)));
}

void usartGetBuffer(USART_TypeDef *usart,uint8_t *data,uint16_t size)
{
	for(uint16_t i = 0;i<size;i++){
		while(!(usart->SR & (1<<5)));
		data[i] = usart->DR;
	}
}

void dmaInit(DMA_TypeDef *dma,enum dmadataDirection dd,DMA_Stream_TypeDef *str,enum dataSize ds,enum mcuFunction_e mf){
	
	//getDmaChannel
	 str->CR  = 0;
   str->FCR = DMA_SxFCR_DMDIS;
	
	if(dma == DMA1){
		RCC->AHB1ENR |= (1<<21);
		if      (str == DMA1_Stream0) DMA1->LIFCR = DMA_LIFCR_CTCIF0|DMA_LIFCR_CHTIF0|DMA_LIFCR_CTEIF0|DMA_LIFCR_CDMEIF0|DMA_LIFCR_CFEIF0;
    else if (str == DMA1_Stream1) DMA1->LIFCR = DMA_LIFCR_CTCIF1|DMA_LIFCR_CHTIF1|DMA_LIFCR_CTEIF1|DMA_LIFCR_CDMEIF1|DMA_LIFCR_CFEIF1;
    else if (str == DMA1_Stream2) DMA1->LIFCR = DMA_LIFCR_CTCIF2|DMA_LIFCR_CHTIF2|DMA_LIFCR_CTEIF2|DMA_LIFCR_CDMEIF2|DMA_LIFCR_CFEIF2;
    else if (str == DMA1_Stream3) DMA1->LIFCR = DMA_LIFCR_CTCIF3|DMA_LIFCR_CHTIF3|DMA_LIFCR_CTEIF3|DMA_LIFCR_CDMEIF3|DMA_LIFCR_CFEIF3;
    else if (str == DMA1_Stream4) DMA1->HIFCR = DMA_HIFCR_CTCIF4|DMA_HIFCR_CHTIF4|DMA_HIFCR_CTEIF4|DMA_HIFCR_CDMEIF4|DMA_HIFCR_CFEIF4;
    else if (str == DMA1_Stream5) DMA1->HIFCR = DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5;
    else if (str == DMA1_Stream6) DMA1->HIFCR = DMA_HIFCR_CTCIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTEIF6|DMA_HIFCR_CDMEIF6|DMA_HIFCR_CFEIF6;
    else if (str == DMA1_Stream7) DMA1->HIFCR = DMA_HIFCR_CTCIF7|DMA_HIFCR_CHTIF7|DMA_HIFCR_CTEIF7|DMA_HIFCR_CDMEIF7|DMA_HIFCR_CFEIF7;
	}
	else if(dma == DMA2){
		RCC->AHB1ENR |= (1<<22);
    if      (str == DMA2_Stream0) DMA2->LIFCR = DMA_LIFCR_CTCIF0|DMA_LIFCR_CHTIF0|DMA_LIFCR_CTEIF0|DMA_LIFCR_CDMEIF0|DMA_LIFCR_CFEIF0;
    else if (str == DMA2_Stream1) DMA2->LIFCR = DMA_LIFCR_CTCIF1|DMA_LIFCR_CHTIF1|DMA_LIFCR_CTEIF1|DMA_LIFCR_CDMEIF1|DMA_LIFCR_CFEIF1;
    else if (str == DMA2_Stream2) DMA2->LIFCR = DMA_LIFCR_CTCIF2|DMA_LIFCR_CHTIF2|DMA_LIFCR_CTEIF2|DMA_LIFCR_CDMEIF2|DMA_LIFCR_CFEIF2;
    else if (str == DMA2_Stream3) DMA2->LIFCR = DMA_LIFCR_CTCIF3|DMA_LIFCR_CHTIF3|DMA_LIFCR_CTEIF3|DMA_LIFCR_CDMEIF3|DMA_LIFCR_CFEIF3;
    else if (str == DMA2_Stream4) DMA2->HIFCR = DMA_HIFCR_CTCIF4|DMA_HIFCR_CHTIF4|DMA_HIFCR_CTEIF4|DMA_HIFCR_CDMEIF4|DMA_HIFCR_CFEIF4;
    else if (str == DMA2_Stream5) DMA2->HIFCR = DMA_HIFCR_CTCIF5|DMA_HIFCR_CHTIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5;
    else if (str == DMA2_Stream6) DMA2->HIFCR = DMA_HIFCR_CTCIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTEIF6|DMA_HIFCR_CDMEIF6|DMA_HIFCR_CFEIF6;
    else if (str == DMA2_Stream7) DMA2->HIFCR = DMA_HIFCR_CTCIF7|DMA_HIFCR_CHTIF7|DMA_HIFCR_CTEIF7|DMA_HIFCR_CDMEIF7|DMA_HIFCR_CFEIF7;		
	};
	
	str->CR |= ( (1<<10) | (1<<8) | (1<<4) | (1<<3) | (1<<2) | (1<<1));
	
	if(dd == Receiver){
		str->CR &= ~(3<<6);
	}
	else if(dd == Transmitter){
		str->CR |= (1<<6);
	}
	
	if(ds == byte){
		str->CR &= ~(ds<<11);
		str->CR &= ~(ds<<13);
	}
	else{
		str->CR |= (ds<<11);
		str->CR |= (ds<<13);
	}
	
	str->CR |= (getDmaChannel(mf,str)<<25);
	
}

void dmaConfig(DMA_Stream_TypeDef *stream,uint32_t srcAdd,uint32_t destAdd,uint16_t dataSize)
{
	stream->NDTR = dataSize;
	stream->PAR = srcAdd;
	stream-> M0AR = destAdd; 
	
	//Double buffer mod kesintisiz veri akisi için kullanilabilir M1AR CR_DMB Registerinden denenebilir 
	
	stream->CR |= (1<<0);
}

void delay(volatile uint32_t d) {
    while (d--) {
        __NOP();
    }
}
