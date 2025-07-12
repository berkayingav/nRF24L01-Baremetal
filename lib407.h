#ifndef LIB407_H 
#define LIB407_H

#include <stm32f407xx.h>

#define DMA_STREAM_COUNT 8
#define DMA_CH_INVALID 0xFF

enum mcuFunction_e{
    MCU_SYS = 0,
		MCU_ADC1,
		MCU_ADC2,
	  MCU_ADC3,
		MCU_DAC1,
		MCU_DAC2,
    MCU_I2C1,
    MCU_I2C2,
    MCU_I2C3,
    MCU_SPI1,
    MCU_SPI2,
    MCU_SPI3,
    MCU_USART1,
    MCU_USART2,
    MCU_USART3,
    MCU_UART4,
    MCU_UART5,
    MCU_USART6,
    MCU_SDIO,
    MCU_DCMI,
    MCU_EVENTOUT,

    MCU_FUNC_COUNT,   // toplam üye sayisi
};

typedef struct{
	enum mcuFunction_e func;
	DMA_Stream_TypeDef *str;
	uint8_t channel;
}dmaMap;

static const dmaMap dmamap[] = {
	{MCU_ADC1,DMA2_Stream0,0},
	{MCU_ADC1,DMA2_Stream4,0},
	{MCU_ADC2,DMA2_Stream2,1},
	{MCU_ADC2,DMA2_Stream3,1},
	{MCU_ADC3,DMA2_Stream0,2},
	{MCU_ADC3,DMA2_Stream1,2},
	{MCU_DAC1,DMA1_Stream5,7},
	{MCU_DAC2,DMA1_Stream6,7},
	{MCU_I2C1,DMA1_Stream0,1},
	{MCU_I2C1,DMA1_Stream5,1},
	{MCU_I2C1,DMA1_Stream6,1},
	{MCU_I2C1,DMA1_Stream7,1},
	{MCU_I2C2,DMA1_Stream2,7},
	{MCU_I2C2,DMA1_Stream3,7},
	{MCU_I2C2,DMA1_Stream7,7},
	{MCU_I2C3,DMA1_Stream2,3},
	{MCU_I2C3,DMA1_Stream4,3},
	{MCU_SPI1,DMA2_Stream0,3},
	{MCU_SPI1,DMA2_Stream2,3},
	{MCU_SPI1,DMA2_Stream3,3},
	{MCU_SPI1,DMA2_Stream5,3},
	{MCU_SPI2,DMA2_Stream3,0},
	{MCU_SPI2,DMA2_Stream4,0},
	{MCU_SPI3,DMA1_Stream0,0},
	{MCU_SPI3,DMA1_Stream2,0},
	{MCU_SPI3,DMA1_Stream5,0},
	{MCU_SPI3,DMA1_Stream7,0},
	{MCU_USART1,DMA2_Stream2,4},
	{MCU_USART1,DMA2_Stream5,4},
	{MCU_USART1,DMA2_Stream7,4},
	{MCU_USART2,DMA1_Stream5,4},
	{MCU_USART2,DMA1_Stream6,4},
	{MCU_USART3,DMA1_Stream1,4},
	{MCU_USART3,DMA1_Stream3,4},
	{MCU_USART3,DMA1_Stream4,7},
	{MCU_UART4,DMA1_Stream2,4},
	{MCU_UART4,DMA1_Stream4,4},
	{MCU_UART5,DMA1_Stream0,4},
	{MCU_UART5,DMA1_Stream7,4},
	{MCU_USART6,DMA2_Stream1,5},
	{MCU_USART6,DMA2_Stream2,5},
	{MCU_USART6,DMA2_Stream6,5},
	{MCU_USART6,DMA2_Stream7,5},
  {MCU_SDIO,DMA2_Stream6,4}, 
	{MCU_SDIO,DMA2_Stream3,4},
	{MCU_DCMI,DMA2_Stream1,1},
	{MCU_DCMI,DMA2_Stream7,1},
};

enum speedValues
{
	 low,
	 medium,
	 high,
	 veryHigh
};

enum dmadataDirection
{
	Receiver,
	Transmitter,
  notDMA
};

enum gpioMode
{
	input,
	generalOutput,
	alternateFunction,
	analog
};

enum outputMode{
	pushpull,
	openDrain,
	NotOutput
};

enum mcuFunction
{
	system_ = 0,
	TIM1_ = 1,
	TIM2_= 1,
	TIM3_ = 2,
	TIM4_ = 2,
	TIM5_ = 2,
	TIM8_ = 3,
	TIM9_ = 3,
	TIM10_ = 3,
	TIM11_ =3,
	I2C1_ = 4,
	I2C2_ = 4,
	I2C3_ = 4,
	SPI1_ = 5,
	SPI2_ = 5,
	SPI3_ = 6,
	USART1_ = 7,
	USART2_ = 7,
	USART3_ = 7,
	UART4_ = 8,
	UART5_ = 8,
	USART6_ = 8,
	CAN1_ = 9,
	CAN2_ = 9,
	TIM12_ = 9,
	TIM13_ = 9,
	TIM14_ =9,
	OTG_FS_ = 10,
	OTG_HS_ = 10,
	ETH_ = 11,
	FSMC_ = 12,
	SDIO_ = 12,
	OTG_HS1_ = 12,
	DCMI_ = 13,
	EVENTOUT_ = 15,
	NotFunction
};

enum dataSize
{
	byte = 3,
	halfWord = 1,
	word = 2	
};

enum pullMode{
	noPP,
	pullUp,
	pullDown,
	NotPullMode
};

enum interruptFeature{
	interruptDisable = 0,
	interruptActive = 1
};

enum dmaFeature{
	dmaDisable = 0,
	dmaActive = 1
};

#define DMA_MAP_LEN  (int)(sizeof(dmamap)/sizeof(dmamap[0]))
#define DMA_CH_INVALID 0xFF


uint8_t getDmaChannel(enum mcuFunction_e func,DMA_Stream_TypeDef *stream);
void pinSet(GPIO_TypeDef *gpio,enum gpioMode gm,enum speedValues sv,enum mcuFunction mf,enum outputMode om,enum pullMode pm,uint8_t pin);
void ussartConfig(USART_TypeDef *usart,enum interruptFeature inf, enum dmaFeature df,enum dmadataDirection dd,uint32_t baudrate,uint16_t mcuFreq);
void usartSendBuffer(USART_TypeDef *usart,uint8_t *data,uint16_t size);
void usartGetBuffer(USART_TypeDef *usart,uint8_t *data,uint16_t size);
void dmaInit(DMA_TypeDef *dma,enum dmadataDirection dd,DMA_Stream_TypeDef *str,enum dataSize ds,enum mcuFunction_e mf);
void dmaConfig(DMA_Stream_TypeDef *stream,uint32_t srcAdd,uint32_t destAdd,uint16_t dataSize);
void pinHigh(GPIO_TypeDef *gpio,uint32_t pin);
void pinLow(GPIO_TypeDef *gpio,uint32_t pin);
void delay(volatile uint32_t d);
#endif
