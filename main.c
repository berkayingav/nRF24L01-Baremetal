#include "main.h"
#include <string.h>

uint8_t RxAddress[] = {0xE7,0xE7,0xE7,0xE7,0xE7};	
uint8_t RxData[32] = {0};  
uint8_t dataReceived = 0;

int main(void){

SysClockConfig();
TIM6Config();
SPI_Config();
GPIO_Config();
SPIEnable();  
pinSet(GPIOD,generalOutput,veryHigh,NotFunction,pushpull,noPP,14); //For Test
pinSet(GPIOD,generalOutput,veryHigh,NotFunction,pushpull,noPP,12); //For Test
pinSet(GPIOD,generalOutput,veryHigh,NotFunction,pushpull,noPP,15); //For Test
nrf24l01p_reset();
RXInit(RxAddress,10);

while(1){
				//SPIControl();		
				if(isDataAvailable(1) == 1){
					NRFReceive(RxData);
					pinHigh(GPIOD,15);
					pinLow(GPIOD,12);
					dataReceived = 1;   // Yeni veri flag'i
		
				}
				else{
					pinHigh(GPIOD,12);
					pinLow(GPIOD,15);   // Blue LED'i söndür

				}

		if(dataReceived == 1) {
        if(RxData[0] == 'A'){
            pinHigh(GPIOD,14);  // Red LED
        }
        else if(RxData[0] == 'B'){
            pinLow(GPIOD,14);
        }
				else{
					 nrf24l01p_flush_rx_fifo();
					 memset(RxData, 0, 32);
				}
        
        // Veri islendikten sonra temizle
        dataReceived = 0;
    }

}

return 0;



}
