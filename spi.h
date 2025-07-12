#ifndef SPI_H_
#define SPI_H_

#include "lib407.h"

void SPI_Config(void);
void GPIO_Config(void);
void SPIEnable(void);
void SPIDisable(void);
void CSEnable(void);
void CSDisable(void);
void SPI_Transmit(uint8_t *data,int size);
void SPI_Receive(uint8_t *data, int size);
void SPI_SendData(uint8_t data);
uint8_t SPI_ReceiveData(void);
uint8_t SPI_RW(uint8_t data);

#endif