#include "nrf24Set.h"
//#include "lib407.h"
#include "Sysconfig.h"

#define NRF24_CE_PORT_S GPIOA
#define NRF24_CE_PIN_S 15

#define NRF24_CE_PORT_UN GPIOD
#define NRF24_CE_PIN_UN 0

void setCEPin(void)
{
	pinSet(NRF24_CE_PORT_UN,generalOutput,veryHigh,NotFunction,pushpull,noPP,NRF24_CE_PIN_UN);

}

void CESelectPin(void)
{
	pinHigh(NRF24_CE_PORT_UN,NRF24_CE_PIN_UN);
}

void CEUnselectPin(void)
{
	pinLow(NRF24_CE_PORT_UN,NRF24_CE_PIN_UN);
}

void nrf24WriteReg(uint8_t Reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = Reg|NRF_CMD_W_REGISTER;
	buf[1] = data;
	CSEnable();
	SPI_Transmit(buf,2);
	CSDisable();
}

void nrf24WriteRegMulti(uint8_t Reg,uint8_t *data,uint8_t size)
{
	uint8_t buf[1];
	buf[0] = Reg|NRF_CMD_W_REGISTER;
	CSEnable();
	SPI_Transmit(buf,1);
	SPI_Transmit(data,size);
	CSDisable();
}

uint8_t nrf24ReadReg(uint8_t Reg)
{
	uint8_t data = 0;
	uint8_t buf[1];
	buf[0] = Reg|NRF_CMD_R_REGISTER;
	CSEnable();
	SPI_Transmit(buf,1);
	SPI_Receive(&data,1);
	CSDisable();
	return data;
}


void nrf24ReadRegMulti(uint8_t Reg,uint8_t *data,uint8_t size)
{
	uint8_t buf[1];
	buf[0] = Reg|NRF_CMD_R_REGISTER;
	CSEnable();
	SPI_Transmit(buf,1);
	SPI_Receive(data,size);
	CSDisable();
}

void nrf24Init(void) 
{
	setCEPin();
	CSEnable();
	CEUnselectPin();
	CSDisable();
	nrf24WriteReg(NRF_CONFIG, (1<<1));
	nrf24WriteReg(NRF_EN_AA,0);
	nrf24WriteReg(NRF_EN_RXADDR,0);
	nrf24WriteReg(NRF_SETUP_AW,0x03);
	nrf24WriteReg(NRF_SETUP_RETR,3);
	nrf24WriteReg(NRF_RF_CH,0);
	nrf24WriteReg(NRF_RF_SETUP,0x0F);
	CESelectPin();
	CSEnable();
}


void nrf24TXMode(uint8_t *Address,uint8_t channel)
{
	CEUnselectPin();
	nrf24WriteReg(NRF_RF_CH,channel);
	nrf24WriteRegMulti(NRF_TX_ADDR,Address,5);
	uint8_t config = nrf24ReadReg(NRF_CONFIG);
	config = config | (1<<1);
	nrf24WriteReg(NRF_CONFIG,config);
	CESelectPin();
}

uint8_t NRF24Transmit(uint8_t *data)
{
	CSEnable();
	uint8_t cmnd = NRF_CMD_W_TX_PAYLOAD;
	SPI_Transmit(&cmnd,1);
	SPI_Transmit(data,32);
	CSDisable();
	
	uint8_t fifostat = nrf24ReadReg(NRF_FIFO_STATUS);
	if((fifostat&(1<<4)) && (!(fifostat&(1<<3)))){
		uint8_t cmnd1 = NRF_CMD_FLUSH_TX; 
		SPI_Transmit(&cmnd1,1);
		return 1;
	}
	return 0;
}

void NRF24RXMode(uint8_t *Address,uint8_t channel)
{
	setCEPin();
	CEUnselectPin();
	CSDisable();
	uint8_t config = nrf24ReadReg(NRF_CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24WriteReg(NRF_CONFIG,config);
	nrf24WriteReg(NRF_RF_CH,channel);
	uint8_t rxaddr = nrf24ReadReg(NRF_EN_RXADDR);
	rxaddr = rxaddr|(1<<1);
	nrf24WriteReg(NRF_EN_RXADDR,rxaddr);
	nrf24WriteRegMulti(NRF_RX_ADDR_P1,Address,5);
	nrf24WriteReg(NRF_RX_PW_P1,32);
	CESelectPin();
	CSEnable();
}

void SPIControl()
{
	setCEPin();
	CSEnable();
	CEUnselectPin();
	CSDisable();
	nrf24WriteReg(NRF_CONFIG,(1<<1)|(1<<3));
	uint8_t control;
	nrf24WriteReg(NRF_RF_CH,0xA);
	delay_us(1000);
	control = nrf24ReadReg(NRF_RF_CH);
	if(control == 0xA){
		pinHigh(GPIOD,15);
	}
	else{
		pinHigh(GPIOD,12);
	}

	
}

void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF_CMD_FLUSH_RX;
    CSEnable();
    SPI_Transmit(&command,1);
    CSDisable();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF_CMD_FLUSH_TX;

    CSEnable();
    SPI_Transmit(&command,1);
    CSDisable();
}


void nrf24l01p_reset()
{
    // Reset pins
	  setCEPin();
    CSDisable();
    CEUnselectPin();

    // Reset registers
    nrf24WriteReg(NRF_CONFIG, 0x08);
	  nrf24WriteReg(NRF_EN_AA,0x3F);
		nrf24WriteReg(NRF_EN_RXADDR,0x03);
		nrf24WriteReg(NRF_SETUP_AW,0x03);
		nrf24WriteReg(NRF_SETUP_RETR,0x03);
		nrf24WriteReg(NRF_RF_CH,0x02);
		nrf24WriteReg(NRF_RF_SETUP,0x07);
		nrf24WriteReg(NRF_STATUS,0x7E);
		nrf24WriteReg(NRF_RX_PW_P0,0x00);

		
		
    // Reset FIFO
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void RXInit(uint8_t *Address,uint8_t channel){
	nrf24l01p_reset();
	delay_us(10000);
  nrf24WriteReg(NRF_CONFIG, 0x09);  // Power down
  nrf24WriteReg(NRF_EN_AA,0x00);
	nrf24WriteReg(NRF_SETUP_AW,0x03);
	nrf24WriteReg(NRF_SETUP_RETR, 0x00);	
	nrf24WriteReg(NRF_RF_CH,channel);
  nrf24WriteReg(NRF_RF_SETUP, 0x06);  
  nrf24WriteRegMulti(NRF_TX_ADDR, Address, 5);
	nrf24WriteRegMulti(NRF_RX_ADDR_P1,Address,5);
	nrf24WriteReg(NRF_RX_PW_P1,32);
	nrf24WriteReg(NRF_EN_RXADDR,0x02);
	nrf24WriteReg(NRF_STATUS, 0x70);
  nrf24l01p_flush_rx_fifo();
  nrf24WriteReg(NRF_CONFIG, 0x0B);
	delay_us(5000);	
	CESelectPin();
	CSDisable();
}

uint8_t isDataAvailable(int pipenum)
{
	uint8_t status = nrf24ReadReg(NRF_STATUS);
	if(status&(1<<6))
	{
		uint8_t pipe = (status>>1) & 0x07;
		if(pipe == pipenum){
				return 1;
		}
	}
	return 0;

}

void NRFReceive(uint8_t *data)
{
	nrf24WriteReg(NRF_STATUS,0x70);
	CSEnable();
	uint8_t cmnd2 = SPI_RW(NRF_CMD_R_RX_PAYLOAD);	
	
	for (int i = 0; i < 32; i++) {
			data[i] = SPI_RW(0xFF);
  }
	CSDisable();
	nrf24WriteReg(NRF_STATUS,(1<<6));
}



