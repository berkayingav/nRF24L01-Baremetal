

#ifndef NRF24SET_H_
#define NRF24SET_H_

#include "lib407.h"
#include "spi.h"

#define NRF_CONFIG		0x00
#define NRF_EN_AA		0x01
#define NRF_EN_RXADDR	0x02
#define NRF_SETUP_AW	0x03
#define NRF_SETUP_RETR	0x04
#define NRF_RF_CH		0x05
#define NRF_RF_SETUP	0x06
#define NRF_STATUS		0x07
#define NRF_OBSERVE_TX	0x08
#define NRF_CD			0x09
#define NRF_RX_ADDR_P0	0x0A
#define NRF_RX_ADDR_P1	0x0B
#define NRF_RX_ADDR_P2	0x0C
#define NRF_RX_ADDR_P3	0x0D
#define NRF_RX_ADDR_P4	0x0E
#define NRF_RX_ADDR_P5	0x0F
#define NRF_TX_ADDR		0x10
#define NRF_RX_PW_P0	0x11
#define NRF_RX_PW_P1	0x12
#define NRF_RX_PW_P2	0x13
#define NRF_RX_PW_P3	0x14
#define NRF_RX_PW_P4	0x15
#define NRF_RX_PW_P5	0x16
#define NRF_FIFO_STATUS	0x17
#define NRF_DYNPD		0x1C
#define NRF_FEATURE		0x1D

/* Commands */
#define NRF_CMD_R_REGISTER			0x00
#define NRF_CMD_W_REGISTER			0x20
#define NRF_CMD_R_RX_PAYLOAD		0x61
#define NRF_CMD_W_TX_PAYLOAD		0xA0
#define NRF_CMD_FLUSH_TX			0xE1
#define NRF_CMD_FLUSH_RX			0xE2
#define NRF_CMD_REUSE_TX_PL			0xE3
#define NRF_CMD_ACTIVATE			0x50
#define NRF_CMD_R_RX_PL_WID			0x60
#define NRF_CMD_W_ACK_PAYLOAD		0xA8
#define NRF_CMD_W_TX_PAYLOAD_NOACK	0xB0
#define NRF_CMD_NOP					0xFF

void setCEPin(void);
void CESelectPin(void);
void CEUnselectPin(void);
void nrf24WriteReg(uint8_t Reg, uint8_t data);
void nrf24WriteRegMulti(uint8_t Reg,uint8_t *data,uint8_t size);
uint8_t nrf24ReadReg(uint8_t Reg);
void nrf24ReadRegMulti(uint8_t Reg,uint8_t *data,uint8_t size);
void nrf24Init(void);
void nrf24TXMode(uint8_t *Address,uint8_t channel);
uint8_t NRF24Transmit(uint8_t *data);
void NRF24RXMode(uint8_t *Address,uint8_t channel);
uint8_t isDataAvailable(int pipenum);
void NRFReceive(uint8_t *data);
void RXInit(uint8_t *Address,uint8_t channel);
void nrf24l01p_reset();
void nrf24l01p_flush_tx_fifo();
void nrf24l01p_flush_rx_fifo();
void SPIControl();





#endif 