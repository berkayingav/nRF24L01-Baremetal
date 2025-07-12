//This section use for Arduino NANO - Transmitter.

 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

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

#define CE_PIN 9
#define CSN_PIN 10
#define BUTTON 12


void CSEnable(){
  digitalWrite(CSN_PIN,LOW);
}

void CSDisable(){
  digitalWrite(CSN_PIN,HIGH);
}

void CESelect(){
  digitalWrite(CE_PIN,LOW);
}

void CEUnselect(){
  digitalWrite(CE_PIN,HIGH);
}

void nrf24WriteReg(uint8_t reg,uint8_t data){
  uint8_t buf[2];
  buf[0] = reg|NRF_CMD_W_REGISTER;
  buf[1] = data;
  CSEnable();
  SPI.transfer(buf,2);
  CSDisable();
}

void nrf24WriteRegMulti(uint8_t reg,uint8_t *data,uint8_t size){
  uint8_t buf[1];
  buf[0] = reg|NRF_CMD_W_REGISTER;
  CSEnable();
  SPI.transfer(buf,1);
  SPI.transfer(data,size);
  CSDisable();
}

uint8_t nrf24ReadReg(uint8_t reg){
  uint8_t data = 0;
  uint8_t buf[1];
  buf[0] = reg|NRF_CMD_R_REGISTER;
  CSEnable();
  SPI.transfer(buf,1); //write
  data = SPI.transfer(0x00);//reading
  CSDisable();
  return data;
}

void nrf24ReadRegMulti(uint8_t reg,uint8_t *data, uint8_t size){
  uint8_t buf[1];
  buf[0] = reg|NRF_CMD_R_REGISTER;
  CSEnable();
  SPI.transfer(buf,1);
  for(int i = 0;i<size;i++){
    data[i] = SPI.transfer(0x00);
  }
  CSDisable();
}

void nrf24flushrxfifo(){
  CSEnable();
  SPI.transfer(NRF_CMD_FLUSH_RX);
  CSDisable();
}

void nrf24flushtxfifo(){
  CSEnable();
  SPI.transfer(NRF_CMD_FLUSH_TX);
  CSDisable();
}

void nrf24Reset(){
  CSDisable();
  CEUnselect();
  nrf24WriteReg(NRF_CONFIG,0x08);
  nrf24WriteReg(NRF_EN_AA,0x3F);
  nrf24WriteReg(NRF_EN_RXADDR,0x03);
  nrf24WriteReg(NRF_SETUP_AW,0x03);
  nrf24WriteReg(NRF_SETUP_RETR,0x03);
  nrf24WriteReg(NRF_RF_CH,0x02);
  nrf24WriteReg(NRF_RF_SETUP,0x07);
  nrf24WriteReg(NRF_STATUS,0x7E);
  nrf24WriteReg(NRF_RX_PW_P0,0x00);
	nrf24WriteReg(NRF_RX_PW_P1,0x00);
	nrf24WriteReg(NRF_RX_PW_P2,0x00);
	nrf24WriteReg(NRF_RX_PW_P3,0x00);
	nrf24WriteReg(NRF_RX_PW_P4,0x00);
	nrf24WriteReg(NRF_RX_PW_P5,0x00);
	nrf24WriteReg(NRF_FIFO_STATUS,0x11);
	nrf24WriteReg(NRF_DYNPD,0x00);
	nrf24WriteReg(NRF_FEATURE,0x00);  
  nrf24flushrxfifo();
  nrf24flushrxfifo();
}

void TXInit(uint8_t *Address,uint8_t channel){
	nrf24Reset();
  delay(2);
  nrf24WriteReg(NRF_CONFIG, 0x08);  // Power down
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
  nrf24flushtxfifo();

  nrf24WriteReg(NRF_CONFIG, 0x0A);  // PWR_UP=1, PRIM_RX=0
  delay(5);  // Power up delay
	CESelect();
  CSDisable();
}

uint8_t NRF24Transmit(uint8_t *data,uint8_t len)
{
  nrf24WriteReg(NRF_STATUS, 0x70);

  CSEnable();
  
  SPI.transfer(NRF_CMD_W_TX_PAYLOAD);
  for(int i = 0;i<32;i++){
    SPI.transfer(data[i]);
  }
  CSDisable();

  CESelect();    // CE HIGH
  delayMicroseconds(10);  // Min 10us pulse
  CEUnselect();

  uint8_t status;
  uint32_t timeout = millis() + 100;  // 100ms timeout

  nrf24WriteReg(NRF_STATUS, (1<<4));

  return 1;
}

const byte address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
const uint8_t channel = 10;


void setup() {
  Serial.begin(115200);
  pinMode(12,INPUT);
  pinMode(CE_PIN, OUTPUT);
  pinMode(CSN_PIN, OUTPUT); 
  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
  TXInit(address,channel);
  delay(1500);

}

void loop() {
  uint8_t payload[32];
  memset(payload, 0, 32);
  payload[0] = 'A';
  payload[1] = 'B';
  payload[2] = 'C';

  if(NRF24Transmit(payload,32) == 1){
    Serial.println("Gönderim başarılı");
    Serial.println(payload[0]);
  }
  else{
    Serial.println("Başarısız");
  }
  delay(500);
}
