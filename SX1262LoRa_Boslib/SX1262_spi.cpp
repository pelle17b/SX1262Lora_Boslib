#include "SX1262_spi.h"

uint8_t spiBuff[SPI_BUFFER_SIZE];   //Buffer for sending SPI commands to radio

SX1262_spi::SX1262_spi()
{
	
}

void SX1262_spi::spiInitialSetup()
{
	SPI.begin();
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	delay(10);

	//Initialize pins
	digitalWrite(SX1262_NSS, 1);  //High = inactive 
	pinMode(SX1262_NSS,OUTPUT);

	pinMode(SX1262_BUSY, INPUT);  //Busy pin is low when not busy
}

void SX1262_spi::writeRegister(uint16_t address, uint8_t value){
  waitUntilReady();
  digitalWrite(SX1262_NSS,0); //CS 
  delay(SPI_CS_DELAY);
  SPI.transfer(0x0D); //Write register
  //delay(SPI_DELAY);
  SPI.transfer16(address);
  //delay(SPI_DELAY);
  SPI.transfer(value);  
  digitalWrite(SX1262_NSS,1); //!CS
  waitUntilReady();
}

uint8_t SX1262_spi::readRegister(uint16_t address){
  uint8_t receivedVal = 0x00;
  waitUntilReady();
  digitalWrite(SX1262_NSS,0); //CS  
  delay(SPI_CS_DELAY);
  SPI.transfer(0x1D); //Read register
  //delay(SPI_DELAY);
  SPI.transfer16(address);
  //delay(SPI_DELAY);
  SPI.transfer(0x00);  //Dummy byte
  receivedVal = SPI.transfer(0x00);  //Read response
  digitalWrite(SX1262_NSS,1); //!CS
  waitUntilReady();
  return receivedVal;
}

void SX1262_spi::transferSPI(int nbrBytes){
  waitUntilReady();
  digitalWrite(SX1262_NSS,0); //CS 
  delay(SPI_CS_DELAY);
  SPI.transfer(spiBuff, nbrBytes);
  digitalWrite(SX1262_NSS,1); //Disable radio chip-select
  waitUntilReady();
}

void SX1262_spi::transferTransmitBuffer(int nbrBytes, char* txRxBuff){
  //WriteBuffer command
  //Transfer data to buffer
  waitUntilReady();
  digitalWrite(SX1262_NSS,0); //CS 
  delay(SPI_CS_DELAY);
  SPI.transfer(0x0E);
  SPI.transfer(0x00);
  for (int i = 0; i < nbrBytes; i++){
    SPI.transfer(txRxBuff[i]);
    delay(SPI_DELAY);
    }
  digitalWrite(SX1262_NSS,1); //Disable radio chip-select
  waitUntilReady();
}

void SX1262_spi::transferReceiveBuffer(int payloadLength, uint8_t startAddress, char* txRxBuff){
  //Read the radio buffer from the SX1262 into the user-supplied buffer
  for (int i = 0; i < TRANSMIT_RECEIVE_BUFFER_SIZE; i++){
    txRxBuff[i] = 0x00;
  }
  digitalWrite(SX1262_NSS,0); //CS
  delay(SPI_CS_DELAY);
  SPI.transfer(0x1E);
  SPI.transfer(startAddress);
  SPI.transfer(0x00);
  for (int i = 0; i < payloadLength; i++){
    txRxBuff[i] = SPI.transfer(txRxBuff[i]);
    delay(SPI_DELAY);
  }
  txRxBuff[payloadLength] = '\0'; //Add end of string character after message 
  digitalWrite(SX1262_NSS,1); //Disable radio chip-select
  waitUntilReady();
}

void SX1262_spi::waitUntilReady(){
  delay(1);
  while (digitalRead(SX1262_BUSY)){
    delay(1);
  }  
}