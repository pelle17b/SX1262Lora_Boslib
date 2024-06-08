#ifndef SX1262_spi_h
#define SX1262_spi_h

#include <Arduino.h>
#include <Lora.h>
#include <SPI.h>

class SX1262_spi
{
  public:
	SX1262_spi();
	void spiInitialSetup();
	void writeRegister(uint16_t address, uint8_t value);
	uint8_t readRegister(uint16_t address);
	void transferSPI(int nbrBytes);
	void transferTransmitBuffer(int nbrBytes, char* txRxBuff);
	void transferReceiveBuffer(int payloadLength, uint8_t startAddress, char* txRxBuff);
	uint8_t spiBuff[SPI_BUFFER_SIZE];   //Buffer for sending SPI commands to radio
  private:
	void waitUntilReady();

};
#endif