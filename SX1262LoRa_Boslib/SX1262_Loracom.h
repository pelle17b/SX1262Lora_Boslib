#ifndef SX1262_Loracom_h
#define SX1262_Loracom_h

#include <SX1262_spi.h>

#include <Arduino.h>
#include <Lora.h>

struct chipModeStruct
{
  bool stbyRC;
  bool stbyXosc;
  bool Fs;
  bool Rx;
  bool Tx;
};

struct commandStatusStruct
{
  bool rxSuccessPacketReady;
  bool internalWatchdogTripped;
  bool invalidOpCodeOrWrongNumberOfParameters;
  bool couldNotExecuteCommand;
  bool txCurrentPackageTerminated;
};

struct errorsStruct 
{	
  bool rc64kCalibrationError;
  bool rc13mCalibrationError;
  bool pllCalibrationError;
  bool adcCalibrationError;
  bool imgCalibrationError;
  bool xoscStartError;
  bool pllLockError;
  bool paRampError;
  bool allOk;
};

class SX1262_Loracom
{
  public:
    SX1262_Loracom();
	char txRxBuff[TRANSMIT_RECEIVE_BUFFER_SIZE];   //Buffer for sending data
    void sx1262InitialSetup();
    void radioTransmit(int dataLen);
    void setModeReceive();
	int receive_async();
	errorsStruct getDeviceErrors();
	chipModeStruct getChipMode();
	commandStatusStruct getCommandStatus();
	int getLastReceivedRSSI();
	int getLastReceivedSNR();
	int getLastReceivedSignalRSSI();
	
  private:
	void resetSX1262();
	bool getIrqStatus();
	void printRxTxBuffer(int length);
	void waitUntilReady();
	void rfSwitch(int state);
	
};
#endif