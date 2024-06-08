#include <Arduino.h>
#include <SX1262_Loracom.h>

#define DEBUG_LORA true

char  txRxBuff[TRANSMIT_RECEIVE_BUFFER_SIZE];   //Buffer for sending data

int lastReceivedRSSI = 0;
int lastReceivedSNR = 0;
int lastReceivedSignalRSSI = 0;



SX1262_spi loraSpi;

SX1262_Loracom::SX1262_Loracom()
{
	
}

void SX1262_Loracom::sx1262InitialSetup(){

  loraSpi.spiInitialSetup();

  //Initialize pins
  digitalWrite(SX1262_RESET, 1);  //High = inactive
  pinMode(SX1262_RESET,OUTPUT);

  digitalWrite(SX1262_RXEN, 1);  //High = Inactive
  pinMode(SX1262_RXEN,OUTPUT);
  digitalWrite(SX1262_TXEN, 1);  //High = Inactive
  pinMode(SX1262_TXEN,OUTPUT);

  pinMode(SX1262_DIO1, INPUT);  //Radio interrupt pin.  Goes high when we receive a packet
  pinMode(SX1262_DIO2, INPUT);  //Radio interrupt pin.  Goes high when we transmitted a packet
  pinMode(SX1262_BUSY, INPUT);  //Busy pin is low when not busy

  // Setup radio
  resetSX1262();

  // Set standby mode
  loraSpi.g_spiBuff[0] = 0x80;
  loraSpi.g_spiBuff[1] = 0x00; 
  loraSpi.transferSPI(2);

// Clear device errors caused by using extern TCXO
  loraSpi.g_spiBuff[0] = 0x07;          
  loraSpi.g_spiBuff[1] = 0x00;          
  loraSpi.g_spiBuff[2] = 0x00;          
  loraSpi.transferSPI(3);
  
  // Set packet type to Lora
  loraSpi.g_spiBuff[0] = 0x8A;
  loraSpi.g_spiBuff[1] = 0x01; //Lora
  loraSpi.transferSPI(2);

 // Set DIO3 to controll osc
  loraSpi.g_spiBuff[0] = 0x97;
  loraSpi.g_spiBuff[1] = TCXO_VOLTAGE;
  loraSpi.g_spiBuff[2] = 0x00;
  loraSpi.g_spiBuff[3] = 0x0F;
  loraSpi.g_spiBuff[4] = 0xFF;
  loraSpi.transferSPI(5);

  // Call for calibratiom since extern TCXO
  loraSpi.g_spiBuff[0] = 0x98;
  loraSpi.g_spiBuff[1] = 0xD7;
  loraSpi.g_spiBuff[2] = 0xDB;
  loraSpi.transferSPI(3);

  // Set standby mode
  loraSpi.g_spiBuff[0] = 0x80;
  loraSpi.g_spiBuff[1] = 0x01; 
  loraSpi.transferSPI(2);

  //Workaround to avoid overprotection when antenna mismatch. According to 15.2.2 Workaround
  uint8_t regValOP = loraSpi.readRegister(0x08D8);
  loraSpi.writeRegister(0x08D8, regValOP | 0x1E); 

  //Increase RX gain
  loraSpi.writeRegister(0x029F, 0x01); //Data retention after warm start
  loraSpi.writeRegister(0x02A0, 0x08); //Data retention after warm start
  loraSpi.writeRegister(0x02A1, 0xAC); //Data retention after warm start
  loraSpi.writeRegister(0x08AC, SX1262_RX_BOOSTED_GAIN); //Set gain

  //Set radio frequency 868MHz
  uint32_t pllFrequency = 910163968; //910163968
  loraSpi.g_spiBuff[0] = 0x86; 
  loraSpi.g_spiBuff[1] = (pllFrequency >> 24) & 0xFF;  //MSB of pll frequency
  loraSpi.g_spiBuff[2] = (pllFrequency >> 16) & 0xFF;  //
  loraSpi.g_spiBuff[3] = (pllFrequency >>  8) & 0xFF;  //
  loraSpi.g_spiBuff[4] = (pllFrequency >>  0) & 0xFF;  //LSB 
  loraSpi.transferSPI(5);  

  // Set PA parameters +22dBm
  loraSpi.g_spiBuff[0] = 0x95;
  loraSpi.g_spiBuff[1] = PA_DUTY_CYCLE;
  loraSpi.g_spiBuff[2] = PA_HP_MAX;
  loraSpi.g_spiBuff[3] = PA_DEVICE;
  loraSpi.g_spiBuff[4] = PA_LUT;
  loraSpi.transferSPI(5);
  
  // Set TX parameters
  loraSpi.g_spiBuff[0] = 0x8E;
  loraSpi.g_spiBuff[1] = TX_PARAMS_POWER;
  loraSpi.g_spiBuff[2] = TX_PARAMS_RAMP_TIME;
  loraSpi.transferSPI(3);

  // Set send/receive buffer base address
  loraSpi.g_spiBuff[0] = 0x8F;
  loraSpi.g_spiBuff[1] = 0x00;
  loraSpi.g_spiBuff[2] = 0x00;
  loraSpi.transferSPI(3);

  //Workaround for 500kHz BW sensitivity. Manual 15.1 Modulation Quality with 500 kHz LoRa Bandwidth
  if (SX1262_MODULATION_PARAMETERS_BANDWIDTH == BW500){
    uint8_t regValBW500 = loraSpi.readRegister(0x0889);
    loraSpi.writeRegister(0x0889, regValBW500 & 0xFB); 
  }
  else {
    uint8_t regValBW500 = loraSpi.readRegister(0x0889);
    loraSpi.writeRegister(0x0889, regValBW500 | 0x04); 
  }

  // SetModulationParameters
  loraSpi.g_spiBuff[0] = 0x8B;                
  loraSpi.g_spiBuff[1] = SX1262_MODULATION_PARAMETERS_SPREADING_FACTOR;               
  loraSpi.g_spiBuff[2] = SX1262_MODULATION_PARAMETERS_BANDWIDTH;
  loraSpi.g_spiBuff[3] = SX1262_MODULATION_PARAMETERS_CODING_FACTOR;
  loraSpi.g_spiBuff[4] = SX1262_MODULATION_PARAMETERS_LOW_DATARATE_OPTIMIZE;
  loraSpi.transferSPI(5);

  // SetDioIrqParams
  loraSpi.g_spiBuff[0] = 0x08;
  loraSpi.g_spiBuff[1] = 0xFF;
  loraSpi.g_spiBuff[2] = 0xFF;
  loraSpi.g_spiBuff[3] = 0x00; 
  loraSpi.g_spiBuff[4] = 0x02; 
  loraSpi.g_spiBuff[5] = 0x00;
  loraSpi.g_spiBuff[6] = 0x01; 
  loraSpi.g_spiBuff[7] = 0x00;
  loraSpi.g_spiBuff[8] = 0x00;
  loraSpi.transferSPI(9);
  
}

void SX1262_Loracom::resetSX1262(){
  delay(10);
  digitalWrite(SX1262_RESET, 0); 
  delay(10);
  digitalWrite(SX1262_RESET, 1); 
  delay(10);
  waitUntilReady();
  Serial.println(F("SX1262 Reset"));

  //Read a couple of registers to check SPI communication 
  if (loraSpi.readRegister(0x0740) != 0x14 || loraSpi.readRegister(0x0741) != 0x24){
    Serial.println(F("SPI communication error"));
    while (true);
  }
  else{
    Serial.println(F("SPI communication OK"));
  }
  return;
}

void SX1262_Loracom::waitUntilReady(){
  delay(1);
  while (digitalRead(SX1262_BUSY)){
    delay(1);
  } 
  return; 
}

void SX1262_Loracom::rfSwitch(int state){
  if (state == RF_SWITCH_OFF){
    //Set RF switch to OFF
    digitalWrite(SX1262_TXEN, 0);
    digitalWrite(SX1262_RXEN, 1);
  }
  else if (state == RF_SWITCH_RX){
  //Set RF switch to Receiver
    digitalWrite(SX1262_TXEN, 0);
    digitalWrite(SX1262_RXEN, 0);
  }
  else if (state == RF_SWITCH_TX){
    //Set RF switch to Transmitter
    digitalWrite(SX1262_TXEN, 1);
    digitalWrite(SX1262_RXEN, 1);
  }
  return;
}

void SX1262_Loracom::radioTransmit(int dataLen){
  if (dataLen > MAX_TRANSMIT_DATA_LENGTH) 
  { 
    dataLen = MAX_TRANSMIT_DATA_LENGTH;
    Serial.println(F("##ERROR## Transmit data length is longer than MAX_TRANSMIT_DATA_LENGTH. Truncated! "));
  }
  if (dataLen > TRANSMIT_RECEIVE_BUFFER_SIZE) 
  { 
    dataLen = TRANSMIT_RECEIVE_BUFFER_SIZE;
    Serial.println(F("##ERROR## Transmit data length is longer than TRANSMIT_RECEIVE_BUFFER_SIZE. Truncated! "));
   }

  if (DEBUG_LORA)
  {
    Serial.print(F("Transmit: "));
    Serial.println(txRxBuff); 
  }

  // Set standby mode 
  loraSpi.g_spiBuff[0] = 0x80;
  loraSpi.g_spiBuff[1] = 0x01;
  loraSpi.transferSPI(2);

  //Send transmit buffer
  loraSpi.transferTransmitBuffer(dataLen, txRxBuff);

  //Set RF switch to Transmitter
  rfSwitch(RF_SWITCH_TX);

  //Packet parameters IQ workaround
  uint8_t regValIQ = loraSpi.readRegister(0x0736);
  if (SX1262_PACKET_PARAMETERS_INVERT_IQ == PACKET_PARAMETERS_IQ_INVERTED){  
    loraSpi.writeRegister(0x0736, regValIQ & 0xFB); 
  }
  else if (SX1262_PACKET_PARAMETERS_INVERT_IQ == PACKET_PARAMETERS_IQ_NORMAL) {
    loraSpi.writeRegister(0x0736, regValIQ | 0x04); 
  }  

  //SetPacketParameters
  loraSpi.g_spiBuff[0] = 0x8C;          
  loraSpi.g_spiBuff[1] = SX1262_PACKET_PARAMETERS_PREAMBLE_LENGTH_MSB;
  loraSpi.g_spiBuff[2] = SX1262_PACKET_PARAMETERS_PREAMBLE_LENGTH_MSB;
  loraSpi.g_spiBuff[3] = SX1262_PACKET_PARAMETERS_HEADER_TYPE;
  loraSpi.g_spiBuff[4] = dataLen; //Payload Length (Max is 255 bytes)
  loraSpi.g_spiBuff[5] = SX1262_PACKET_PARAMETERS_CRC_ON_OFF;
  loraSpi.g_spiBuff[6] = SX1262_PACKET_PARAMETERS_INVERT_IQ;
  loraSpi.transferSPI(7);
  
  //Transmit
  loraSpi.g_spiBuff[0] = 0x83;          
  loraSpi.g_spiBuff[1] = SX1262_TRANSMIT_TIMEOUT_MSB;
  loraSpi.g_spiBuff[2] = SX1262_TRANSMIT_TIMEOUT_MID;
  loraSpi.g_spiBuff[3] = SX1262_TRANSMIT_TIMEOUT_LSB;
  loraSpi.transferSPI(4);

  delay(10);
  uint32_t startTransmitTime = millis();
  Serial.println(F("Transmitting"));
  
  //Radio pin DIO2 (interrupt) goes high when we have transmitted. 
  uint32_t startTime = millis();
  while (digitalRead(SX1262_DIO2) == false && millis() - startTime <= TRANSMIT_TIMEOUT); 
  if (digitalRead(SX1262_DIO2) == false){
      Serial.println(F("Transmit error timer expired")); 
    }
  getIrqStatus();
  //Tell the radio to clear the interrupt, and set the pin back inactive.
  loraSpi.g_spiBuff[0] = 0x02;          //Opcode for ClearIRQStatus command
  loraSpi.g_spiBuff[1] = 0xFF;          //IRQ bits to clear (MSB) (0xFFFF means clear all interrupts)
  loraSpi.g_spiBuff[2] = 0xFF;          //IRQ bits to clear (LSB)
  loraSpi.transferSPI(3);
  
  //Disable Transmitter
  rfSwitch(RF_SWITCH_OFF);
  if (DEBUG_LORA_MESSAGE)
  {
    Serial.print(F("Transmitted. Transmit time seconds "));
    Serial.println((millis() - startTransmitTime) / 1000);
  }
}

//Sets the radio into receive mode, allowing it to listen for incoming packets.
//If radio is already in receive mode, this does nothing.
//There's no such thing as "setModeTransmit" because it is set automatically when transmit() is called
void SX1262_Loracom::setModeReceive() {

  //Enable Receiver
  rfSwitch(RF_SWITCH_RX);

 // Set standby mode
  loraSpi.g_spiBuff[0] = 0x80;
  loraSpi.g_spiBuff[1] = 0x01; 
  loraSpi.transferSPI(2);

  //Packet parameters IQ workaround
  uint8_t regValIQ = loraSpi.readRegister(0x0736);
  if (SX1262_PACKET_PARAMETERS_INVERT_IQ == PACKET_PARAMETERS_IQ_INVERTED){  
    loraSpi.writeRegister(0x0736, regValIQ & 0xFB); 
  }
  else if (SX1262_PACKET_PARAMETERS_INVERT_IQ == PACKET_PARAMETERS_IQ_NORMAL) {
    loraSpi.writeRegister(0x0736, regValIQ | 0x04); 
  }  
  //Set packet parameters
  loraSpi.g_spiBuff[0] = 0x8C;
  loraSpi.g_spiBuff[1] = SX1262_PACKET_PARAMETERS_PREAMBLE_LENGTH_MSB;
  loraSpi.g_spiBuff[2] = SX1262_PACKET_PARAMETERS_PREAMBLE_LENGTH_MSB;
  loraSpi.g_spiBuff[3] = SX1262_PACKET_PARAMETERS_HEADER_TYPE;
  loraSpi.g_spiBuff[4] = 0xFF; //Payload Length (Max is 255 bytes)
  loraSpi.g_spiBuff[5] = SX1262_PACKET_PARAMETERS_CRC_ON_OFF;
  loraSpi.g_spiBuff[6] = SX1262_PACKET_PARAMETERS_INVERT_IQ;
  loraSpi.transferSPI(7);

  // Tell the chip to wait for it to receive a packet.
  // Based on our previous config, this should throw an interrupt when we get a packet
  loraSpi.g_spiBuff[0] = 0x82;          //0x82 is the opcode for "SetRX"
  loraSpi.g_spiBuff[1] = SX1262_RECEIVE_TIMEOUT_MSB;
  loraSpi.g_spiBuff[2] = SX1262_RECEIVE_TIMEOUT_MID;
  loraSpi.g_spiBuff[3] = SX1262_RECEIVE_TIMEOUT_LSB;
  loraSpi.transferSPI(4);

  // Disable receive mode
  rfSwitch(RF_SWITCH_OFF);

  Serial.println(F("RX listening"));
  return;
}

/*Receive a packet if available
If available, this will return the size of the packet and store the packet contents into the user-provided buffer.
A max length of the buffer can be provided to avoid buffer overflow.  If buffer is not large enough for entire payload, overflow is thrown out.
Recommended to pass in a buffer that is 255 bytes long to make sure you can received any lora packet that comes in.

Returns -1 when no packet is available.
Returns 0 when an empty packet is received (packet with no payload)
Returns payload size (1-255) when a packet with a non-zero payload is received. If packet received is larger than the buffer provided, this will return buffMaxLen
*/
int SX1262_Loracom::receive_async() {

  //Radio pin DIO1 (interrupt) goes high when we have a packet ready.  If it's low, there's no packet yet
  if (digitalRead(SX1262_DIO1) == false) { return -1; } //Return -1, meanining no packet ready

  if (getIrqStatus())
  {

    //Tell the radio to clear the interrupt, and set the pin back inactive.
    while (digitalRead(SX1262_DIO1)) {
      loraSpi.g_spiBuff[0] = 0x02;          //Opcode for ClearIRQStatus command
      loraSpi.g_spiBuff[1] = 0xFF;          //IRQ bits to clear (MSB) (0xFFFF means clear all interrupts)
      loraSpi.g_spiBuff[2] = 0xFF;          //IRQ bits to clear (LSB)
      loraSpi.transferSPI(3);
    }

    // (Optional) Read the packet status info from the radio.
    // This is things like radio strength, noise, etc.
    // See datasheet 13.5.3 for more info
    // This provides debug info about the packet we received
    
    loraSpi.g_spiBuff[0] = 0x14;          //Opcode for get packet status
    loraSpi.g_spiBuff[1] = 0x00;          //Dummy byte. Returns status
    loraSpi.g_spiBuff[2] = 0x00;          //Dummy byte. Returns rssi
    loraSpi.g_spiBuff[3] = 0x00;          //Dummy byte. Returns snd
    loraSpi.g_spiBuff[4] = 0x00;          //Dummy byte. Returns signal RSSI
    loraSpi.transferSPI(5);

    // Check transmission data
    lastReceivedRSSI = (-((int)loraSpi.g_spiBuff[2]) / 2);
    lastReceivedSNR = (((int)loraSpi.g_spiBuff[3]) / 4);
    lastReceivedSignalRSSI = (-((int)loraSpi.g_spiBuff[4]) / 2);

    if (DEBUG_LORA_SIGNAL)
    {
      Serial.print(F("RSSI "));
      Serial.println(lastReceivedRSSI);
      Serial.print(F("snr "));
      Serial.println(lastReceivedSNR);
      Serial.print(F("signalRssi "));
      Serial.println(lastReceivedSignalRSSI);
    }

    //We're almost ready to read the packet from the radio
    //But first we have to know how big the packet is, and where in the radio memory it is stored
    loraSpi.g_spiBuff[0] = 0x13;          //Opcode for GetRxBufferStatus command
    loraSpi.g_spiBuff[1] = 0x00;          //Dummy.  Returns radio status
    loraSpi.g_spiBuff[2] = 0x00;          //Dummy.  Returns loraPacketLength
    loraSpi.g_spiBuff[3] = 0x00;          //Dummy.  Returns memory offset (address)
    loraSpi.transferSPI(4);

    int payloadLen = loraSpi.g_spiBuff[2];
    int offset = loraSpi.g_spiBuff[3];

    if (payloadLen > TRANSMIT_RECEIVE_BUFFER_SIZE - 1){   //Leave room to add eol during receive buffer transfer
      payloadLen = TRANSMIT_RECEIVE_BUFFER_SIZE -1 ;
      if (DEBUG_LORA_ERROR)
      {
        Serial.print(F("Warning! Received message length is longer than receive buffer. Message is truncated! Max message length is "));
        Serial.println(payloadLen);
      }     
    }

    loraSpi.transferReceiveBuffer(payloadLen, offset, txRxBuff);

    if (DEBUG_LORA)
    {
      Serial.print(F("Received: "));
      Serial.println(txRxBuff); 
    }

    return payloadLen;  //Return how many bytes we actually read
  }
  //Clear interrupts
  loraSpi.g_spiBuff[0] = 0x02;          //Opcode for ClearIRQStatus command
  loraSpi.g_spiBuff[1] = 0xFF;          //IRQ bits to clear (MSB) (0xFFFF means clear all interrupts)
  loraSpi.g_spiBuff[2] = 0xFF;          //IRQ bits to clear (LSB)
  loraSpi.transferSPI(3);
  return -1;  //Wrong CRC or other problem
}

bool SX1262_Loracom::getIrqStatus(){

  loraSpi.g_spiBuff[0] = 0x12;          
  loraSpi.g_spiBuff[1] = 0x00;          
  loraSpi.g_spiBuff[2] = 0x00;          
  loraSpi.g_spiBuff[3] = 0x00;          
  loraSpi.transferSPI(4);

  bool IrqOk = true;
  
  //Parse out the status (see datasheet for what each bit means)
  if ((loraSpi.g_spiBuff[3]) & 0x01) {
    Serial.println(F("IRQ set: TxDone"));
  }
  if ((loraSpi.g_spiBuff[3]) & 0x02) {
    Serial.println(F("IRQ set: RxDone"));
  }
  if ((loraSpi.g_spiBuff[3]) & 0x04) {
    Serial.println(F("IRQ set: Preamble detected"));
  }
  if ((loraSpi.g_spiBuff[3]) & 0x08) {
    Serial.println(F("IRQ set: FSK Valid Sync Word detected"));
  }
  if ((loraSpi.g_spiBuff[3]) & 0x10) {
    Serial.println(F("IRQ set: Valid LoRa Header received"));
  }
  if ((loraSpi.g_spiBuff[3]) & 0x20) {
    Serial.println(F("IRQ Sync: LoRa Header CRC error"));
    IrqOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x40) {
    Serial.println(F("IRQ Sync: Wrong CRC received"));
    IrqOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x80) {
    Serial.println(F("IRQ Sync: Channel activity detection finished"));
  }
  if ((loraSpi.g_spiBuff[2]) & 0x01) {
    Serial.println(F("IRQ set: Channel activity detected"));
  }
  if ((loraSpi.g_spiBuff[2]) & 0x02) {
    Serial.println(F("IRQ set: Rx or Tx Timeout"));
    IrqOk = false;
  }  
  return IrqOk;
}

errorsStruct SX1262_Loracom::getDeviceErrors(){

  loraSpi.g_spiBuff[0] = 0x17;          
  loraSpi.g_spiBuff[1] = 0x00;          
  loraSpi.g_spiBuff[2] = 0x00;          
  loraSpi.g_spiBuff[3] = 0x00;          
  loraSpi.transferSPI(4);
  
  //Init all to default
  errorsStruct deviceErrors = {false, false, false, false, false, false, false, false, true};
  
  //Parse out the status (see datasheet for what each bit means)
  if ((loraSpi.g_spiBuff[3]) & 0x01) {
    //Serial.println(F("##ERROR## Device error: RC64K_CALIB_ERR"));
	deviceErrors.rc64kCalibrationError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x02) {
    //Serial.println(F("##ERROR## Device error:  RC13M_CALIB_ERR"));
	deviceErrors.rc13mCalibrationError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x04) {
    //Serial.println(F("##ERROR## Device error: PLL_CALIB_ERR"));
	deviceErrors.pllCalibrationError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x08) {
    //Serial.println(F("##ERROR## Device error: ADC_CALIB_ERR"));
	deviceErrors.adcCalibrationError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x10) {
    //Serial.println(F("##ERROR## Device error: IMG_CALIB_ERR"));
	deviceErrors.imgCalibrationError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x20) {
    //Serial.println(F("##ERROR## Device error: XOSC_START_ERR"));
	deviceErrors.xoscStartError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[3]) & 0x40) {
    //Serial.println(F("##ERROR## Device error: PLL_LOCK_ERR"));
	deviceErrors.pllLockError = true;
    deviceErrors.allOk = false;
  }
  if ((loraSpi.g_spiBuff[2]) & 0x01) {
    //Serial.println(F("##ERROR## Device error: PA_RAMP_ERR"));
	deviceErrors.paRampError = true;
    deviceErrors.allOk = false;
  }
  if (deviceErrors.allOk){
    //Serial.println(F("Device OK"));
  }
  else{
    // Clear device errors
    loraSpi.g_spiBuff[0] = 0x07;          
    loraSpi.g_spiBuff[1] = 0x00;          
    loraSpi.g_spiBuff[2] = 0x00;          
    loraSpi.transferSPI(3);
  }
  return deviceErrors;
}

commandStatusStruct SX1262_Loracom::getCommandStatus(){

  loraSpi.g_spiBuff[0] = 0xC0;          
  loraSpi.g_spiBuff[1] = 0x00;               
  loraSpi.transferSPI(2);

  uint8_t cmdStatus = (loraSpi.g_spiBuff[1] >> 1) & 0x7;//Command status is bits [3:1] (3-bits)
  
  commandStatusStruct commandStatus = {false, false, false, false, false};

  //Parse out the status (see datasheet for what each bit means)
  if ((cmdStatus) == 0x02) {
    Serial.println(F("Command status:  A packet has been successfully received and data can be retrieved"));
	commandStatus.rxSuccessPacketReady = true;
  }
  if ((cmdStatus) == 0x03) {
    Serial.println(F("Command status: A transaction from host took too long to complete and triggered an internal watchdog"));
	commandStatus.internalWatchdogTripped = true;
  }
  if ((cmdStatus) == 0x04) {
    Serial.println(F("Command status: . Processor was unable to process command either because of an invalid opcode or because an incorrect number of parameters has been provided"));
	commandStatus.invalidOpCodeOrWrongNumberOfParameters = true;
  }
  if ((cmdStatus) == 0x05) {
    Serial.println(F("Command status: The command was successfully processed, however the chip could not execute the command; for instance it was unable to enter the specified device mode or send the requested data"));
	commandStatus.couldNotExecuteCommand = true;
  }
  if ((cmdStatus) == 0x06) {
    Serial.println(F("Command status: The transmission of the current packet has terminated"));
	commandStatus.txCurrentPackageTerminated = true;
  }
  return commandStatus;
}

chipModeStruct SX1262_Loracom::getChipMode(){

  loraSpi.g_spiBuff[0] = 0xC0;          
  loraSpi.g_spiBuff[1] = 0x00;               
  loraSpi.transferSPI(2);

  uint8_t chipM = (loraSpi.g_spiBuff[1] >> 4) & 0x7;     //Chip mode is bits [6:4] (3-bits)
  
  chipModeStruct chipMode = {false, false, false, false, false};

  //Parse out the status (see datasheet for what each bit means)
  if ((chipM) == 0x02) {
    Serial.println(F("Device status: STBY_RC"));
	chipMode.stbyRC = true;
  }
  if ((chipM) == 0x03) {
    Serial.println(F("Device status: STBY_XOSC"));
	chipMode.stbyXosc = true;
  }
  if ((chipM) == 0x04) {
    Serial.println(F("Device status: FS"));
	chipMode.Fs = true;
  }
  if ((chipM) == 0x05) {
    Serial.println(F("Device status: RX"));
	chipMode.Rx = true;
  }
  if ((chipM) == 0x06) {
    Serial.println(F("Device status: TX"));
	chipMode.Tx = true;
  }
  return chipMode;
}


int SX1262_Loracom::getLastReceivedRSSI()
{
	return lastReceivedRSSI;
}

int SX1262_Loracom::getLastReceivedSNR()
{
	return lastReceivedSNR;
}

int SX1262_Loracom::getLastReceivedSignalRSSI()
{
	return lastReceivedSignalRSSI;
}

void SX1262_Loracom::printRxTxBuffer(int length){
  for(int i=0; i < length; i++){
    Serial.print("txRxBuff ");
    Serial.print(i);
    Serial.print(" -> ");
    Serial.println(txRxBuff[i]);
  }
  return;
}