#define DEBUG_LORA_LOWLEVEL false
#define DEBUG_LORA_MESSAGE true
#define DEBUG_LORA_SIGNAL false
#define DEBUG_LORA_ERROR false

#define SX1262_TXEN  8
#define SX1262_RXEN  10
#define SX1262_NSS   7
#define SX1262_RESET 14 
#define SX1262_DIO1  5
#define SX1262_DIO2  9
#define SX1262_BUSY	 3

#define RF_SWITCH_RX 0x01
#define RF_SWITCH_TX 0x02
#define RF_SWITCH_OFF 0x00

#define SPI_BUFFER_SIZE 32
#define TRANSMIT_RECEIVE_BUFFER_SIZE 160
#define MAX_TRANSMIT_DATA_LENGTH 160

#define TRANSMIT_TIMEOUT 600000
#define SPI_DELAY 10
#define SPI_CS_DELAY 1

//Define Spread Factors register values
#define SF5 0x05
#define SF6 0x06
#define SF7 0x07
#define SF8 0x08
#define SF9 0x09
#define SF10 0x0A
#define SF11 0x0B
#define SF12 0x0C

//Define Bandwidth registry values kHz 
#define BW7 0x00  // 7.81 
#define BW10 0x08 // 10.42 
#define BW15 0x01 // 15.63 
#define BW20 0x09 // 20.83
#define BW31 0x02 // 31.25 
#define BW41 0x0A // 41.67 
#define BW62 0x03 // 62.5 
#define BW125 0x04 // 125 
#define BW250 0x05 // 250 
#define BW500 0x06 // 500 

//Define code rate
#define CR45 0x01 //Code rate 4-5
#define CR46 0x02 //Code rate 4-6
#define CR47 0x03 //Code rate 4-7
#define CR48 0x04 //Code rate 4-8

//Low data rate optimize
#define LR_OFF 0x00
#define LR_ON 0x01

//PA config
#define PA_DUTY_CYCLE_22_DBM 0x04 
#define PA_DUTY_CYCLE_20_DBM 0x03 
#define PA_DUTY_CYCLE_17_DBM 0x02 
#define PA_DUTY_CYCLE_14_DBM 0x02 

#define PA_HP_MAX_22_DBM 0x07
#define PA_HP_MAX_20_DBM 0x05
#define PA_HP_MAX_17_DBM 0x03
#define PA_HP_MAX_14_DBM 0x02

//TX params
#define TX_PARAMS_RAMP_TIME_10 0x00
#define TX_PARAMS_RAMP_TIME_20 0x01
#define TX_PARAMS_RAMP_TIME_40 0x02
#define TX_PARAMS_RAMP_TIME_80 0x03
#define TX_PARAMS_RAMP_TIME_200 0x04
#define TX_PARAMS_RAMP_TIME_800 0x05
#define TX_PARAMS_RAMP_TIME_1700 0x06
#define TX_PARAMS_RAMP_TIME_3400 0x07

#define TCXO_VOLTAGE_16 0x00 //1.6v
#define TCXO_VOLTAGE_17 0x01 //1.7v
#define TCXO_VOLTAGE_18 0x02 //1.8v
#define TCXO_VOLTAGE_22 0x03 //2.2v
#define TCXO_VOLTAGE_24 0x04 //2.4v
#define TCXO_VOLTAGE_27 0x05 //2.7v
#define TCXO_VOLTAGE_30 0x06 //3.0v
#define TCXO_VOLTAGE_33 0x07 //3.3v

#define TCXO_VOLTAGE TCXO_VOLTAGE_24

#define TX_PARAMS_POWER_22 0x16
#define TX_PARAMS_POWER_20 0x14
#define TX_PARAMS_POWER_17 0x11
#define TX_PARAMS_POWER_14 0x0E


#define TX_PARAMS_POWER TX_PARAMS_POWER_22

#define TX_PARAMS_RAMP_TIME TX_PARAMS_RAMP_TIME_200

#define PA_DUTY_CYCLE PA_DUTY_CYCLE_22_DBM 
#define PA_HP_MAX PA_HP_MAX_22_DBM
#define PA_DEVICE 0x00   //1262 = 0x00
#define PA_LUT 0x01 //Always

#define SX1262_RX_BOOSTED_GAIN 0x96                                   // 0x96 = Boosted, 0x94 = Normal

#define SX1262_MODULATION_PARAMETERS_SPREADING_FACTOR SF7            // Higher SF gives better sensitivity and longer transmission time. SF5-SF12 (For SF5, SF6 use minimum preamble of 12 symbols)
#define SX1262_MODULATION_PARAMETERS_BANDWIDTH BW125                  // Higher bamdwidth is higher speed and lower sensitivity
#define SX1262_MODULATION_PARAMETERS_CODING_FACTOR CR45              // Higher CF is Better noise immunity and lower transmission speed
#define SX1262_MODULATION_PARAMETERS_LOW_DATARATE_OPTIMIZE LR_ON    // On for High SF and/or Low BW. (typically on for symbol times over 16.38 ms)


#define PACKET_PARAMETERS_IQ_NORMAL 0x00
#define PACKET_PARAMETERS_IQ_INVERTED 0x01

#define PACKET_PARAMETERS_CRC_OFF 0x00
#define PACKET_PARAMETERS_CRC_ON 0x01

#define PACKET_PARAMETERS_HEADER_TYPE_VARIABLE_LENGTH 0X0
#define PACKET_PARAMETERS_HEADER_TYPE_FIXED_LENGTH 0X01

#define SX1262_PACKET_PARAMETERS_PREAMBLE_LENGTH_MSB 0x00                                         //PacketParam1 = Preamble Len MSB
#define SX1262_PACKET_PARAMETERS_PREAMBLE_LENGTH_MSB 0x0C                                         //PacketParam2 = Preamble Len LSB (normal is 12 symbols)
#define SX1262_PACKET_PARAMETERS_HEADER_TYPE PACKET_PARAMETERS_HEADER_TYPE_VARIABLE_LENGTH                    
#define SX1262_PACKET_PARAMETERS_CRC_ON_OFF PACKET_PARAMETERS_CRC_ON                     
#define SX1262_PACKET_PARAMETERS_INVERT_IQ PACKET_PARAMETERS_IQ_NORMAL                 

#define SX1262_TRANSMIT_TIMEOUT_MSB 0x00                  //Transmit timeout 0x000000 means no timeout
#define SX1262_TRANSMIT_TIMEOUT_MID 0x00
#define SX1262_TRANSMIT_TIMEOUT_LSB 0x00

#define SX1262_RECEIVE_TIMEOUT_MSB 0xFF                   //Receive timeout 0xFFFFFF means no timeout
#define SX1262_RECEIVE_TIMEOUT_MID 0xFF
#define SX1262_RECEIVE_TIMEOUT_LSB 0xFF