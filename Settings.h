/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 04/04/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, a ESP32 shield base with my BBF board shield on
//top. Be sure to change the definitions to match your own setup. Some pins such as DIO2, DIO3, BUZZER
//may not be in used by this sketch so they do not need to be connected and should be included and be 
//set to -1.


#define NSS 15                                   //select pin on LoRa device
#define SCK 14                                  //SCK on SPI3
#define MISO 12                                 //MISO on SPI3 
#define MOSI 13                                 //MOSI on SPI3 

#define NRESET 33                               //reset pin on LoRa device
#define RFBUSY 25                               //busy line

#define LED1 32                                  //on board LED, high for on
#define DIO1 4                                 //DIO1 pin on LoRa device, used for RX and TX done 

#define RFSW_V1 19
#define RFSW_V2 18

#define LORA_DEVICE DEVICE_SX1261               //we need to define the device we are using


//*******  Setup LoRa Parameters Here ! ***************

//LoRa Modem Parameters
const uint32_t Frequency = 915000000;           //frequency of transmissions in hertz
const uint32_t Offset = 0;                      //offset frequency for calibration purposes

const uint8_t Bandwidth = LORA_BW_125;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF12;       //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_8;           //LoRa coding rate
const uint8_t Optimisation = LDRO_AUTO;         //low data rate optimisation setting, normally set to auto

const int8_t TXpower = 10;                      //LoRa transmit power in dBm


const uint16_t packet_delay = 1000;             //mS delay between packets

#define RXBUFFER_SIZE 32                        //RX buffer size  
