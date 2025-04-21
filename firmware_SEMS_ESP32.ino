#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
//#define TIMER_BASE_CLK    (APB_CLK_FREQ)  // Add this before include
//#include <ESP32TimerInterrupt.h>
//#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <cstdio>
#include <iostream>

#include <BLEDevice.h>

#include "WiFi.h"
//#include "ADE9000.h"
//#include "PCA9671BS.h"
#include "Connection.h"
#include "Utils.h"
#include "esp_log.h"
//LORA
#include "SX126XLT.h"                            //include the appropriate library   
#include "Settings.h"                            //include the setiings file, frequencies, LoRa settings etc   

#define bleServerName "Client_Board"

static BLEUUID cbServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID cCharacteristicUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID dCharacteristicUUID("fb9ed969-b64c-4ea8-9111-325e3687b3fb");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

//Characteristicd that we want to read
static BLERemoteCharacteristic* cCharacteristic;
static BLERemoteCharacteristic* dCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//Variables to store data
char* dataChar;

SX126XLT LT;                                     //create a library class instance called LT

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
int8_t  PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio (SNR) of received packet

bool RX_set = false;
bool RX_received = false;
//LORA

Connection conn("http:/192.168.0.229:8000/boards/authenticate"); 
//ADE9000 ade_0(&expander, 0);
//ADE9000 ade_1(&expander, 1);
//int json_state = 0;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      Serial.println("Found it! Stopping Scan");
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

static void dataNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    dataChar = (char*)pData;
    Serial.print("Received data: ");
    Serial.println(dataChar);
}

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
  Serial.println("Creating DFA Client");
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  Serial.println("Attempting to Connect");
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(cbServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(cbServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  cCharacteristic = pRemoteService->getCharacteristic(cCharacteristicUUID);
  dCharacteristic = pRemoteService->getCharacteristic(dCharacteristicUUID);
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  //temperatureCharacteristic->registerForNotify(temperatureNotifyCallback);
  dCharacteristic->registerForNotify(dataNotifyCallback);
  return true;
}

//LORA
void packet_is_OK()
{
  uint16_t IRQStatus, localCRC;
  IRQStatus = LT.readIrqStatus();                 //read the LoRa device IRQ status register

  RXpacketCount++;

  printElapsedTime();                             //print elapsed time to Serial Monitor
  Serial.print(F("  "));
  LT.printASCIIPacket(RXBUFFER, RXPacketL);       //print the packet as ASCII characters

  localCRC = LT.CRCCCITT(RXBUFFER, RXPacketL, 0xFFFF);  //calculate the CRC, this is the external CRC calculation of the RXBUFFER
  Serial.print(F(",CRC,"));                       //contents, not the LoRa device internal CRC
  Serial.print(localCRC, HEX);
  Serial.print(F(",RSSI,"));
  Serial.print(PacketRSSI);
  Serial.print(F("dBm,SNR,"));
  Serial.print(PacketSNR);
  Serial.print(F("dB,Length,"));
  Serial.print(RXPacketL);
  Serial.print(F(",Packets,"));
  Serial.print(RXpacketCount);
  Serial.print(F(",Errors,"));
  Serial.print(errors);
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);

  conn.ping_LoRa_Backend();
}


void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                   //read the LoRa device IRQ status register

  printElapsedTime();                               //print elapsed time to Serial Monitor

  if (IRQStatus & IRQ_RX_TIMEOUT)                   //check for an RX timeout
  {
    Serial.print(F(" RXTimeout"));
  }
  else
  {
    errors++;
    Serial.print(F(" PacketError"));
    Serial.print(F(",RSSI,"));
    Serial.print(PacketRSSI);
    Serial.print(F("dBm,SNR,"));
    Serial.print(PacketSNR);
    Serial.print(F("dB,Length,"));
    Serial.print(LT.readRXPacketL());               //get the device packet length
    Serial.print(F(",Packets,"));
    Serial.print(RXpacketCount);
    Serial.print(F(",Errors,"));
    Serial.print(errors);
    Serial.print(F(",IRQreg,"));
    Serial.print(IRQStatus, HEX);
    LT.printIrqStatus();                            //print the names of the IRQ registers set
  }

  delay(250);                                       //gives a longer buzzer and LED flash for error

}


void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}


void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;

  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}
//LORA interrupt
void IRAM_ATTR wakeUp()
{
  RX_received = true;
}

//void IRAM_ATTR timerISR();
/*
void ADE9000_setup(uint32_t SPI_speed) {
  SPI.begin(26,25,33);    //Initiate SPI port 26,25,33
  SPI.beginTransaction(SPISettings(SPI_speed,MSBFIRST,SPI_MODE0));    //Setup SPI parameters
  ade_0.SPI_Init(SPI_speed, PCA_PIN_P10, LOW); // for ADE9000 id 0, the enable pin should be LOW
  ade_1.SPI_Init(SPI_speed, PCA_PIN_P10, HIGH); // for ADE9000 id 0, the enable pin should be HIGH
  ade_0.begin(); //set up the chip and get it running
  ade_1.begin();
  delay(200); //give some time for everything to come up
}
*/
/*
void sendData(float voltage, float current){
  // Here is where we would send the data to the server 
}
*/
/*
void parseData(const char* message){
  float voltage, current;
  if (sscanf(message, "{%f,%f}", &voltage, &current) == 2) {
    // Successfully parsed
    Serial.print(F("Parsed values: "));
    Serial.print(voltage);
    Serial.print(F(","));
    Serial.println(current);
  } else {
    // Parsing failed
    Serial.print(F("Invalid message format\n"));
  }
}
*/

void setup()
{
  //bool bt_connected;
  // Initialize Serial Communication
  Serial.begin(115200);
  delay(100);

  // Wait for the user to press start
  Serial.println(F(" --- Energy Prediction and Monitoring System --- \n"));
  Serial.println(F("Please press enter to continue..."));
  while(Serial.available() == 0);
  flushInputBuffer();

  // Connect to the server
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.println();
  Serial.println(F("----------- Step 1: WiFi Connection -----------\n"));
  conn.initWiFi();
  conn.initBackend();
  delay(100);

  //LORA Setup
  pinMode(LED1, OUTPUT);                        //setup pin as output for indicator LED
  led_Flash(2, 125);                            //two quick LED flashes to indicate program start

  pinMode(RFSW_V1, OUTPUT);
  pinMode(RFSW_V2, OUTPUT);
  digitalWrite(RFSW_V1, HIGH);
  digitalWrite(RFSW_V2, LOW);

  Serial.println();
  Serial.println(F("104_LoRa_Receiver_Detailed_Setup_ESP32 Starting"));

  SPI.begin(SCK, MISO, MOSI);
  //setup hardware pins used by device, then check if device is found
  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, RFSW_V1, RFSW_V2, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                       //long fast speed LED flash indicates device error
    }
  }

  LT.setMode(MODE_STDBY_XOSC);
  LT.setRegulatorMode(USE_DCDC);
  LT.setPaConfig(0x04, PAAUTO, LORA_DEVICE);
  LT.setDIO3AsTCXOCtrl(TCXO_CTRL_3_3V);
  LT.calibrateDevice(ALLDevices);                //is required after setting TCXO
  LT.calibrateImage(Frequency);
  LT.setDIO2AsRfSwitchCtrl();
  LT.setPacketType(PACKET_TYPE_LORA);
  LT.setRfFrequency(Frequency, Offset);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate, Optimisation);
  LT.setBufferBaseAddress(0, 0);
  LT.setPacketParams(8, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);   //set for IRQ on TX done and timeout on DIO1
  LT.setHighSensitivity();  //set for maximum gain
  LT.setSyncWord(LORA_MAC_PUBLIC_SYNCWORD);

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();

  Serial.println("Initializing DFA Board Bluetooth");

  BLEDevice::init("DFA");
  Serial.println("Initialized");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  Serial.println("Scanned for Client Board");
}

void loop()
{
  conn.loop();
  delay(100);

  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      if(cCharacteristic->canNotify())
        cCharacteristic->registerForNotify(dataNotifyCallback);
      if(dCharacteristic->canNotify())
        dCharacteristic->registerForNotify(dataNotifyCallback);
      connected = true;
      Serial.println("Checking for load dump");
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }
  
  //LORA
  if (!RX_set) {
    RX_set = true;
    LT.fillSXBuffer(0, 1, '#');
    RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 0, NO_WAIT); //wait for a packet to arrive with 60seconds (60000mS) timeout
    LT.clearIrqStatus(IRQ_RADIO_ALL);                     //ensure the DIO1 low is cleared, otherwise there could be an immediate wakeup 
    Serial.println("Set LoRa Interrupt");
    attachInterrupt(DIO1, wakeUp, RISING);
    interrupts();
  }

  if (RX_received) {
    detachInterrupt(DIO1);

    digitalWrite(LED1, HIGH);

    //handler for the interrupt
    RXPacketL = LT.readPacket(RXBUFFER, RXBUFFER_SIZE);   //now read in the received packet to the RX buffer

    PacketRSSI = LT.readPacketRSSI();
    PacketSNR = LT.readPacketSNR();

    if (RXPacketL == 0)
    {
      packet_is_Error();
    }
    else
    {
      packet_is_OK();
    }

    RX_set = false;
    RX_received = false;

    digitalWrite(LED1, LOW);
  }

  //If user input is provided, send the command over bluetooth to the clientboard
  /*
  if(connected && Serial.available()){
    flushInputBuffer();
    char command = '2';
    while(1){
      char input = Serial.read();
      if (input == '\r') {
        Serial.println();
        break;
      } else {
        command = input;
      }
    }
    cCharacteristic->writeValue(command, 8);
  }
  */

  /*
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 3000) {
    String json;
    if (json_state == 0) {
      json = ade_0.generate_json();
      json_state = 1;
    } else if (json_state == 1) {
      json = ade_1.generate_json();
      json_state = 0;
    }
    conn.send(json.c_str());
    // conn.send("{\"action\": \"ping\"}");
    lastTime = currentTime;
  }
  delay(100);
  
  // ADE ID 0:
  ade_0.ReadVoltageRMSRegs(&RMS_voltage);
  Serial.print("ADE9000 ID NUMBER: ");
  Serial.println(ade_0.id());
  Serial.print("Phase A Voltage: ");
  Serial.println((float) ade_0.V_nom()/(ade_0.V_expected()) * RMS_voltage.VoltageRMSReg_A);
  Serial.print("Phase B Voltage: ");
  Serial.println((float) ade_0.V_nom()/(ade_0.V_expected()) * RMS_voltage.VoltageRMSReg_B);
  Serial.print("Phase C Voltage: ");
  Serial.println((float) ade_0.V_nom()/(ade_0.V_expected()) * RMS_voltage.VoltageRMSReg_C);

  // // ADE ID 1:
  ade_1.ReadVoltageRMSRegs(&RMS_voltage);
  Serial.print("ADE9000 ID NUMBER: ");
  Serial.println(ade_1.id());
  Serial.print("Phase A Voltage: ");
  Serial.println((float) ade_1.V_nom()/(ade_1.V_expected()) * RMS_voltage.VoltageRMSReg_A);
  Serial.print("Phase B Voltage: ");
  Serial.println((float) ade_1.V_nom()/(ade_1.V_expected()) * RMS_voltage.VoltageRMSReg_B);
  Serial.print("Phase C Voltage: ");
  Serial.println((float) ade_1.V_nom()/(ade_1.V_expected()) * RMS_voltage.VoltageRMSReg_C);
  Serial.println();
  */

}
