/*
  SEMS DFA Motherboard Firmware - ECE/MEM 17
  Developed by: Nate Judd, Cassius Garcia, Kaylie Ludwick, Varun Iyengar, and Harrison Muller
  Description:
    This file includes firmware for the operation of the DFA Motherboard, including code to read energy data from the onboard ADE9000s, LoRa communications
    Bluetooth Low Energy Communications, and Data Transfer to the System Backend.
  Compiling & Uploading:
    For uploading, use the Huge APP partition scheme.
*/

/*
  Network Communication Libraries - Needed for Bluetooth, LoRa, and WiFi
*/
#include <HTTPClient.h>
#include "WiFi.h"
#include "Connection.h"
#include <BLEDevice.h>
#include "SX126XLT.h"                            //include the appropriate library   
#include "Settings.h"                            //include the setiings file, frequencies, LoRa settings etc

/*
  Additional Libraries - Needed for integration
*/
#include <Wire.h>
#include <SPI.h>
#include <soc/soc.h>
#include <cstdio>
#include <iostream>
#include "Utils.h"
#include "esp_log.h"  

/*
  SD & RTC Card Libraries
*/
#include <SPI.h>
#include <SdFat.h>
#include <sdios.h>
#include "RTClib.h"

/*
  Setup SD Card and RTC
*/
#define SCK  14
#define MISO  12
#define MOSI  13
#define CS  15

#define I2C_SDA 19
#define I2C_SCL 18
#define SPI_CLOCK SD_SCK_MHZ(25)
#define SD_CONFIG SdSpiConfig(CS, SHARED_SPI, SPI_CLOCK)

TwoWire I2CRTC = TwoWire(0);
RTC_DS3231 rtc;
SdFat sd;
SdFile file;

uint32_t cardSectorCount = 0;
uint8_t sectorBuffer[512];
SdCardFactory cardFactory;
SdCard* m_card = nullptr;

int lastDay = 0;
int lastMinute = 0;
unsigned long lastTime = 0;

//change based on frequency of measurement
unsigned long timerDelay = 30000;

String filePath = "";

/*
  Setup Bluetooth Low Energy - DFA is client and will scan for the server (Client Boards)
  https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
*/

//Define Client Board server name
#define bleServerName "Client_Board"

#define ADC_PIN 2

//Define BLUEUUIDs for required services/characteristics
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

/*
  Setup LoRa Modules - Reciever
  https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/src
*/

//Create a library class instance called LT
SX126XLT LT;

//Define packet count and error for debugging
uint32_t RXpacketCount;
uint32_t errors;

//Create the buffer that received packets are copied into
uint8_t RXBUFFER[RXBUFFER_SIZE];                 

uint8_t RXPacketL;                               //Stores length of packet received
int8_t  PacketRSSI;                              //Stores RSSI of received packet
int8_t  PacketSNR;                               //Stores signal to noise ratio (SNR) of received packet

bool RX_set = false;
bool RX_received = false;

String dummy_data = "{\"data\": [{\"id\": \"board-069\",\"VA_MAG\": 120.0,\"VB_MAG\": 118.5,\"VC_MAG\": 119.2,\"IA_MAG\": 5.2,\"IB_MAG\": 4.8,\"IC_MAG\": 5.0,\"VA_ANG\": 0.0,\"VB_ANG\": 120.0,\"VC_ANG\": -120.0,\"IA_ANG\": 30.5,\"IB_ANG\": 150.2,\"IC_ANG\": -90.8,\"POW_FACTOR\": 0.92,\"POW_APPARENT\": 624.0,\"POW_ACTIVE\": 574.08,\"POW_REACTIVE\": 255.36,\"board_info\": {\"board_id\": \"Nates's house\",  \"ade_id\": 420000000}}]}";

int iterate_actuate = 0;
int iterate_ADC = 0;

/*
  Setup Wifi Connection - Needed for sending Data to Server
  **See Connection.cpp and Connection.h
*/
Connection conn("http://172.20.10.10:8000/boards/authenticate"); //Change IP to current when running
//ADE9000 ade_0(&expander, 0);
//ADE9000 ade_1(&expander, 1);
//int json_state = 0;

/*
  Set up SD and RTC Functions
*/

uint32_t const ERASE_SIZE = 262144L;

void eraseCard() {
  Serial.println("Erasing\n");
  uint32_t firstBlock = 0;
  uint32_t lastBlock;
  uint16_t n = 0;

  do {
    lastBlock = firstBlock + ERASE_SIZE - 1;
    if (lastBlock >= cardSectorCount) {
      lastBlock = cardSectorCount - 1;
    }
    if (!m_card->erase(firstBlock, lastBlock)) {
      Serial.print("erase failed");
    }
    firstBlock += ERASE_SIZE;
  } while (firstBlock < cardSectorCount);

  if (!m_card->readSector(0, sectorBuffer)) {
    Serial.print("erase error");
  }
}

void formatCard() {
  ExFatFormatter exFatFormatter;
  FatFormatter fatFormatter;

  // Format exFAT if larger than 32GB
  bool rtn = cardSectorCount > 67108864
               ? exFatFormatter.format(m_card, sectorBuffer, &Serial)
               : fatFormatter.format(m_card, sectorBuffer, &Serial);

  if (!rtn) {
    Serial.print("error");
  }
}

float getUsageRatio(SdFat& sd) {
  uint64_t total = sd.vol()->clusterCount();
  total *= sd.vol()->bytesPerCluster();
  uint64_t free = sd.vol()->freeClusterCount();
  free *= sd.vol()->bytesPerCluster();
  return (float)(total - free) / total;
}

// Generate file name based on date
String fileName(DateTime now){

  String yearStr = String(now.year(), DEC);
  String monthStr = (now.month() < 10 ? "0" : "") + String(now.month(), DEC);
  String dayStr = (now.day() < 10 ? "0" : "") + String(now.day(), DEC);

  return "/SMESdata_" + dayStr +  "_" + monthStr + "_" + yearStr + ".txt";
}

void initSDCard() {
  SPI.begin(SCK, MISO, MOSI, CS);
  if (!sd.begin(SD_CONFIG)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint32_t cardSizeMB = sd.card()->sectorCount() / 2048;
  Serial.printf("SD Card Size: %luMB\n", cardSizeMB);

  m_card = cardFactory.newCard(SD_CONFIG);
  if (!m_card ) {
    Serial.print("card init failed.");
    return;
  }
  cardSectorCount = m_card->sectorCount();
}

void writeFile(const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);
  if (!file.open(path, O_WRITE | O_CREAT | O_TRUNC)) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.print(message);
  file.close();
  Serial.println("File written");
}

void appendFile(const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);
  if (!file.open(path, O_APPEND | O_WRITE)) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.print(message);
  file.close();
  Serial.println("Message appended");
}

void sdEntry(String message){
  DateTime now = rtc.now();
  int dayOfMonth = now.day();
  int minuteOfHour = now.minute();
  std::string hourStr   = (now.hour() < 10 ? "0" : "") + std::string(now.hour(), DEC);
  std::string minuteStr = (now.minute() < 10 ? "0" : "") + std::string(now.minute(), DEC);
  std::string secondStr = (now.second() < 10 ? "0" : "") + std::string(now.second(), DEC);
    
  std::string timetest = hourStr + ":" + minuteStr + ":" + secondStr + "\r\n";
  Serial.println(timetest.c_str());

  if (dayOfMonth != lastDay) {
    if (getUsageRatio(sd) > 0.9) {
      Serial.print("erasing");
      eraseCard();
      formatCard();
      initSDCard();
    }
    filePath = fileName(now);
    writeFile(filePath.c_str(), "Energy Data, Time\r\n");
    lastDay = dayOfMonth;
  }
  String dataSD = message + ", " + hourStr + ":" + minuteStr + ":" + secondStr + "\r\n";
  appendFile(filePath.c_str(), dataSD.c_str());
}
/*
  Set up BLE callbacks - Scan and Connect to Server
  https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

/*
  Print Data Received from Server
  https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
*/
static void dataNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    dataChar = (char*)pData;
    String ble_json = generate_json_energy_record_cb(dataChar);
    //conn.send(json.c_str());
    conn.HTTP_send_data(ble_json);
    Serial.print("Received data: ");
    Serial.println(ble_json);

    //SD Card
    sdEntry(ble_json.c_str());
}

/*
  Connect to the BLE Server that has the name, Service, and Characteristics
  https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
*/
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
  dCharacteristic->registerForNotify(dataNotifyCallback);
  return true;
}

/*
  Print Packet Info on Successful Receive
  https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/src
*/
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
  Serial.println(IRQStatus, HEX);

  String lora_json = generate_json_energy_record((char *)RXBUFFER);
  Serial.println(lora_json.c_str());
  conn.HTTP_send_data(lora_json);
  conn.ping_LoRa_Backend();

  //SD Card
  sdEntry(lora_json.c_str());
}

/*
  Print Packet Info on Failed Receive
  https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/src
*/
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

/*
  Print elapsed time - Use for debugging
*/
void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}

/*
  Flash LED - Use for debugging
*/
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

/*
  LoRa Interrupt Handler 
  https://github.com/StuartsProjects/SX12XX-LoRa/blob/master/examples/SX126x_examples/Sleep/62_LoRa_Wake_on_RX_Atmel/62_LoRa_Wake_on_RX_Atmel.ino
*/
void IRAM_ATTR wakeUp()
{
  RX_received = true;
}

/*
  Code for ADE9000 and Data Manipulation - Commented for Development
*/

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

/*
  Setup Function - Try to declutter
  https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/src
  https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
*/
void setup()
{
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

  //Set up LoRa Parameters
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

  //Initialize BLE and Begin Scanning
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

  //SD Card Setup
  I2CRTC.begin(I2C_SDA, I2C_SCL, 100000);
  
  if (! rtc.begin(&I2CRTC)) {
    Serial.println("Couldn't find RTC");
    //Serial.flush();
    while (1) {}
  }
  
//after the time is set, this needs to be removed and then the code recompiled?
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  initSDCard();

  DateTime now = rtc.now();

  filePath = fileName(now);
  
  if (!file.open(filePath.c_str(), O_READ)) {
    Serial.println("File doesn't exist. Creating file...");
    writeFile(filePath.c_str(), "Energy Data, Time\r\n");
  } else {
    Serial.println("File already exists");
    //file.close();
  }
}

/*
  Loop Function - Implement WiFi, BLE, and LoRa Interrupt Functionality
  https://github.com/StuartsProjects/SX12XX-LoRa/tree/master/src
  https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE
*/
void loop()
{
  //Connect to WiFi using Websocket
  //conn.loop();
  delay(100);

  //Connect Bluetooth Client Board and Handle Notifications
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
  
  //Set LoRa Interrupts
  if (!RX_set) {
    RX_set = true;
    LT.fillSXBuffer(0, 1, '#');
    RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 0, NO_WAIT); //wait for a packet to arrive with 60seconds (60000mS) timeout
    LT.clearIrqStatus(IRQ_RADIO_ALL);                     //ensure the DIO1 low is cleared, otherwise there could be an immediate wakeup 
    Serial.println("Set LoRa Interrupt");
    attachInterrupt(DIO1, wakeUp, RISING);
    interrupts();
  }

  //After Successful receive, read packet and debug
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

  iterate_actuate++;
  if (iterate_actuate > 50) {
    int actuate = conn.HTTP_poll_actuate();
    
    if (actuate == 1) {
      cCharacteristic->writeValue("1");
      Serial.println("Actuated Relay On");
    }
    else if (actuate == 0) {
      cCharacteristic->writeValue("0");
      Serial.println("Actuated Relay Off");
    }
    else if (actuate == -2){
      Serial.println("Disconnected from Wifi");
    }
    else {
      Serial.println("Error in HTTP Communication");
    }
	iterate_actuate = 0;
  }
  
  //Send mainboard data
  iterate_ADC++;
  if (iterate_ADC > 50) {
	  int ADC_val = analogRead(ADC_PIN);
	  
	  String adc;
	  sprintf(adc, "{\"data\": [{\"id\": \"board-069\",\"VA_MAG\": %s,\"VB_MAG\": 118.5,\"VC_MAG\": 119.2,\"IA_MAG\": 5.2,\"IB_MAG\": 4.8,\"IC_MAG\": 5.0,\"VA_ANG\": 0.0,\"VB_ANG\": 120.0,\"VC_ANG\": -120.0,\"IA_ANG\": 30.5,\"IB_ANG\": 150.2,\"IC_ANG\": -90.8,\"POW_FACTOR\": 0.92,\"POW_APPARENT\": 624.0,\"POW_ACTIVE\": 574.08,\"POW_REACTIVE\": 255.36,\"board_info\": {\"board_id\": \"Nates's house\",  \"ade_id\": 420000000}}]}", std::to_string(ADC_val));
	  
	  conn.HTTP_send_data(adc);
	  iterate_ADC = 0;

    //SD Card
    sdEntry(adc.c_str());
  }

  /*
  Code for ADE9000 and Data Manipulation - Commented for Development
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
