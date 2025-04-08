#include <HTTPClient.h>
#include <Wire.h>
#include <SPI.h>
//#define TIMER_BASE_CLK    (APB_CLK_FREQ)  // Add this before include
//#include <ESP32TimerInterrupt.h>
#include <esp_task_wdt.h>
#include <soc/soc.h>
#include <cstdio>
//#include <iostream>
#include "BluetoothSerial.h"

#include "WiFi.h"
#include "ADE9000.h"
#include "PCA9671BS.h"
#include "Connection.h"
#include "Utils.h"
#include "esp_log.h"
//LORA
#include "SX126XLT.h"                            //include the appropriate library   
#include "Settings.h"                            //include the setiings file, frequencies, LoRa settings etc   

SX126XLT LT;                                     //create a library class instance called LT

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                 //create the buffer that received packets are copied into

uint8_t RXPacketL;                               //stores length of packet received
int8_t  PacketRSSI;                              //stores RSSI of received packet
int8_t  PacketSNR;                               //stores signal to noise ratio (SNR) of received packet
//LORA

BluetoothSerial SerialBT;

Connection conn("http://172.20.10.8:8000/boards/authenticate"); 
ADE9000 ade_0(&expander, 0);
ADE9000 ade_1(&expander, 1);
int json_state = 0;

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
//LORA

//void IRAM_ATTR timerISR();

void ADE9000_setup(uint32_t SPI_speed) {
  SPI.begin(26,25,33);    //Initiate SPI port 26,25,33
  SPI.beginTransaction(SPISettings(SPI_speed,MSBFIRST,SPI_MODE0));    //Setup SPI parameters
  ade_0.SPI_Init(SPI_speed, PCA_PIN_P10, LOW); // for ADE9000 id 0, the enable pin should be LOW
  ade_1.SPI_Init(SPI_speed, PCA_PIN_P10, HIGH); // for ADE9000 id 0, the enable pin should be HIGH
  ade_0.begin(); //set up the chip and get it running
  ade_1.begin();
  delay(200); //give some time for everything to come up
}

void sendData(float voltage, float current){
  /* Here is where we would send the data to the server */
}

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
  LT.setSyncWord(LORA_MAC_PRIVATE_SYNCWORD);

  Serial.print(F("Receiver ready - RXBUFFER_SIZE "));
  Serial.println(RXBUFFER_SIZE);
  Serial.println();

  // Initialize Bluetooth for Commands/Data Transfer
  if(!SerialBT.begin("ESP32")){ //https://techtutorialsx.com/2018/04/27/esp32-arduino-bluetooth-classic-controlling-a-relay-remotely/
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    Serial.println("Bluetooth initialized");
  }

  // Initiate the expander
  /*
  expander.begin();
  delay(100);

  // Set up the ADE9000
  ADE9000_setup(8000000);

  // Calibratring the ADE9000
  Serial.println();
  Serial.println("----------- Step 2: Voltage Calibration -----------\n");
  ade_0.calibrate_VRMS();
  ade_1.calibrate_VRMS();
  delay(1000);
  */
}

void loop()
{
  //conn.loop();
  delay(100);
  
  //LORA
  RXPacketL = LT.receive(RXBUFFER, RXBUFFER_SIZE, 60000, WAIT_RX); //wait for a packet to arrive with 60seconds (60000mS) timeout

  digitalWrite(LED1, HIGH);                      //something has happened

  PacketRSSI = LT.readPacketRSSI();              //read the recived RSSI value
  PacketSNR = LT.readPacketSNR();                //read the received SNR value

  Serial.println(RXPacketL);
  if (RXPacketL == 0)                            //if the LT.receive() function detects an error, RXpacketL is 0
  {
    packet_is_Error();
  }
  else
  {
    packet_is_OK();
  }

  digitalWrite(LED1, LOW);                       //LED off

  //Check for Bluetooth messages
  if(SerialBT.available()){
    String message;
    Serial.print(F("BT Message received: "));
    char incomingChar = SerialBT.read();
    if (incomingChar != '\n'){
      message += String(incomingChar);
    }
    else{
      message = "";
    }
    parseData(message.c_str());
    Serial.write(incomingChar); 
  }

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
