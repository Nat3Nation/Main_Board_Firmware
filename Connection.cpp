/****************************************************************************************************************
 Includes
***************************************************************************************************************/

#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "Connection.h"
#include "Utils.h"

void Connection::onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  // noInterrupts();
  JsonObject payload_obj;
  String action;
  DynamicJsonDocument doc(1024);
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from WebSocket server");
      break;
    case WStype_ERROR:
    Serial.print("Received error: ");
      Serial.write(payload, length);
      Serial.println();
    case WStype_CONNECTED:
      Serial.println("Connected to WebSocket server");
      break;
    case WStype_TEXT:
      deserializeJson(doc, payload);
      action = doc["action"].as<String>();
      payload_obj = doc["payload"];
      if (action == "pin_select") {
        if (payload_obj.containsKey("PIN_1")) {
          int val = payload_obj["PIN_1"].as<String>() == "HIGH" ? LOW : HIGH; // active low
          expander.digital_write(PCA_PIN_P05, val);
          // Serial.print("PIN_1: ");
          // Serial.println(val);
        }
        if (payload_obj.containsKey("PIN_2")) {
          int val = payload_obj["PIN_2"].as<String>() == "HIGH" ? LOW : HIGH; // active high
          expander.digital_write(PCA_PIN_P06, val);
          // Serial.print("PIN_2: ");
          // Serial.println(val);
        }
      }
      // Serial.print("Received message: ");
      // Serial.write(payload, length);
      // Serial.println();
      break;
    default:
      // Serial.println("Callback: Something else.");
      break;
  }
  // interrupts();
}

Connection::Connection(String serverName) {
  lastTime_ = 0;
  timerDelay_ = 5000;
  serverName_ = serverName; //should be serverName... figure out the error
}
 
void Connection::initWiFi()
{
  // Connect to wifi:
  WiFi.disconnect();
  String ssid = get_wifi_ssid();
  Serial.print("Please enter the password for ");
  Serial.println(ssid);
  String pass = get_wifi_pass();
  Serial.println();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println("Connected to the WiFi network");
  delay(500);
}

void Connection::initBackend()
{
  String token;
  if(WiFi.status()== WL_CONNECTED) {
    HTTPClient http;
    http.begin(client_, serverName_);
    http.setTimeout(10000);
    http.addHeader("Content-Type", "application/json");
    String jsonPayload = "{\"id\":\"680984395ae884f071d887ae\",\"board_type\":\"ADE9000\"}";
    int httpCode = http.POST(jsonPayload);
    if (httpCode > 0) { // HTTP response received
      String payload = http.getString();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload);
      token = doc["token"].as<String>();
      Serial.println(payload);
    } else {
      Serial.printf("[HTTP] POST request to %s failed with error %d\n", serverName_.c_str(), httpCode);
    }
    http.end();
  }
  char header[250];  
  sprintf(header, "Authorization: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpZCI6IjY4MDk4NDM5NWFlODg0ZjA3MWQ4ODdhZSIsImV4cCI6MTc0NTU0MzE4M30.xallqydEMP7ilQxfH3JbWtB5Wwy8KHlClgfPrPHHiEU\r\n");
  webSocket_.begin("172.20.10.10", 8000, "/boards/socket");
  webSocket_.setExtraHeaders(header);

  webSocket_.onEvent(onWebSocketEvent);
}

String Connection::get_wifi_ssid() {
  Serial.println("Scanning for networks...\n");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No networks found! Please make sure that there is accessible WiFi connections for the board.");
    return "0";
  } else {
    Serial.println("The following networks are found. Please select one:");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print("[");
      Serial.print(i + 1);
      Serial.print("] ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }
  String ssid;
  while(1) {
    ssid = getUserInput("Please select the WiFi by indicating the number in [] next to it: ");
    int val = ssid.toInt();
    if (val < 1 || val > n) {
      Serial.print("Please enter a number from 1 to ");
      Serial.println(n);
      continue;
    }
    Serial.println();
    return WiFi.SSID(val - 1);
  }
}

String Connection::get_wifi_pass() {
  Serial.print("WiFi Password: ");
  delay(10);
  String pass = "";
  flushInputBuffer();
  while(1) {
      if (Serial.available()) {
        char input = Serial.read();
        if (input == '\r') {
          Serial.println();
          break;
        } else if (input == DEL && pass.length() > 0) {
          pass.remove(pass.length() - 1);
          Serial.write("\b \b");
        } else {
          pass += input;
          Serial.write("*");
        }
      }
    }
  return pass;
}

void Connection::HTTP_post(String const & message) {
  //Send an HTTP POST request every 10 minutes
  while ((millis() - lastTime_) <= timerDelay_);
  //Check WiFi connection status
  if(WiFi.status()== WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
  
    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName_);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(message);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  lastTime_ = millis();
  
}

void Connection::ping_LoRa_Backend() {
  if(WiFi.status()== WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    String token;
  
    // Your Domain name with URL path or IP address with path
    String server = "http://172.20.10.10:8000/demo";
    http.begin(client, server);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.GET();
    if (httpResponseCode > 0) { // HTTP response received
      String payload = http.getString();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload);
      token = doc["token"].as<String>();
      Serial.println(payload);
    }
    else{
      Serial.println("Error");
    }
      
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
}

void Connection::HTTP_send_data(String const & message) {
  //Check WiFi connection status
  if(WiFi.status()== WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
  
    // Your Domain name with URL path or IP address with path
    http.begin(client, "http://172.20.10.10:8000/energy-recordss");
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(message);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
      
    // Free resources
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }
  lastTime_ = millis();
  
}

void Connection::loop() {
  webSocket_.loop();
}

void Connection::send(const char * data) {
  webSocket_.sendTXT(data);
}




