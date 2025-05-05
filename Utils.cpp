#include <Arduino.h>
#include "Utils.h"

bool isNumeric(String str) {
  for (int i = 0; i < str.length(); i++) {
    if (!isDigit(str.charAt(i))) {
      return false;
    }
  }
  return true;
}

void flushInputBuffer() {
  Serial.flush();
  while(Serial.read() != -1);
}

String getUserInput(String prompt) {
  Serial.print(prompt);
  delay(10);
  String out = "";
  flushInputBuffer();
  while(1) {
    if (Serial.available()) {
      char input = Serial.read();
      if (input == '\r') {
        Serial.println();
        break;
      } else if (input == DEL && out.length() > 0) {
        out.remove(out.length() - 1);
        Serial.write("\b \b");
      } else {
        out += input;
        Serial.write(input);
      }
    }
  }
  return out;
}

String generate_json_energy_record(char *transmission) {
  // Create a DynamicJsonDocument
  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();
  JsonArray data = root.createNestedArray("data");
  JsonObject dataObj = data.createNestedObject();

  String fields[19];
  String copy;
  char *str;
  int i = 0;
  while ((str = strtok_r(transmission, ",", &transmission)) != NULL) {
    copy = String(str);
    fields[i] = copy;
    i++;
  } // delimiter is the semicolon
  
  // Add data to the document
  dataObj["id"] = fields[0];
  dataObj["VA_MAG"] = fields[1].toFloat();
  dataObj["VB_MAG"] = fields[2].toFloat();
  dataObj["VC_MAG"] = fields[3].toFloat();
  dataObj["IA_MAG"] = fields[4].toFloat();
  dataObj["IB_MAG"] = fields[5].toFloat();
  dataObj["IC_MAG"] = fields[6].toFloat();
  dataObj["VA_ANG"] = fields[7].toFloat();
  dataObj["VB_ANG"] = fields[8].toFloat();
  dataObj["VC_ANG"] = fields[9].toFloat();
  dataObj["IA_ANG"] = fields[10].toFloat();
  dataObj["IB_ANG"] = fields[11].toFloat();
  dataObj["IC_ANG"] = fields[12].toFloat();
  dataObj["POW_FACTOR"] = fields[13].toFloat();
  dataObj["POW_APPARENT"] = fields[14].toFloat();
  dataObj["POW_ACTIVE"] = fields[15].toFloat();
  dataObj["POW_REACTIVE"] = fields[16].toFloat();
  JsonObject boardInfo = dataObj.createNestedObject("board_info");
  boardInfo["board_id"] = fields[17];
  boardInfo["ade_id"] = fields[18].toInt();
  
  
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

String generate_json_energy_record_cb(char *transmission){
  // Create a DynamicJsonDocument
  DynamicJsonDocument doc(2048);
  JsonObject root = doc.to<JsonObject>();
  JsonArray data = root.createNestedArray("data");
  JsonObject dataObj = data.createNestedObject();

  String fields[3];
  String copy;
  char *str;
  int i = 0;
  while ((str = strtok_r(transmission, ",", &transmission)) != NULL) {
    copy = String(str);
    fields[i] = copy;
    i++;
  } // delimiter is the semicolon
  
  // Add data to the document
  dataObj["id"] = fields[0];
  dataObj["VA_MAG"] = fields[1].toFloat();
  dataObj["VB_MAG"] = 0.0;
  dataObj["VC_MAG"] = 0.0;
  dataObj["IA_MAG"] = fields[2].toFloat();
  dataObj["IB_MAG"] = 0.0;
  dataObj["IC_MAG"] = 0.0;
  dataObj["VA_ANG"] = 0.0;
  dataObj["VB_ANG"] = 0.0;
  dataObj["VC_ANG"] = 0.0;
  dataObj["IA_ANG"] = 0.0;
  dataObj["IB_ANG"] = 0.0;
  dataObj["IC_ANG"] = 0.0;
  dataObj["POW_FACTOR"] = 1.0;
  dataObj["POW_APPARENT"] = 1200.0;
  dataObj["POW_ACTIVE"] = 1200.0;
  dataObj["POW_REACTIVE"] = 0.0;
  JsonObject boardInfo = dataObj.createNestedObject("board_info");
  boardInfo["board_id"] = "Nate's House";
  boardInfo["ade_id"] = 0;
  
  
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}