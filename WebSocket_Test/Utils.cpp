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

  String fields[18];
  String copy;
  char *str;
  int i = 0;
  while ((str = strtok_r(transmission, ",", &transmission)) != NULL) {
    copy = String(str);
    fields[i] = copy;
    i++;
  } // delimiter is the semicolon

  // Add data to the document
  doc["action"] = fields[0];
  doc["ade_id"] = fields[1];
  // Create a nested object
  JsonObject payload = doc.createNestedObject("payload");
  payload["VA_MAG"] = fields[2];
  payload["VB_MAG"] = fields[3];
  payload["VC_MAG"] = fields[4];
  payload["IA_MAG"] = fields[5];
  payload["IB_MAG"] = fields[6];
  payload["IC_MAG"] = fields[7];
  payload["VA_ANG"] = fields[8];
  payload["VB_ANG"] = fields[9];
  payload["VC_ANG"] = fields[10];
  payload["IA_ANG"] = fields[11];
  payload["IB_ANG"] = fields[12];
  payload["IC_ANG"] = fields[13];
  payload["POW_FACTOR"] = fields[14];
  payload["POW_APPARENT"] = fields[15];
  payload["POW_ACTIVE"] = fields[16];
  payload["POW_REACTIVE"] = fields[17];
  
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}