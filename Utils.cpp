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

  String fields[19];
  String copy;
  char *str;
  int i = 0;
  while ((str = strtok_r(transmission, ",", &transmission)) != NULL) {
    copy = String(str);
    fields[i] = copy;
    i++;
  } // delimiter is the semicolon

  String output[19];
  sprintf(output[0], "{\"data\": [{\"id\": \"%s\",", fields[0]);
  sprintf(output[1], "\"VA_MAG\": %s,", fields[1]);
  sprintf(output[2], "\"VB_MAG\": %s,", fields[2]);
  sprintf(output[3], "\"VC_MAG\": %s,", fields[3]);
  sprintf(output[4], "\"IA_MAG\": %s,", fields[4]);
  sprintf(output[5], "\"IB_MAG\": %s,", fields[5]);
  sprintf(output[6], "\"IC_MAG\": %s,", fields[6]);
  sprintf(output[7], "\"VA_ANG\": %s,", fields[7]);
  sprintf(output[8], "\"VB_ANG\": %s,", fields[8]);
  sprintf(output[9], "\"VC_ANG\": %s,", fields[9]);
  sprintf(output[10], "\"IA_ANG\": %s,", fields[10]);
  sprintf(output[11], "\"IB_ANG\": %s,", fields[11]);
  sprintf(output[12], "\"IC_ANG\": %s,", fields[12]);
  sprintf(output[13], "\"POW_FACTOR\": %s,", fields[13]);
  sprintf(output[14], "\"POW_APPARENT\": %s,", fields[14]);
  sprintf(output[15], "\"POW_ACTIVE\": %s,", fields[15]);
  sprintf(output[16], "\"POW_REACTIVE\": %s,", fields[16]);
  sprintf(output[17], "\"board_info\": {\"board_id\": \"%s\", ", fields[17]);
  sprintf(output[18], "\"ade_id\": %s}}]}", fields[18]);

  String data = "";
  for(int i = 0; i < 19; i++) {
    data = data + output[i];
  }

  return data;
  /*
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
  */
}