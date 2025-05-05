#ifndef UTILS_H
#define UTILS_H

#include <string.h>
#include <ArduinoJson.h>

#define DEL 127

bool isNumeric(String str);

void flushInputBuffer();

String getUserInput(String prompt);

String generate_json_energy_record(char *transmission);

String generate_json_energy_record_cb(char *transmission)

#endif