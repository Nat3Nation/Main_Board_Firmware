/*
  PCA9671BS.h - Library for PCA9671BS - An 8 to 16 IO Expander
  Author: alekdunja
*/
#ifndef PCA9671BS_h
#define PCA9671BS_h

#include "Arduino.h"
#include <Wire.h>

#define PCA_PIN_P00 0
#define PCA_PIN_P01 1
#define PCA_PIN_P02 2
#define PCA_PIN_P03 3
#define PCA_PIN_P04 4
#define PCA_PIN_P05 5
#define PCA_PIN_P06 6
#define PCA_PIN_P07 7
#define PCA_PIN_P10 8
#define PCA_PIN_P11 9
#define PCA_PIN_P12 10
#define PCA_PIN_P13 11
#define PCA_PIN_P14 12
#define PCA_PIN_P15 13
#define PCA_PIN_P16 14
#define PCA_PIN_P17 15

class PCA9671BS
{
  public:
    /* constructor */
    PCA9671BS(uint8_t const address = 0x20);
    /* initialize the I2C communication */
    void begin(uint8_t const I2C_SDA_pin = 23, uint8_t const I2C_SCL_pin = 22);
    /* write both bytes to the expander */
    bool write_bytes(uint8_t const first_byte, uint8_t const second_byte);
    /* only write to the first byte (pins PP00 to PP07) */
    bool write_first_byte(uint8_t const first_byte);
    /* only write to the second byte (pins PP10 to PP17) */
    bool write_second_byte(uint8_t const second_byte);
    /* write to the specified pin */
    bool digital_write(uint8_t const pin_value, bool const value);
    /* TODO: read byte statuses */
    uint16_t read_bytes();
    /* TODO: only read the first byte (pins PP00 to PP07) */ 
    uint8_t read_first_byte();
    /* TODO: only read the first byte (pins PP10 to PP17) */ 
    uint8_t read_second_byte();
    /* TODO: read the value of the specified pin */
    bool digital_read(uint8_t const pin_value);
    /* TODO: reset device */
    bool reset();

  private:
    /* TODO: should I be storing the two bytes or read them every time ? */
    /* create a funciton that check whether the bytes are the same as the directly read ones ? */
    uint8_t first_byte_val_;
    uint8_t second_byte_val_;
    uint8_t address_;
};

static PCA9671BS expander;

#endif