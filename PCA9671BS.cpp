#include <Arduino.h>
#include "PCA9671BS.h"

PCA9671BS::PCA9671BS(uint8_t const address) {
  address_ = address;

  /* or read from the device?? */
  first_byte_val_ = 0x00;
  second_byte_val_ = 0x3f;
  //write_bytes(first_byte_val_, second_byte_val_);
}

void PCA9671BS::begin(uint8_t const I2C_SDA_pin, uint8_t const I2C_SCL_pin) {
  /* reset first ?? */
  Wire.begin(I2C_SDA_pin, I2C_SCL_pin);
}

bool PCA9671BS::write_bytes(uint8_t const first_byte, uint8_t const second_byte) {
  bool ack1, ack2; 

  Wire.beginTransmission(address_);
  Wire.write(first_byte); // write the first byte
  ack1 = static_cast<bool>(Wire.endTransmission());
  Wire.write(second_byte); // write the second byte
  ack2 = static_cast<bool>(Wire.endTransmission());
  return ack1 && ack2;
}

bool PCA9671BS::write_first_byte(uint8_t const first_byte) {
  return write_bytes(first_byte, second_byte_val_);
}

bool PCA9671BS::write_second_byte(uint8_t const second_byte) {
  return write_bytes(first_byte_val_, second_byte);
}

bool PCA9671BS::digital_write(uint8_t const pin_number, bool const value) {
  if (pin_number < PCA_PIN_P10) {
    if (value) {
      first_byte_val_ |= (1 << pin_number);
    } else {
      first_byte_val_ &= ~(1 << pin_number);
    }
  } else {
    if (value) {
      second_byte_val_ |= (1 << (pin_number - 8));
    } else {
      second_byte_val_ &= ~(1 << (pin_number - 8));
    }
  }
  return write_bytes(first_byte_val_, second_byte_val_); 
}


uint16_t PCA9671BS::read_bytes() {
  // should return the read bytes
  Wire.requestFrom(address_, (uint8_t) 2);
  if (Wire.available() >= 2) {
    byte data1 = Wire.read();
    byte data2 = Wire.read();
    // Print the received data to the serial monitor
    Serial.print("Firts byte: ");
    Serial.println(data1, BIN);
    Serial.print("Second byte: ");
    Serial.println(data2, BIN);
  }
  return 1;
}

uint8_t PCA9671BS::read_first_byte() {
  /* TODO */
  return 1;
}

uint8_t PCA9671BS::read_second_byte() {
  /* TODO */
  return 1;
}

bool PCA9671BS::digital_read(uint8_t const pin_value) {
  /* TODO */
  return true;
}

bool PCA9671BS::reset() {
  /* TODO */
  return true;
}
