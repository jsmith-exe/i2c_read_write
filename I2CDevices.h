#pragma once
#include <Arduino.h>
#include <Wire.h>

// ===================== Base I2C device =====================

class I2CDevice {
public:
  I2CDevice(uint8_t address, TwoWire &wire = Wire);

  bool begin();        
  uint8_t getAddress() const { return addr; }


protected:
  static constexpr uint8_t SDA_PIN = 3;
  static constexpr uint8_t SCL_PIN = 2;
  static constexpr uint32_t I2C_FREQ = 400000;
  bool readRegN(uint8_t reg, uint8_t *buf, size_t n);
  bool readMultiByteRegN(uint8_t *buf, size_t n);
  bool writeRegN(uint8_t reg, const uint8_t *data, size_t n);
  bool readReg8(uint8_t reg, uint8_t &val);
  bool writeReg8(uint8_t reg, uint8_t val);

  static uint16_t u16_be(const uint8_t *b);   // big-endian unsigned
  static int16_t  s16_be(const uint8_t *b);   // big-endian signed

  float lowpass(float input, float &state, float alpha);
  bool accumulateAverage(float input, float &accum, uint8_t &count, uint8_t window, float &avgOut);

  TwoWire &bus;
  uint8_t  addr;
  
};