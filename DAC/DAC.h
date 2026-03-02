#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "../I2CDevices.h"

class DAC : public I2CDevice {
public:
  // dacSel: 0=A, 1=B, 2=both
  explicit DAC(uint8_t i2cAddr, TwoWire &wire = Wire)
  : I2CDevice(i2cAddr, wire) {}

  // Stores config + optional probe + optional force known output
  bool begin(float vrefVolts,
             uint8_t resolutionBits = 16,
             uint8_t defaultSel = 0,
             bool forceKnownState = false);

  // Simple I2C ACK check (base class doesn't provide this)
  bool probe();

  // Readbacks
  bool readback(uint8_t &commandByte, uint16_t &dataWord);
  bool readback24(uint32_t &word24);

  // Simple user-facing calls (use stored config)
  bool setVoltage(float volts);                 // uses defaultSel
  bool setVoltage(float volts, uint8_t dacSel); // override channel
  bool setCode(uint32_t code);                  // uses defaultSel
  bool setCode(uint32_t code, uint8_t dacSel);  // override channel

  // Advanced calls
  bool setCode(uint32_t code, uint8_t resolutionBits, uint8_t dacSel);
  bool setVoltage(float volts, float vref, uint8_t resolutionBits, uint8_t dacSel);

private:
  // Helpers
  static uint8_t makeCmdByte(uint8_t cmd3, uint8_t addr3, bool Sbit);
  static uint8_t addrBitsFromSel(uint8_t dacSel);
  static uint8_t leftJustifyShift(uint8_t resolutionBits);

  // Send a DAC frame using I2CDevice::writeRegN (no manual bus writes)
  bool writeFrame(uint8_t cmdByte, uint16_t dataWord);

  // Stored config
  float   _vref       = 3.28f;
  uint8_t _resBits    = 16;
  uint8_t _defaultSel = 0;
};