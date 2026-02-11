#pragma once
#include "../I2CDevices.h"   // if this provides I2CDevice base / readMultiByteRegN etc.

class DAC : public I2CDevice {
public:
  // dacSel: 0=A, 1=B, 2=both
  explicit DAC(uint8_t i2cAddr, TwoWire &wire = Wire)
    : I2CDevice(i2cAddr, wire), addr(i2cAddr), bus(wire) {}

  // Configure defaults once (call in setup)
  void begin(float vrefVolts, uint8_t resolutionBits = 16, uint8_t defaultSel = 0) {
    _vref = vrefVolts;
    _resBits = resolutionBits;
    _defaultSel = defaultSel;
  }

  // Existing readbacks
  bool readback(uint8_t &commandByte, uint16_t &dataWord);
  bool readback24(uint32_t &word24);

  // Simple user-facing calls (use stored config)
  bool setVoltage(float volts);                 // uses defaultSel
  bool setVoltage(float volts, uint8_t dacSel); // override channel
  bool setCode(uint32_t code);                  // uses defaultSel
  bool setCode(uint32_t code, uint8_t dacSel);  // override channel

  // Advanced calls (still available if you want)
  bool setCode(uint32_t code, uint8_t resolutionBits, uint8_t dacSel);
  bool setVoltage(float volts, float vref, uint8_t resolutionBits, uint8_t dacSel);

private:
  // New helpers
  uint8_t makeCmdByte(uint8_t cmd3, uint8_t addr3, bool Sbit);
  uint8_t addrBitsFromSel(uint8_t dacSel);
  uint8_t leftJustifyShift(uint8_t resolutionBits);
  bool writeFrame(uint8_t cmdByte, uint16_t dataWord);

  // Stored config
  float   _vref = 3.3f;
  uint8_t _resBits = 16;
  uint8_t _defaultSel = 0;

  // Keep these if your base class doesn’t already store them
  uint8_t  addr;
  TwoWire &bus;
};
