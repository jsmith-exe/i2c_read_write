#pragma once
#include "../I2CDevices.h"

// ===================== MCP9808 Temperature Sensor =====================

class MCP9808 : public I2CDevice {
public:
  MCP9808(uint8_t address, TwoWire &wire = Wire);

  // resolution: 0x00=0.5°C, 0x01=0.25°C, 0x02=0.125°C, 0x03=0.0625°C
  bool begin(uint8_t resolution = 0x03);  

  bool readTemperature(float &tempC);
  bool getFilteredTemp(float &out);
  bool getAverageTemp(float &out);

  // Set TLOWER, TUPPER, TCRIT (°C)
  bool setLimits(float t_lower, float t_upper, float t_crit);

private:
  float    decodeTemp(uint16_t raw) const;
  uint16_t encodeTemp(float tempC) const;
  bool     writeTempReg(uint8_t reg, float tempC);

  // Stored values
  float lastTempC = NAN;
  float filteredTempC = NAN;
  float avgAccumTemp = NAN;
  uint8_t avgCountTemp = NAN;
  float lastAvgTempC   = NAN;
};