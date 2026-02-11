#pragma once
#include "../I2CDevices.h"

// ===================== MCP9984T-1E/E3 Temperature Sensor =====================

class MCP9984T : public I2CDevice {
public:
  MCP9984T(uint8_t address, TwoWire &wire = Wire);

  bool begin(uint16_t &id); // nothing special yet, kept for symmetry
  bool device_id(uint16_t &id);
  bool readExternalTemp(uint8_t channel, float &tempC);
  bool readAllExternalTemps(float *temps, uint8_t count);

  // false = ALERT/interrupt mode, true = THERM/comparator mode
  bool setAlertThermMode(bool thermMode);

  // Optional helpers (if you want ALERT thresholds on BJTs)
  bool setExtAlertLimits(uint8_t channel, float alertTempC);
  bool setAlertLimitsAllBJTs(float alertTempC);
  bool setThermHysteresis(float hystC);

private:
  float decodeExternalTemp(uint8_t high, uint8_t low) const;
  void encodeTempLimit(float tempC, uint8_t &high, uint8_t &low) const;
};