#pragma once
#include "../I2CDevices.h"

// ===================== SLF3S-4000B Flow Sensor =====================

class SLF3S4000B : public I2CDevice {
public:
  SLF3S4000B(uint8_t address = 0x08, TwoWire &wire = Wire);

  bool begin();                // nothing special yet, kept for symmetry
  bool startWater();           // start continuous measurement (H2O)
  bool stop();                 // stop continuous measurement

  bool read(float &flow_ml_min, float &temp_C, uint16_t &flags, float &byte1, float &byte2, float &CRC);
  bool decodeData(const uint8_t *buf, float &flow_ml_min, float &temp_C, uint16_t &flags, float &byte1, float &byte2, float &CRC) ;

  bool getFilteredFlow(float &out);
  bool getAverageFlow(float &out);


  bool resetSensor();

  static void printFlags(Stream &out, uint16_t f);

private:
  uint8_t crc8(const uint8_t *data, size_t len) const;

  // Stored values
  float lastFlow = NAN;
  float filteredFlow = NAN;
  float avgAccumFlow = NAN;
  uint8_t avgCountFlow = NAN;
  float lastAvgFlow   = NAN;
};