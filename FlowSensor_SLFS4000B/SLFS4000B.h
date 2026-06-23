#pragma once
#include "../I2CDevices.h"

// ===================== SLF3S-4000B Flow Sensor =====================

class SLF3S4000B : public I2CDevice {
public:
  SLF3S4000B(uint8_t address = SLF3S4000B_ADDR, TwoWire &wire = Wire);

  bool begin();                // nothing special yet, kept for symmetry
  bool startWater();           // start continuous measurement (H2O)
  bool stop();                 // stop continuous measurement

  bool read(float &flow_ml_min, float &temp_C, uint16_t &flags);
  bool decodeData(const uint8_t *buf, float &flow_ml_min, float &temp_C, uint16_t &flags) ;

  bool getFilteredFlow(float &out);
  bool getAverageFlow(float &out);


  bool resetSensor();

  void update();
  void printFlow();

  static void printFlags(Stream &out, uint16_t f);

private:

  static constexpr uint8_t SLF3S4000B_ADDR = 0x08;
  uint8_t crc8(const uint8_t *data, size_t len) const;

  // Stored values
  float lastFlow = NAN;
  float filteredFlow = NAN;
  float avgAccumFlow = NAN;
  uint8_t avgCountFlow = NAN;
  float lastAvgFlow   = NAN;

  float flow_ml_min;
  float temp_C ;
  uint16_t flowFlags = 0;
  float flow_filtered_ml_min;

  bool liveFlow = false;
};