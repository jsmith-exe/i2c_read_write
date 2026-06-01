#pragma once
#include "../I2CDevices.h"

// ===================== SLF3S-4000B Flow Sensor =====================

class SLF3S4000B : public I2CDevice {
public:
  SLF3S4000B(uint8_t address = 0x08, TwoWire &wire = Wire);

  bool begin();                // nothing special yet, kept for symmetry
  bool startWater();           // start continuous measurement (H2O)
  bool stop();                 // stop continuous measurement

  // Raw sensor read (mL/min and °C)
  bool read(float &flow_ml_min, float &temp_C, uint16_t &flags);
  bool decodeData(const uint8_t *buf,
                  float &flow_ml_min,
                  float &temp_C,
                  uint16_t &flags);

  // Processed flow helpers
  bool getFilteredFlow(float &out);          // filtered flow in mL/min
  bool getAverageFlow(float &out);           // averaged flow in mL/min
  bool getFilteredFlow_lpm(float &out);      // filtered flow in L/min
  bool getCorrectedFlowRate_lpm(float &out); // corrected flow in L/min
  bool getCorrectedFlowRate_mlpm(float &out);// corrected flow in mL/min

  bool resetSensor();

  static void printFlags(Stream &out, uint16_t f);

private:
  uint8_t crc8(const uint8_t *data, size_t len) const;

  // Error model (piecewise linear e(f) in L/min)
  float computeErrorRateLpm(float flow_lpm) const;

  // Stored values
  float lastFlow              = NAN;   // mL/min
  float filteredFlow          = NAN;   // mL/min

  float avgAccumFlow          = 0.0f;  // accumulator for averaging
  uint8_t avgCountFlow       = 0;     // number of samples in average
  float lastAvgFlow           = NAN;   // last computed average (mL/min)

  float errorRate             = NAN;   // latest error e(f)
  float filteredFlow_lpm      = NAN;   // filtered flow in L/min
  float correctedFlowRate_lpm = NAN;   // corrected flow in L/min
  float correctedFlowRate_mlpm = NAN;  // corrected flow in mL/min
};