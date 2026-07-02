#pragma once
#include "../I2CDevices.h"

// ===================== SLF3S-4000B Flow Sensor =====================

class SLF3S4000B : public I2CDevice 
{
public:
  SLF3S4000B();
  bool begin();     

  bool startWater();           
  bool stop();                 

  bool update();

  float getFlowMlmin() const;
  float getTempC() const;
  uint16_t getFlags() const;
  bool isLive() const;

  static void printFlags(Stream &out, uint16_t flags);

private:

  static constexpr uint8_t SLF3S4000B_ADDR = 0x08;

  bool readSample(float &flow_ml_min, float &temp_C, uint16_t &flags);
  bool decodeData(const uint8_t *buf, float &flow_ml_min, float &temp_C, uint16_t &flags);

  uint8_t crc8(const uint8_t *data, size_t len) const;

  float flow_ml_min;
  float temp_C;
  uint8_t flow_flags = 0;

  bool live_flow = false;
};