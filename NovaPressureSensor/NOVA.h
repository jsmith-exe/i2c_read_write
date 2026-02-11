#pragma once
#include "../I2CDevices.h"

// ===================== Nova NPI-19 Pressure Sensor =====================

class NOVA_PRESSURE : public I2CDevice {
public:
  NOVA_PRESSURE(uint8_t address, TwoWire &wire = Wire);

  bool read();   // prints to Serial as above
};