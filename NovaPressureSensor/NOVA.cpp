#include "NOVA.h"

NOVA_PRESSURE::NOVA_PRESSURE(uint8_t address, TwoWire &wire)
  : I2CDevice(address, wire) {}

bool NOVA_PRESSURE::read() { /*
  uint8_t bytes = 4;
  uint8_t buf[bytes];
  if (!readMultiByteRegN(buf, bytes)){
    return false;
  }
  
  if (false) {
  Serial.print(buf[3]);Serial.print(", ");Serial.print(buf[2]);Serial.print(", ");Serial.print(buf[1]);Serial.print(", ");Serial.print(buf[0]);Serial.print(", ");
  Serial.println(" ");
  }

  uint16_t counts = ((uint16_t)buf[1]& << 8) | buf[2];
  uint8_t rawTemp = buf[3];   // Already an 8-bit temperature
  uint8_t status = buf[0];

  if (true) {
    Serial.print(counts);Serial.print(", ");Serial.print(rawTemp);Serial.print(", ");Serial.print(status);
    Serial.println("");

  }

  return true;*/
} 