#include "SLFS4000B.h"

// ===================== SLF3S-4000B implementation =====================

SLF3S4000B::SLF3S4000B(uint8_t address, TwoWire &wire)
  : I2CDevice(address, wire) {}

bool SLF3S4000B::begin() {
    return true;
}

// CRC-8 for Sensirion (poly 0x31, init 0xFF)
uint8_t SLF3S4000B::crc8(const uint8_t *data, size_t len) const {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool SLF3S4000B::startWater() {
  // Sensirion command: just send the 2-byte command, no register index
  uint8_t reg = 0x36;
  uint8_t data = 0x8;
  if (!writeReg8(reg,data)) return false;

  return true;
}

bool SLF3S4000B::stop() {
  uint8_t reg = 0x3F;
  uint8_t data = 0xF9;
  if (!writeReg8(reg,data)) return false;

  return true;
}

bool SLF3S4000B::read(float &flow_ml_min, float &temp_C, uint16_t &flags) {
  const uint8_t NumberBytes = 9;
  uint8_t buf[NumberBytes];

  readMultiByteRegN(buf, NumberBytes);

  if (crc8(&buf[0], 2) != buf[2]) return false;
  if (crc8(&buf[3], 2) != buf[5]) return false;
  if (crc8(&buf[6], 2) != buf[8]) return false;

  if(!decodeData(buf, flow_ml_min, temp_C, flags)) return false;

  return true;
}

bool SLF3S4000B::decodeData(const uint8_t *buf, float &flow_ml_min, float &temp_C, uint16_t &flags) 
{
    // Extract signed 16-bit values from big-endian format
    int16_t raw_flow = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_temp = (int16_t)((buf[3] << 8) | buf[4]);
    flags            = (uint16_t)((buf[6] << 8) | buf[7]);

    // Sensirion SLF3S-4000B scaling
    flow_ml_min = raw_flow / 32.0f;
    temp_C      = raw_temp / 200.0f;

    lastFlow = flow_ml_min;

    return true;
}

void SLF3S4000B::printFlags(Stream &out, uint16_t f) {
  if (f & 0b00000001) out.print("AIR_IN_LINE, ");
  if (f & 0b00000010) out.print("HIGHFLOW, ");
  if (f & 0x00010000) out.print("EXP_SMOOTHING, ");
}

bool SLF3S4000B::resetSensor() {
  stop();
  delay(5);

  uint8_t reg = 0x0;
  uint8_t data = 0x6;
  if (!writeReg8(reg,data)) return false;

  delay(20);
  bool ok = startWater();
  delay(10);
  return ok;
}

bool SLF3S4000B::getFilteredFlow(float &out) {

  if (isnan(lastFlow)) {
    return false;
  }

  // Set fitler strength
  const float alpha = 0.075f;

  filteredFlow = lowpass(lastFlow, filteredFlow, alpha);

  out = filteredFlow;

  return true;
}

bool SLF3S4000B::getAverageFlow(float &out) {

  if (isnan(lastFlow)) {
    return false;
  }
  
  float newAvg;

  // Set sample window
  const uint16_t sampleWindow = 30;
  
  if (accumulateAverage(lastFlow, avgAccumFlow, avgCountFlow, sampleWindow, newAvg)) lastAvgFlow = newAvg;

  if (isnan(lastAvgFlow)) {
    return false;
  }

  out = lastAvgFlow;

  return true;
}