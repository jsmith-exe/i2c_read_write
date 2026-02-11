#include "MCP9808.h"

// ===================== MCP9808 implementation =====================

MCP9808::MCP9808(uint8_t address, TwoWire &wire)
  : I2CDevice(address, wire) {}

bool MCP9808::begin(uint8_t resolution) {
  // Resolution register (0x08):
  // 00 = 0.5°C, 01 = 0.25°C, 10 = 0.125°C, 11 = 0.0625°C
  if (!writeReg8(0x08, resolution & 0x03)) {
    return false;
  }
  return true;
}

float MCP9808::decodeTemp(uint16_t raw) const {
  // MCP9808 format:
  // bit15..13: flags
  // bit12    : sign bit
  // bit11..0 : magnitude (0.0625°C per LSB)
  uint16_t t12 = raw & 0x0FFF;
  bool neg = raw & 0x1000;
  float temp = t12 * 0.0625f;
  if (neg) temp -= 256.0f;
  return temp;
}

uint16_t MCP9808::encodeTemp(float tempC) const {
  bool neg = (tempC < 0.0f);

  if (neg) {
    tempC += 256.0f;
  }

  uint16_t t12 = (uint16_t)(tempC / 0.0625f) & 0x0FFF;

  if (neg) {
    t12 |= 0x1000;
  }

  return t12;
}

bool MCP9808::readTemperature(float &tempC) {
  uint8_t buf[2];
  if (!readRegN(0x05, buf, 2)) {    // ambient temperature register
    return false;
  }
  uint16_t raw = u16_be(buf);
  tempC = decodeTemp(raw);

  // Save for whole class acces
  lastTempC = tempC;
  return true;
}

bool MCP9808::writeTempReg(uint8_t reg, float tempC) {
  uint16_t raw = encodeTemp(tempC);
  uint8_t data[2] = {
    (uint8_t)(raw >> 8),
    (uint8_t)(raw & 0xFF)
  };
  return writeRegN(reg, data, 2);
}

bool MCP9808::setLimits(float t_lower, float t_upper, float t_crit) {
  bool ok = true;
  ok &= writeTempReg(0x03, t_lower);  // TLOWER
  ok &= writeTempReg(0x02, t_upper);  // TUPPER
  ok &= writeTempReg(0x04, t_crit);   // TCRIT
  return ok;
}

bool MCP9808::getFilteredTemp(float &out) {

  if (isnan(lastTempC)) {
    return false;
  }

  // Set fitler strength
  const float alpha = 0.2f;

  filteredTempC = lowpass(lastTempC, filteredTempC, alpha);

  out = filteredTempC;

  return true;
}

bool MCP9808::getAverageTemp(float &output_av) {

  if (isnan(lastTempC)) {
    return false;
  }
  
  float newAvg;

  // Set sample window
  const uint16_t sampleWindow = 10;
  
  if (accumulateAverage(lastTempC, avgAccumTemp, avgCountTemp, sampleWindow, newAvg)) lastAvgTempC = newAvg;

  if (isnan(lastAvgTempC)) {
    return false;
  }

  output_av = lastAvgTempC;

  return true;
}