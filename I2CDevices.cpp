#include "I2CDevices.h"

// ===================== I2CDevice base =====================

I2CDevice::I2CDevice(uint8_t address, TwoWire &wire)
  : bus(wire), addr(address) {}

bool I2CDevice::begin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);
  return true;
}

bool I2CDevice::readRegN(uint8_t reg, uint8_t *buf, size_t n) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) { // repeated start
    return false;
  }
  if (bus.requestFrom((int)addr, (int)n) != (int)n) {
    return false;
  }
  for (size_t i = 0; i < n; i++) {
    buf[i] = bus.read();
  }
  return true;
}

bool I2CDevice::readMultiByteRegN(uint8_t *buf, size_t n) {
  int got = bus.requestFrom((int)addr, (int)n);

  if (got != (int)n){
    return false;
  }

  for (uint8_t i=0; i<n; i++) {
    if (!bus.available()) {
      return false;
    }
    buf[i] = bus.read();
  }

  return true;
}

bool I2CDevice::writeRegN(uint8_t reg, const uint8_t *data, size_t n) {
  bus.beginTransmission(addr);
  bus.write(reg);
  for (size_t i = 0; i < n; i++) {
    bus.write(data[i]);
  }
  return bus.endTransmission() == 0;
}

bool I2CDevice::readReg8(uint8_t reg, uint8_t &val) {
  return readRegN(reg, &val, 1);
}

bool I2CDevice::writeReg8(uint8_t reg, uint8_t val) {
  return writeRegN(reg, &val, 1);
}

uint16_t I2CDevice::u16_be(const uint8_t *b) {
  return ((uint16_t)b[0] << 8) | b[1];
}

int16_t I2CDevice::s16_be(const uint8_t *b) {
  return (int16_t)(((uint16_t)b[0] << 8) | b[1]);
}

float I2CDevice::lowpass(float input, float &state, float alpha) {
  // Clamp alpha to a sane range
  if (alpha <= 0.0f) alpha = 0.0f;
  if (alpha >= 1.0f) alpha = 1.0f;

  // If sate is NaN (e.g. first run), just initialise it
  if (isnan(state)) {
    state = input;
    return state;
  }

  state = state + (alpha * (input - state));
  return state;
}

bool I2CDevice::accumulateAverage(float input, float &accum, uint8_t &count, uint8_t window, float &avgOut) {
  // Avoid div-by-zero; treat as "no avaeraging"
  if (window == 0) return true;

  accum = accum + input;
  count++;

  if (count >= window) {
    avgOut = accum / (float)window;
    // reset for next block
    accum = 0.0f;
    count = 0;
    return true;
  }

  return false;
}