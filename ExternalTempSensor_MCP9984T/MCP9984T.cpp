#include "MCP9984T.h"

// ===================== MCP9984T implementation =====================

MCP9984T::MCP9984T(uint8_t address, TwoWire &wire)
  : I2CDevice(address, wire) {}

bool MCP9984T::begin(uint16_t &id) {
    if (!device_id(id)){
        return false;
    }
    return true;
}

bool MCP9984T::device_id(uint16_t &id) {
  uint8_t buf[2];

  if (!readReg8(0x3F, buf[0])) return false;
  if (!readReg8(0x3E, buf[1])) return false;

  id = u16_be(buf);
  return true;
}

float MCP9984T::decodeExternalTemp(uint8_t high, uint8_t low) const {
  uint8_t frac_code = (low >> 5) & 0x07;
  /*Serial.print(high, HEX);
  Serial.print("   ");
  Serial.println(low, HEX);*/
  return (float)high + 0.125f * (float)frac_code;
}

bool MCP9984T::readExternalTemp(uint8_t channel, float &tempC) {
  if (channel < 1 || channel > 4) return false;

  uint8_t reg_high = 0x02 + (channel - 1) * 2;

  uint8_t buf[2];
  if (!(readReg8(reg_high, buf[0]) & readReg8(reg_high+1, buf[1]))) return false;

  tempC = decodeExternalTemp(buf[0], buf[1]);
  return true;
}

bool MCP9984T::readAllExternalTemps(float *temps, uint8_t count) {
  if (count < 3) return false;

  for (uint8_t ch = 1; ch <= 3; ch++) {
    if (!readExternalTemp(ch, temps[ch - 1])) {
      return false;
    }
  }
  return true;
}

void MCP9984T::encodeTempLimit(float tempC, uint8_t &high, uint8_t &low) const {
  // RANGE = 0: 0.000 .. 127.875°C
  if (tempC < 0.0f)       tempC = 0.0f;
  if (tempC > 127.875f)   tempC = 127.875f;

  uint8_t integer = (uint8_t)tempC;
  float frac       = tempC - (float)integer;

  // 0.125°C per LSB in bits 7..5
  uint8_t frac_code = (uint8_t)(frac / 0.125f + 0.5f);
  if (frac_code > 7) frac_code = 7;

  high = integer;
  low  = (uint8_t)(frac_code << 5);   // bits 7..5 used, 4..0 = 0
}

bool MCP9984T::setAlertThermMode(bool thermMode) {
  // Congigure register 0x22, bit 5 = Alert/Therm
  // 0 = ALERT (interrupt), 1 = THERM (comparator)

  uint8_t cfg;
  if (!readReg8(0x22, cfg)) {
    return false;
  }

  if (thermMode) {
    cfg = cfg | (1 << 5);
  }
  else {
    cfg &= ~(1 << 5);
  }

  return writeReg8(0x22, cfg);
}

bool MCP9984T::setExtAlertLimits(uint8_t channel, float alertTempC) {
  if (channel < 1 || channel > 4) return false;

  // Encode high alert limit
  uint8_t high_hi, high_lo;
  encodeTempLimit(alertTempC, high_hi, high_lo);

  // Low limit = 0°C (or something very low to effectively "disable" low-alert)
  uint8_t low_hi = 0;
  uint8_t low_lo = 0;

  uint8_t base = 0x0D + 4 * (channel - 1);  // 4-byte block per channel

  uint8_t buf_high[2] = { high_hi, high_lo };  // high-limit hi,lo
  uint8_t buf_low[2]  = { low_hi,  low_lo  };  // low-limit hi,lo

  // Write high limit (2 bytes)
  if (!writeRegN(base, buf_high, 2)) {
    return false;
  }

  // Write low limit (2 bytes)
  if (!writeRegN(base + 2, buf_low, 2)) {
    return false;
  }

  return true;
}

bool MCP9984T::setAlertLimitsAllBJTs(float alertTempC) {
  bool ok = true;
  // You’re using external diodes 1..3 for BJT1..3
  for (uint8_t ch = 1; ch <= 3; ch++) {
    ok &= setExtAlertLimits(ch, alertTempC);
  }
  return ok;
}

bool MCP9984T::setThermHysteresis(float hystC) {
  // Clamp the allowed range: 0–127 °C (device supports only integer values)
  if (hystC < 0.0f)   hystC = 0.0f;
  if (hystC > 127.0f) hystC = 127.0f;

  uint8_t val = (uint8_t)(hystC + 0.5f); // round to nearest integer

  // THERM LIMIT HYSTERESIS REGISTER (0x25)
  return writeReg8(0x25, val);
}