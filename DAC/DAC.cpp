#include "DAC.h"

// ---------- Existing readback ----------
bool DAC::readback(uint8_t &commandByte, uint16_t &dataWord) {
  uint8_t b[3] = {0, 0, 0};

  if (!readMultiByteRegN(b, 3)) {
    return false;
  }

  commandByte = b[0];
  dataWord = ((uint16_t)b[1] << 8) | b[2];
  return true;
}

bool DAC::readback24(uint32_t &word24) {
  uint8_t cmd;
  uint16_t data;
  if (!readback(cmd, data)) return false;

  word24 = ((uint32_t)cmd << 16) | (uint32_t)data;
  word24 &= 0x00FFFFFFUL;
  return true;
}

// ---------- New simple overloads (use stored config) ----------
bool DAC::setVoltage(float volts) {
  return setVoltage(volts, _defaultSel);
}

bool DAC::setVoltage(float volts, uint8_t dacSel) {
  return setVoltage(volts, _vref, _resBits, dacSel);
}

bool DAC::setCode(uint32_t code) {
  return setCode(code, _defaultSel);
}

bool DAC::setCode(uint32_t code, uint8_t dacSel) {
  return setCode(code, _resBits, dacSel);
}

// ---------- Helpers ----------

// Command byte format: b7=0, b6=S, b5..b3=cmd(3-bit), b2..b0=addr(3-bit)
uint8_t DAC::makeCmdByte(uint8_t cmd3, uint8_t addr3, bool Sbit) {
  return (uint8_t)((Sbit ? 1 : 0) << 6) | (uint8_t)((cmd3 & 0x07) << 3) | (uint8_t)(addr3 & 0x07);
}

// dacSel: 0=A, 1=B, 2=both
uint8_t DAC::addrBitsFromSel(uint8_t dacSel) {
  if (dacSel == 0) return 0b000; // DAC A
  if (dacSel == 1) return 0b001; // DAC B
  return 0b111;                  // both
}

uint8_t DAC::leftJustifyShift(uint8_t resolutionBits) {
  // Data field is 16 bits. Code must be left-justified.
  // 16-bit -> shift 0, 14-bit -> shift 2, 12-bit -> shift 4
  if (resolutionBits == 16) return 0;
  if (resolutionBits == 14) return 2;
  if (resolutionBits == 12) return 4;
  return 0;
}

bool DAC::writeFrame(uint8_t cmdByte, uint16_t dataWord) {
  bus.beginTransmission(addr);
  bus.write(cmdByte);
  bus.write((uint8_t)(dataWord >> 8));
  bus.write((uint8_t)(dataWord & 0xFF));
  return (bus.endTransmission() == 0);
}

bool DAC::setCode(uint32_t code, uint8_t resolutionBits, uint8_t dacSel) {
  // Clamp code to max for given resolution
  uint32_t maxCode = 0;
  if      (resolutionBits == 12) maxCode = (1UL << 12) - 1;
  else if (resolutionBits == 14) maxCode = (1UL << 14) - 1;
  else                           maxCode = (1UL << 16) - 1;

  if (code > maxCode) code = maxCode;

  // Left-justify into 16-bit data payload
  uint8_t shift = leftJustifyShift(resolutionBits);
  uint16_t dataWord = (uint16_t)(code << shift);

  // Command = 0b011 "Write to and update DAC channel n"
  const uint8_t cmd3 = 0b011;
  const uint8_t a3 = addrBitsFromSel(dacSel);
  const uint8_t cmdByte = makeCmdByte(cmd3, a3, false /*S=0*/);

  return writeFrame(cmdByte, dataWord);
}

bool DAC::setVoltage(float volts, float vref, uint8_t resolutionBits, uint8_t dacSel) {
  if (vref <= 0.0f) return false;

  // Clamp volts to [0, vref]
  if (volts < 0.0f) volts = 0.0f;
  if (volts > vref) volts = vref;

  uint32_t maxCode = 0;
  if      (resolutionBits == 12) maxCode = (1UL << 12) - 1;
  else if (resolutionBits == 14) maxCode = (1UL << 14) - 1;
  else                           maxCode = (1UL << 16) - 1;

  // Convert volts -> code with rounding
  float ratio = volts / vref;
  uint32_t code = (uint32_t)(ratio * (float)maxCode + 0.5f);

  return setCode(code, resolutionBits, dacSel);
}
