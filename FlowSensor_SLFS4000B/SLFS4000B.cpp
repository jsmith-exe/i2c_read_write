#include "SLFS4000B.h"

// ===================== SLF3S-4000B implementation =====================

SLF3S4000B::SLF3S4000B()
: I2CDevice(SLF3S4000B_ADDR, Wire) 
{}

bool SLF3S4000B::begin() 
{
  live_flow = startWater();
  return live_flow;
}

bool SLF3S4000B::startWater() 
{
  // Sensirion command: just send the 2-byte command, no register index
  uint8_t reg = 0x36;
  uint8_t data = 0x8;
  if (!writeReg8(reg, data)) 
  {
    live_flow = false;
    return false;
  }

  live_flow = true;
  return true;
}

bool SLF3S4000B::stop() 
{
  uint8_t reg = 0x3F;
  uint8_t data = 0xF9;
  if (!writeReg8(reg, data))
  {
    return false;
  }

  live_flow = false;
  return true;
}

bool SLF3S4000B::update()
{
  if (!live_flow)
  {
    live_flow = startWater();

    if (!live_flow)
    {
      return false;
    }

    delay(10);
  }

  float new_flow_ml_min = 0.0f;
  float new_temp_C = 0.0f;
  uint16_t new_flags = 0;

  if (!readSample(new_flow_ml_min, new_temp_C, new_flags))
  {
    return false;
  }

  flow_ml_min = new_flow_ml_min;
  temp_C = new_temp_C;
  flow_flags = new_flags;

  return true;
}

bool SLF3S4000B::readSample(float &flow_ml_min, float &temp_C, uint16_t &flags) 
{
  const uint8_t NumberBytes = 9;
  uint8_t buf[NumberBytes];

  readMultiByteRegN(buf, NumberBytes);

  if (crc8(&buf[0], 2) != buf[2]) return false;
  if (crc8(&buf[3], 2) != buf[5]) return false;
  if (crc8(&buf[6], 2) != buf[8]) return false;

  if(!decodeData(buf, flow_ml_min, temp_C, flags)) return false;

  return true;
}

// CRC-8 for Sensirion (poly 0x31, init 0xFF)
uint8_t SLF3S4000B::crc8(const uint8_t *data, size_t len) const 
{
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) 
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) 
    {
      if (crc & 0x80) 
      {
        crc = (crc << 1) ^ 0x31;
      } else 
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool SLF3S4000B::decodeData(const uint8_t *buf, float &flow_ml_min, float &temp_C, uint16_t &flags) 
{
  int16_t raw_flow = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t raw_temp = (int16_t)((buf[3] << 8) | buf[4]);

  flags = (uint16_t)((buf[6] << 8) | buf[7]);

  flow_ml_min = raw_flow / 32.0f;
  temp_C = raw_temp / 200.0f;

  return true;
}

float SLF3S4000B::getFlowMlmin() const
{
  return flow_ml_min;
}

float SLF3S4000B::getTempC() const
{
  return temp_C;
}

uint16_t SLF3S4000B::getFlags() const
{
  return flow_flags;
}

bool SLF3S4000B::isLive() const
{
  return live_flow;
}

void SLF3S4000B::printFlags(Stream &out, uint16_t flags) 
{
  if (flags & 0b00000001) out.print("AIR_IN_LINE, ");
  if (flags & 0b00000010) out.print("HIGHFLOW, ");
  if (flags & 0x00010000) out.print("EXP_SMOOTHING, ");
}