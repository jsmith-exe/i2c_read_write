#include <Arduino.h>
#include <Wire.h>

// -------------------- Config --------------------
#define SDA_PIN 3
#define SCL_PIN 2
#define SENSOR_ADDR 0x08

const uint32_t I2C_FREQ = 400000;   // start at 100 kHz for stability

// -------------------- CRC-8 (Sensirion) --------------------
// Polynomial: 0x31
// Init: 0xFF
uint8_t sensirionCRC(const uint8_t* data, int len) {
  uint8_t crc = 0xFF;

  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// -------------------- Low-level helpers --------------------
bool writeCommand16(uint8_t addr, uint16_t cmd) {
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  uint8_t err = Wire.endTransmission();

  Serial.print("writeCommand16(0x");
  Serial.print(cmd, HEX);
  Serial.print(") -> I2C status: ");
  Serial.println(err);

  return (err == 0);
}

int readBytes(uint8_t addr, uint8_t* buf, int count) {
  int received = Wire.requestFrom((int)addr, count);

  Serial.print("Requested ");
  Serial.print(count);
  Serial.print(" byte(s), received ");
  Serial.println(received);

  int i = 0;
  while (Wire.available() && i < count) {
    buf[i++] = Wire.read();
  }
  return i;
}

void printBufferHex(const uint8_t* buf, int len) {
  Serial.print("Raw bytes: ");
  for (int i = 0; i < len; i++) {
    Serial.print("0x");
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

// -------------------- Sensor commands --------------------
// These are the standard commands commonly used with Sensirion liquid flow sensors.
// Your custom driver may already use these same values.
const uint16_t CMD_START_WATER          = 0x3608;
const uint16_t CMD_STOP_MEASUREMENT     = 0x3FF9;
const uint16_t CMD_SOFT_RESET           = 0x0006;
const uint16_t CMD_READ_PRODUCT_ID      = 0x367C;  // followed by 0xE102 in some APIs; this may or may not work directly depending on family

// -------------------- Debug functions --------------------
void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h = help");
  Serial.println("  s = scan I2C bus");
  Serial.println("  w = start water measurement");
  Serial.println("  t = stop measurement");
  Serial.println("  r = read raw 9-byte measurement frame");
  Serial.println("  x = soft reset sensor");
  Serial.println("  i = try read product ID");
  Serial.println();
}

void scanI2C() {
  Serial.println("Scanning I2C bus...");

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print("Found device at 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total I2C devices found: ");
    Serial.println(found);
  }
}

void stopMeasurement() {
  Serial.println("Stopping measurement...");
  writeCommand16(SENSOR_ADDR, CMD_STOP_MEASUREMENT);
  delay(50);
}

void startWaterMeasurement() {
  Serial.println("Starting water measurement...");
  bool ok = writeCommand16(SENSOR_ADDR, CMD_START_WATER);
  delay(100);

  if (ok) {
    Serial.println("Start command sent.");
  } else {
    Serial.println("Start command failed.");
  }
}

void softResetSensor() {
  Serial.println("Soft resetting sensor...");
  writeCommand16(SENSOR_ADDR, CMD_SOFT_RESET);
  delay(30);
}

void readMeasurementRaw() {
  Serial.println("Reading raw measurement frame...");

  uint8_t buf[9] = {0};
  int n = readBytes(SENSOR_ADDR, buf, 9);

  if (n <= 0) {
    Serial.println("No data received.");
    return;
  }

  printBufferHex(buf, n);

  if (n < 9) {
    Serial.println("Not enough data for full frame.");
    return;
  }

  uint8_t crcFlow = sensirionCRC(&buf[0], 2);
  uint8_t crcTemp = sensirionCRC(&buf[3], 2);
  uint8_t crcFlag = sensirionCRC(&buf[6], 2);

  Serial.print("Flow bytes: 0x");
  if (buf[0] < 0x10) Serial.print("0");
  Serial.print(buf[0], HEX);
  Serial.print(" 0x");
  if (buf[1] < 0x10) Serial.print("0");
  Serial.print(buf[1], HEX);
  Serial.print(" | CRC rx=0x");
  if (buf[2] < 0x10) Serial.print("0");
  Serial.print(buf[2], HEX);
  Serial.print(" calc=0x");
  if (crcFlow < 0x10) Serial.print("0");
  Serial.println(crcFlow, HEX);

  Serial.print("Temp bytes: 0x");
  if (buf[3] < 0x10) Serial.print("0");
  Serial.print(buf[3], HEX);
  Serial.print(" 0x");
  if (buf[4] < 0x10) Serial.print("0");
  Serial.print(buf[4], HEX);
  Serial.print(" | CRC rx=0x");
  if (buf[5] < 0x10) Serial.print("0");
  Serial.print(buf[5], HEX);
  Serial.print(" calc=0x");
  if (crcTemp < 0x10) Serial.print("0");
  Serial.println(crcTemp, HEX);

  Serial.print("Flag bytes: 0x");
  if (buf[6] < 0x10) Serial.print("0");
  Serial.print(buf[6], HEX);
  Serial.print(" 0x");
  if (buf[7] < 0x10) Serial.print("0");
  Serial.print(buf[7], HEX);
  Serial.print(" | CRC rx=0x");
  if (buf[8] < 0x10) Serial.print("0");
  Serial.print(buf[8], HEX);
  Serial.print(" calc=0x");
  if (crcFlag < 0x10) Serial.print("0");
  Serial.println(crcFlag, HEX);

  uint16_t rawFlow  = ((uint16_t)buf[0] << 8) | buf[1];
  uint16_t rawTemp  = ((uint16_t)buf[3] << 8) | buf[4];
  uint16_t rawFlags = ((uint16_t)buf[6] << 8) | buf[7];

  Serial.print("rawFlow: ");
  Serial.println(rawFlow);

  Serial.print("rawTemp: ");
  Serial.println(rawTemp);

  Serial.print("rawFlags: 0x");
  Serial.println(rawFlags, HEX);
}

void tryReadProductId() {
  Serial.println("Trying to read product ID...");

  // Some Sensirion parts use a two-word command sequence here.
  // This may fail if the command differs for your family.
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x36);
  Wire.write(0x7C);
  Wire.write(0xE1);
  Wire.write(0x02);
  uint8_t err = Wire.endTransmission();

  Serial.print("Product ID command I2C status: ");
  Serial.println(err);

  delay(10);

  uint8_t buf[18] = {0};
  int n = readBytes(SENSOR_ADDR, buf, 18);

  if (n <= 0) {
    Serial.println("No product ID data received.");
    return;
  }

  printBufferHex(buf, n);
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  Serial.println();
  Serial.println("SLF3S-4000B raw I2C debug");
  Serial.print("SDA = "); Serial.println(SDA_PIN);
  Serial.print("SCL = "); Serial.println(SCL_PIN);
  Serial.print("I2C freq = "); Serial.println(I2C_FREQ);
  Serial.print("Sensor addr = 0x"); Serial.println(SENSOR_ADDR, HEX);

  printHelp();
  scanI2C();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    // clear any extra newline chars
    while (Serial.available()) {
      Serial.read();
    }

    switch (c) {
      case 'h':
      case 'H':
        printHelp();
        break;

      case 's':
      case 'S':
        scanI2C();
        break;

      case 'w':
      case 'W':
        startWaterMeasurement();
        break;

      case 't':
      case 'T':
        stopMeasurement();
        break;

      case 'r':
      case 'R':
        readMeasurementRaw();
        break;

      case 'x':
      case 'X':
        softResetSensor();
        break;

      case 'i':
      case 'I':
        tryReadProductId();
        break;

      default:
        Serial.print("Unknown command: ");
        Serial.println(c);
        printHelp();
        break;
    }
  }
}