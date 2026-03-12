#include <Arduino.h>
#include <Wire.h>

#include "I2CDevices.h"

#include "FlowSensor_SLFS4000B/SLFS4000B.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.cpp"

// ------------ Config ------------

// ESP32 I2C pins
#define SDA_PIN 3
#define SCL_PIN 2

const uint32_t I2C_FREQ = 400000;

// I2C addresses
#define SLF3S4000B_ADDR 0x08

bool ack;

SLF3S4000B flowSensor(SLF3S4000B_ADDR);

// ------------ Setup ------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // --- SLF3S-4000B flow sensor ---
  ack = flowSensor.startWater();
  if (ack) {
    Serial.println("Address Ack");
  } else {
    Serial.println("Address Failed");
  }

  Serial.println("Press ENTER to take a measurement");
}

// ------------ Loop ------------

void loop() {

  // Wait for user to press ENTER
  if (Serial.available()) {

    // Clear the buffer
    while (Serial.available()) Serial.read();

    float flow = NAN, flowT = NAN, byte1 = NAN, byte2 = NAN, CRC = NAN;
    uint16_t flowFlags = 0;

    flowSensor.read(flow, flowT, flowFlags, byte1, byte2, CRC);

    Serial.print("Flow: "); Serial.print(flow, 4);
    Serial.print("  B1: "); Serial.print(byte1, HEX);
    Serial.print("  B2: "); Serial.print(byte2, HEX);
    Serial.print("  CRC: "); Serial.print(CRC, HEX);
    Serial.println();

    Serial.println("Press ENTER to run again");
  }

}