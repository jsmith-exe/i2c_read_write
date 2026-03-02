#include <Arduino.h>

#include "DAC/DAC.h"
#include "DAC/DAC.cpp"

// ------------ Config ------------

// ESP32 I2C pins
#define SDA_PIN 3
#define SCL_PIN 2

const uint32_t I2C_FREQ = 400000;

// DAC
#define DAC_ADDR 0x0F
static const float   DAC_VREF_VOLTS = 3.28f;
static const uint8_t DAC_RES_BITS   = 16;
static const uint8_t DAC_DEFAULT_CH = 0;   // 0=A, 1=B, 2=both

DAC dac(DAC_ADDR);

// State
static String inputLine;
bool ack;

// ------------ Setup ------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // begin() = store config + probe address + (optional) force known output
  ack = dac.begin(DAC_VREF_VOLTS, DAC_RES_BITS, DAC_DEFAULT_CH, true);

  if (ack) {
    Serial.println("Address Ack");
  } else {
    Serial.println("Address Failed");
  }
  delay(1000);
  
}

// ------------ Loop ------------

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      inputLine.trim();


      // Parse voltage
      float v = inputLine.toFloat();
      inputLine = "";

      if (!dac.setVoltage(v)) {
        Serial.println("DAC setVoltage() failed (I2C error).");
        return;
      }

        Serial.print("DAC set to ");
        Serial.print(v, 4);
        Serial.println(" V");

    } else {
      inputLine += c;
      if (inputLine.length() > 40) inputLine.remove(0, inputLine.length() - 40);
    }
  }
}