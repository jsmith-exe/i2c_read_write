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

// I2C address
#define SLF3S4000B_ADDR 0x08

bool PRINT_DATA = true;
bool PLOT_MODE  = false;

// ------------ Device ------------

SLF3S4000B flowSensor(SLF3S4000B_ADDR);

// ------------ Live flag ------------

bool LIVE_FLOW = false;

// ------------ Setup ------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  Serial.println("Starting SLF3S-4000B flow sensor test...");

  LIVE_FLOW = flowSensor.startWater();

  if (LIVE_FLOW) {
    Serial.println("SLF3S-4000B started successfully in water mode.");
  } else {
    Serial.println("SLF3S-4000B startup FAILED.");
  }

  Serial.println("Setup complete.");
  delay(1000);
}

// ------------ Loop ------------

void loop() {
  static uint8_t slf_fail_count = 0;

  // If startup failed, keep retrying
  if (!LIVE_FLOW) {
    Serial.println("Flow sensor not live. Retrying startWater()...");
    LIVE_FLOW = flowSensor.startWater();
    delay(1000);
    return;
  }

  float flow_ml_min = NAN;
  float temp_C = NAN;
  uint16_t flowFlags = 0;

  float flow_filtered_ml_min = NAN;
  float flow_average_ml_min = NAN;
  float flow_filtered_lpm = NAN;
  float flow_corrected_lpm = NAN;
  float flow_corrected_ml_min = NAN;

  bool okFlow = flowSensor.read(flow_ml_min, temp_C, flowFlags);

  bool okFlowFilt = flowSensor.getFilteredFlow(flow_filtered_ml_min);
  bool okFlowAvg  = flowSensor.getAverageFlow(flow_average_ml_min);
  bool okFlowLpm  = flowSensor.getFilteredFlow_lpm(flow_filtered_lpm);
  bool okCorrLpm  = flowSensor.getCorrectedFlowRate_lpm(flow_corrected_lpm);
  bool okCorrMlpm = flowSensor.getCorrectedFlowRate_mlpm(flow_corrected_ml_min);

  // Failure handling
  if (!okFlow) {
    slf_fail_count++;

    Serial.print("Flow read failed. Fail count: ");
    Serial.println(slf_fail_count);

    if (slf_fail_count >= 3) {
      Serial.println("Flow sensor failed 3 times. Resetting sensor...");

      flowSensor.resetSensor();
      delay(100);

      LIVE_FLOW = flowSensor.startWater();

      if (LIVE_FLOW) {
        Serial.println("Flow sensor restarted successfully.");
      } else {
        Serial.println("Flow sensor restart failed.");
      }

      slf_fail_count = 0;
    }
  } else {
    slf_fail_count = 0;
  }

  // ---------- Output ----------
  if (!PLOT_MODE) {
    if (PRINT_DATA) {
      Serial.print("Flow: ");

      if (okFlow) {
        Serial.print(flow_ml_min, 4);
        Serial.print(" ml/min");

        Serial.print("  Temp: ");
        Serial.print(temp_C, 4);
        Serial.print(" C");

        if (okFlowFilt) {
          Serial.print("  Filtered: ");
          Serial.print(flow_filtered_ml_min, 4);
          Serial.print(" ml/min");
        }

        if (okFlowAvg) {
          Serial.print("  Average: ");
          Serial.print(flow_average_ml_min, 4);
          Serial.print(" ml/min");
        }

        if (okFlowLpm) {
          Serial.print("  Filtered L/min: ");
          Serial.print(flow_filtered_lpm, 6);
        }

        if (okCorrLpm) {
          Serial.print("  Corrected L/min: ");
          Serial.print(flow_corrected_lpm, 6);
        }

        if (okCorrMlpm) {
          Serial.print("  Corrected ml/min: ");
          Serial.print(flow_corrected_ml_min, 4);
        }

        Serial.print("  Flags: ");
        SLF3S4000B::printFlags(Serial, flowFlags);
      } else {
        Serial.print("read fail");
      }

      Serial.println();
    }
  } else {
    // Arduino Serial Plotter-friendly output
    Serial.print("Flow_ml_min:");
    Serial.print(okFlow ? flow_ml_min : 0.0f, 4);

    Serial.print(" Temp_C:");
    Serial.print(okFlow ? temp_C : 0.0f, 4);

    Serial.print(" Filtered_ml_min:");
    Serial.print(okFlowFilt ? flow_filtered_ml_min : 0.0f, 4);

    Serial.print(" Corrected_ml_min:");
    Serial.print(okCorrMlpm ? flow_corrected_ml_min : 0.0f, 4);

    Serial.println();
  }

  delay(100);
}