#include <Arduino.h>
#include <Wire.h>

#include "I2CDevices.h"

#include "TempSensor_MCP9808/MCP9808.h"
#include "TempSensor_MCP9808/MCP9808.cpp"
#include "FlowSensor_SLFS4000B/SLFS4000B.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.cpp"
#include "DAC/DAC.h"
#include "DAC/DAC.cpp"

// ------------ Config ------------

// ESP32 I2C pins
#define SDA_PIN 3
#define SCL_PIN 2

bool PRINT_DATA = true;
bool PLOT_MODE  = false;

const uint32_t I2C_FREQ = 400000;

// I2C addresses
#define MCP9808_A_ADDR        0x18
#define MCP9808_B_ADDR        0x1D
#define SLF3S4000B_ADDR       0x08

// DAC
#define DAC_ADDR 0x0F
static const float   DAC_VREF_VOLTS = 3.28f;
static const uint8_t DAC_RES_BITS   = 16;
static const uint8_t DAC_DEFAULT_CH = 0;

// ------------ Devices ------------

MCP9808     mcpA(MCP9808_A_ADDR);
MCP9808     mcpB(MCP9808_B_ADDR);
SLF3S4000B  flowSensor(SLF3S4000B_ADDR);
DAC         dac(DAC_ADDR);

// ------------ Live flags (only print if true) ------------
bool LIVE_MCP_A = false;
bool LIVE_MCP_B = false;
bool LIVE_FLOW  = false;
bool LIVE_DAC   = false;

// ------------ Serial input handling ------------
String inputLine;

void printHelp() {
  Serial.println("Serial commands:");
  Serial.println("  ENTER on empty line -> toggle TEXT/PLOT");
  Serial.println("  Type voltage then ENTER (e.g. 1.250) -> set DAC output");
  Serial.println();
}

// ------------ Setup ------------

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // --- DAC ---
  // dac.begin(DAC_VREF_VOLTS, DAC_RES_BITS, DAC_DEFAULT_CH);
  // LIVE_DAC = true;                // if you want true detection, we can add an I2C ping
  // dac.setVoltage(0.0f);

  // --- MCP9808 A ---
  // LIVE_MCP_A = mcpA.begin(0x03);
  // if (LIVE_MCP_A) {
  //   Serial.println("MCP9808 'A' started");
  //   mcpA.setLimits(0.0f, 60.0f, 80.0f);
  // } else {
  //   Serial.println("MCP9808 'A' startup FAILED");
  // }

  // // --- MCP9808 B ---
  // LIVE_MCP_B = mcpB.begin(0x03);
  // if (LIVE_MCP_B) {
  //   Serial.println("MCP9808 'B' started");
  //   mcpB.setLimits(0.0f, 60.0f, 80.0f);
  // } else {
  //   Serial.println("MCP9808 'B' startup FAILED");
  // }

  // --- SLF3S-4000B flow sensor ---
  LIVE_FLOW = flowSensor.startWater();
  if (LIVE_FLOW) {
    Serial.println("SLF3S-4000B started (water mode)");
    delay(50);
  } else {
    Serial.println("SLF3S-4000B startup FAILED");
  }

  Serial.println("Setup complete.");
  delay(1000);
  //printHelp();
}

// ------------ Loop ------------

void loop() {
  // Run all I2C commands once
  static bool ranOnce = false;
  if (ranOnce) {
    // Stop forever after the first pass
    Serial.println("Loop already ran once. Halting.");
    while (true) {
      delay(1000);
    }
  }
  ranOnce = true;

  static uint8_t slf_fail_count = 0;

  float tA = NAN, tB = NAN;
  float tA_filtered = NAN, tA_av = NAN;

  float flow = NAN, flowT = NAN, byte1 = NAN, byte2 = NAN, CRC = NAN;
  float flow_filtered = NAN, flow_av = NAN;
  uint16_t flowFlags = 0;


  // --------- Serial input: line-based ----------
  // while (Serial.available() > 0) {
  //   char c = (char)Serial.read();
  //   if (c == '\r') continue;

  //   if (c == '\n') {
  //     String line = inputLine;
  //     inputLine = "";
  //     line.trim();

  //     if (line.length() == 0) {
  //       PLOT_MODE = !PLOT_MODE;
  //       Serial.print("Mode: ");
  //       Serial.println(PLOT_MODE ? "PLOT" : "TEXT");
  //     } else {
  //       float v = line.toFloat();
  //       if (LIVE_DAC) {
  //         bool ok = dac.setVoltage(v);
  //         if (!ok) Serial.println("DAC setVoltage() failed (I2C?)");
  //         else {
  //           Serial.print("DAC set to ");
  //           Serial.print(v, 4);
  //           Serial.println(" V");
  //         }
  //       } else {
  //         Serial.println("DAC not marked LIVE");
  //       }
  //     }
  //   } else {
  //     inputLine += c;
  //     if (inputLine.length() > 40) inputLine.remove(0, inputLine.length() - 40);
  //   }
  // }

  // ----- Reads (only if live) -----
  bool okA=false, okA_filt=false, okA_av=false;
  if (LIVE_MCP_A) {
    okA      = mcpA.readTemperature(tA);
    okA_filt = mcpA.getFilteredTemp(tA_filtered);
    okA_av   = mcpA.getAverageTemp(tA_av);
  }

  bool okB=false;
  if (LIVE_MCP_B) {
    okB = mcpB.readTemperature(tB);
  }

  bool okFlow=false, okFlow_filt=false, okFlow_av=false;
  if (LIVE_FLOW) {
    okFlow      = flowSensor.read(flow, flowT, flowFlags, byte1, byte2, CRC);
    okFlow_filt = flowSensor.getFilteredFlow(flow_filtered);
    okFlow_av   = flowSensor.getAverageFlow(flow_av);

    if (!okFlow) {
      slf_fail_count++;
      if (slf_fail_count >= 3) {
        flowSensor.resetSensor();
        slf_fail_count = 0;
        // optional: LIVE_FLOW = false;  // hide it if it keeps failing
      }
    } else {
      slf_fail_count = 0;
    }
  }

  // ---------- OUTPUT BLOCK ----------
  if (!PLOT_MODE) {
    if (PRINT_DATA) {
      bool printedAnything = false;

      if (LIVE_MCP_A) {
        Serial.print("A: ");
        if (okA) {
          Serial.print(tA, 4); Serial.print(" °C");
          if (okA_filt) { Serial.print("  F:");  Serial.print(tA_filtered, 4); }
          if (okA_av)   { Serial.print("  Av:"); Serial.print(tA_av, 4); }
        } else {
          Serial.print("read fail");
        }
        Serial.print("   ");
        printedAnything = true;
      }

      if (LIVE_MCP_B) {
        Serial.print("B: ");
        if (okB) { Serial.print(tB, 4); Serial.print(" °C"); }
        else     { Serial.print("read fail"); }
        Serial.print("   ");
        printedAnything = true;
      }

      if (LIVE_FLOW) {
        Serial.print("Flow: ");
        if (okFlow) {
          Serial.print(flow, 4);
          if (okFlow_filt) { Serial.print("  F:");  Serial.print(flow_filtered, 4); }
          if (okFlow_av)   { Serial.print("  Av:"); Serial.print(flow_av, 4); }
          Serial.print(" ml/min  T:");
          Serial.print(flowT, 4);
          Serial.print(" Flags:");
          SLF3S4000B::printFlags(Serial, flowFlags);
          Serial.print("  B1: "); Serial.print(byte1, HEX); Serial.print("  B2: "); Serial.print(byte2, HEX); Serial.print("  CRC: "); Serial.print(CRC, HEX);
        } else {
          Serial.print("read fail");
        }
        Serial.print("   ");
        printedAnything = true;
      }

      if (printedAnything) Serial.println();
    }
  } else {
    // Plot mode: only print live channels to keep plotter clean
    bool first = true;

    if (LIVE_MCP_A) {
      Serial.print("tA:"); Serial.print(okA ? tA : 0.0f, 4);
      first = false;
    }

    if (LIVE_MCP_B) {
      if (!first) Serial.print(" ");
      Serial.print("tB:"); Serial.print(okB ? tB : 0.0f, 4);
      first = false;
    }

    if (LIVE_FLOW) {
      if (!first) Serial.print(" ");
      Serial.print("Flow:");  Serial.print(okFlow ? flow : 0.0f, 4);
      Serial.print(" FlowT:");Serial.print(okFlow ? flowT : 0.0f, 4);
      first = false;
    }

    Serial.println();
  }

  delay(100);
}
