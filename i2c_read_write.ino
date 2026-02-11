#include <Arduino.h>
#include <Wire.h>

#include "I2CDevices.h"

#include "TempSensor_MCP9808/MCP9808.h"
#include "TempSensor_MCP9808/MCP9808.cpp"
#include "FlowSensor_SLFS4000B/SLFS4000B.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.cpp"
#include "ExternalTempSensor_MCP9984T/MCP9984T.h"
#include "ExternalTempSensor_MCP9984T/MCP9984T.cpp"
#include "NovaPressureSensor/NOVA.h"
#include "NovaPressureSensor/NOVA.cpp"
#include "DAC/DAC.h"
#include "DAC/DAC.cpp"

// ------------ Config ------------

// ESP32 I2C pins
#define SDA_PIN 3
#define SCL_PIN 2

bool PRINT_DATA = true;
bool PLOT_MODE  = false;

const uint32_t I2C_FREQ = 200000;

// I2C addresses
#define MCP9808_A_ADDR        0x19
#define MCP9808_B_ADDR        0x1D
#define SLF3S4000B_ADDR       0x08
#define MCP9984T_ADDR         0x4C
#define PRESSURE_SENSOR_ADDR  0x28

// DAC
#define DAC_ADDR 0x0F
static const float   DAC_VREF_VOLTS = 3.28f;
static const uint8_t DAC_RES_BITS   = 16;
static const uint8_t DAC_DEFAULT_CH = 0;

// ------------ Devices ------------

MCP9808     mcpA(MCP9808_A_ADDR);
MCP9808     mcpB(MCP9808_B_ADDR);
SLF3S4000B  flowSensor(SLF3S4000B_ADDR);
MCP9984T    mcp9984t(MCP9984T_ADDR);
uint16_t    mcp9984t_id = 0;
NOVA_PRESSURE pressureSensor(PRESSURE_SENSOR_ADDR);
DAC         dac(DAC_ADDR);

// ------------ Live flags (only print if true) ------------
bool LIVE_MCP_A = false;
bool LIVE_MCP_B = false;
bool LIVE_FLOW  = false;
bool LIVE_BJT   = false;
bool LIVE_PRESS = false; // not used in loop currently
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
  dac.begin(DAC_VREF_VOLTS, DAC_RES_BITS, DAC_DEFAULT_CH);
  LIVE_DAC = true;                // if you want true detection, we can add an I2C ping
  dac.setVoltage(0.0f);

  // --- MCP9808 A ---
  LIVE_MCP_A = mcpA.begin(0x03);
  if (LIVE_MCP_A) {
    Serial.println("MCP9808 'A' started");
    mcpA.setLimits(0.0f, 60.0f, 80.0f);
  } else {
    Serial.println("MCP9808 'A' startup FAILED");
  }

  // --- MCP9808 B ---
  LIVE_MCP_B = mcpB.begin(0x03);
  if (LIVE_MCP_B) {
    Serial.println("MCP9808 'B' started");
    mcpB.setLimits(0.0f, 60.0f, 80.0f);
  } else {
    Serial.println("MCP9808 'B' startup FAILED");
  }

  // --- SLF3S-4000B flow sensor ---
  LIVE_FLOW = flowSensor.startWater();
  if (LIVE_FLOW) {
    Serial.println("SLF3S-4000B started (water mode)");
    delay(50);
  } else {
    Serial.println("SLF3S-4000B startup FAILED");
  }

  // --- MCP9984T external BJT temp sensor ---
  LIVE_BJT =
    (mcp9984t.begin(mcp9984t_id) &&
     mcp9984t.setAlertThermMode(true) &&
     mcp9984t.setAlertLimitsAllBJTs(70.0f) &&
     mcp9984t.setThermHysteresis(5.0f));

  if (LIVE_BJT) {
    Serial.print("MCP9984T started, ID: 0x");
    Serial.println(mcp9984t_id, HEX);
  } else {
    Serial.println("MCP9984T: startup failed!");
  }

  Serial.println("Setup complete.");
  printHelp();
}

// ------------ Loop ------------

void loop() {
  static uint8_t slf_fail_count = 0;

  float tA = NAN, tB = NAN;
  float tA_filtered = NAN, tA_av = NAN;

  float flow = NAN, flowT = NAN;
  float flow_filtered = NAN, flow_av = NAN;
  uint16_t flowFlags = 0;

  float tBJTs[3] = {NAN, NAN, NAN};

  // --------- Serial input: line-based ----------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      String line = inputLine;
      inputLine = "";
      line.trim();

      if (line.length() == 0) {
        PLOT_MODE = !PLOT_MODE;
        Serial.print("Mode: ");
        Serial.println(PLOT_MODE ? "PLOT" : "TEXT");
      } else {
        float v = line.toFloat();
        if (LIVE_DAC) {
          bool ok = dac.setVoltage(v);
          if (!ok) Serial.println("DAC setVoltage() failed (I2C?)");
          else {
            Serial.print("DAC set to ");
            Serial.print(v, 4);
            Serial.println(" V");
          }
        } else {
          Serial.println("DAC not marked LIVE");
        }
      }
    } else {
      inputLine += c;
      if (inputLine.length() > 40) inputLine.remove(0, inputLine.length() - 40);
    }
  }

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
    okFlow      = flowSensor.read(flow, flowT, flowFlags);
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

  bool okBJT1=false, okBJT2=false, okBJT3=false;
  if (LIVE_BJT) {
    okBJT1 = mcp9984t.readExternalTemp(1, tBJTs[0]);
    okBJT2 = mcp9984t.readExternalTemp(2, tBJTs[1]);
    okBJT3 = mcp9984t.readExternalTemp(3, tBJTs[2]);
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
        } else {
          Serial.print("read fail");
        }
        Serial.print("   ");
        printedAnything = true;
      }

      if (LIVE_BJT) {
        Serial.print("BJT: ");
        if (okBJT1) { Serial.print("1="); Serial.print(tBJTs[0], 3); Serial.print(" "); }
        else        { Serial.print("1=fail "); }
        if (okBJT2) { Serial.print("2="); Serial.print(tBJTs[1], 3); Serial.print(" "); }
        else        { Serial.print("2=fail "); }
        if (okBJT3) { Serial.print("3="); Serial.print(tBJTs[2], 3); }
        else        { Serial.print("3=fail"); }
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

    if (LIVE_BJT) {
      if (!first) Serial.print(" ");
      Serial.print("BJT1:"); Serial.print(okBJT1 ? tBJTs[0] : 0.0f, 4);
      Serial.print(" BJT2:");Serial.print(okBJT2 ? tBJTs[1] : 0.0f, 4);
      Serial.print(" BJT3:");Serial.print(okBJT3 ? tBJTs[2] : 0.0f, 4);
      first = false;
    }

    Serial.println();
  }

  delay(100);
}
