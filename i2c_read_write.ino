#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "PID/PID.h"
#include "PID/PID.cpp"
#include "I2CDevices.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.cpp"
#include "DAC/DAC.h"
#include "DAC/DAC.cpp"

// -------------------- HW Config --------------------

// ESP32 I2C pins (change if needed)
#define SDA_PIN 3
#define SCL_PIN 2

const uint32_t I2C_FREQ = 200000;

// SLF3S-4000B
#define SLF3S4000B_ADDR 0x08

// DAC
#define DAC_ADDR 0x0F
static const float   VREF            = 3.28f;   // DAC reference voltage (and your PID output limit)
static const uint8_t DAC_RES_BITS    = 16;
static const uint8_t DAC_DEFAULT_CH  = 0;

// -------------------- Devices --------------------

SLF3S4000B flowSensor(SLF3S4000B_ADDR);
DAC        dac(DAC_ADDR);

// -------------------- PID Config --------------------

float kp = 0.005f;
float ki = 0.0f;
float kd = 0.0f;
float d_tau = 0.05f;

// PID output limits: [0 .. VREF] volts
PID flowPid(kp, ki, kd, -1.0f, 0.0f, VREF, d_tau);

// Setpoint in ml/min
float targetFlow = 400.0f;

// Deadband for pump drive (volts). Anything below -> 0V output.
float vDead = 0.25f;

// -------------------- Runtime --------------------

unsigned long lastMicros_;
String inputLine;

bool LIVE_FLOW = false;
bool LIVE_DAC  = false;

static uint8_t slf_fail_count = 0;

// -------------------- 16-bit DAC helpers --------------------

static inline uint16_t voltsToDacCode(float v) {
  if (v <= 0.0f) return 0;
  if (v >= VREF) return 65535;
  // Round-to-nearest code
  return (uint16_t)lroundf((v / VREF) * 65535.0f);
}

static inline float dacCodeToVolts(uint16_t code) {
  return ((float)code * VREF) / 65535.0f;
}

// -------------------- Serial Handling --------------------
// Keep your existing PID tuning format:
//   kp,ki,kd
//   kp,ki,kd,dtau
// Also supports: single number -> targetFlow (ml/min)

void handleLine(const String& lineIn) {
  String line = lineIn;
  line.trim();
  if (line.length() == 0) return;

  // Try parse as PID gains first
  float newKp=0, newKi=0, newKd=0, newDtau=0;
  int n = sscanf(line.c_str(), "%f,%f,%f,%f", &newKp, &newKi, &newKd, &newDtau);

  if (n == 3 || n == 4) {
    flowPid.setGains(newKp, newKi, newKd);
    if (n == 4) flowPid.setDerivativeFilterTau(newDtau);
    flowPid.reset();

    Serial.print("Updated PID: kp="); Serial.print(newKp);
    Serial.print(" ki="); Serial.print(newKi);
    Serial.print(" kd="); Serial.print(newKd);
    if (n == 4) { Serial.print(" dtau="); Serial.print(newDtau); }
    Serial.println();
    return;
  }

  // Otherwise try single float -> setpoint
  char* endPtr = nullptr;
  float sp = strtof(line.c_str(), &endPtr);
  if (endPtr != line.c_str()) {
    targetFlow = sp;
    Serial.print("Updated targetFlow = ");
    Serial.print(targetFlow, 3);
    Serial.println(" ml/min");
    return;
  }

  Serial.println("Commands:");
  Serial.println("  kp,ki,kd        (e.g. 0.10,0.20,0.00)");
  Serial.println("  kp,ki,kd,dtau   (e.g. 0.10,0.20,0.00,0.05)");
  Serial.println("  <number>        set targetFlow in ml/min (e.g. 400)");
}

void checkSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (inputLine.length() > 0) {
        handleLine(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
      if (inputLine.length() > 80) inputLine.remove(0, inputLine.length() - 80);
    }
  }
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // DAC init
  dac.begin(VREF, DAC_RES_BITS, DAC_DEFAULT_CH);
  LIVE_DAC = true; // if you want, we can add an I2C ping to confirm
  dac.setVoltage(0.0f);

  // Flow sensor init
  LIVE_FLOW = flowSensor.startWater();
  if (LIVE_FLOW) {
    Serial.println("SLF3S-4000B started (water mode)");
    delay(50);
  } else {
    Serial.println("SLF3S-4000B startup FAILED");
  }

  flowPid.reset();
  lastMicros_ = micros();

  Serial.println();
  Serial.println("Ready.");
  Serial.println("Tune: kp,ki,kd  or  kp,ki,kd,dtau");
  Serial.println("Setpoint: <number> (ml/min)");
}

// -------------------- Loop --------------------

void loop() {
  checkSerial();

  float dt = (micros() - lastMicros_) * 1e-6f;
  lastMicros_ = micros();
  if (dt <= 0.0f) dt = 1e-3f;

  // ---- Read real flow sensor ----
  float flow = NAN, flowT = NAN;
  float flow_filtered = NAN, flow_av = NAN;
  uint16_t flowFlags = 0;

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
      }
    } else {
      slf_fail_count = 0;
    }
  }

  // Use sensor as measurement (ml/min)
  float flow_meas = flow_av;

  // ---- PID update ----
  // PID returns vcmd in volts [0..VREF]
  auto [vcmd, _] = flowPid.update(targetFlow, flow_meas);


  if (targetFlow <= 0.0f) {
    vcmd = 0.0f;
  }

  // ---- 16-bit code (for printing true DAC resolution) ----
  uint16_t dacCode = voltsToDacCode(vcmd);
  float    vQuant  = dacCodeToVolts(dacCode); // voltage representable by that exact code

  // ---- Apply to DAC ----
  if (LIVE_DAC) {
    bool ok = dac.setVoltage(vcmd);
    if (!ok) {
      Serial.println("DAC setVoltage() failed (I2C?)");
    }
  }

  // ---- Print ----
  Serial.print("target:"); Serial.print(targetFlow, 3); Serial.print(" ");
  Serial.print("meas:");   Serial.print(flow_meas, 3);  Serial.print(" ");

  // show volts with higher precision (float is ~6-7 sig figs on ESP32)
  Serial.print("vcmdV:");  Serial.print(vcmd, 6);       Serial.print(" ");

  // show exact DAC code + quantized voltage (true 16-bit info)
  // Serial.print("dac:");    Serial.print(dacCode);
  // Serial.print(" (0x");    Serial.print(dacCode, HEX);  Serial.print(") ");
  // Serial.print("dacV:");   Serial.print(vQuant, 6);

  if (okFlow) {
    Serial.print(" flowT:"); Serial.print(flowT, 3);
  } else {
    Serial.print(" (flow read fail)");
  }

  Serial.println();

  delay(50);
}