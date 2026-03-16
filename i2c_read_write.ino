#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "PID/PID.h"
#include "PID/PID.cpp"
#include "I2CDevices.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.h"
#include "FlowSensor_SLFS4000B/SLFS4000B.cpp"

// -------------------- HW Config --------------------

// ESP32 I2C pins
#define SDA_PIN 3
#define SCL_PIN 2
const uint32_t I2C_FREQ = 400000;

// SLF3S-4000B
#define SLF3S4000B_ADDR 0x08

// PWM output
#define PWM_PIN 18
const uint32_t PWM_FREQ = 20000;     // 20 kHz
const uint8_t  PWM_RES_BITS = 8;     // 8-bit => duty 0..255

// Reference used by PID output scaling
static const float VREF = 3.28f;

// -------------------- Devices --------------------

SLF3S4000B flowSensor(SLF3S4000B_ADDR);

// -------------------- PID Config --------------------

float kp = 0.005f;
float ki = 0.0f;
float kd = 0.0f;
float d_tau = 0.05f;

// PID output still treated as "virtual voltage" from 0 to VREF
PID flowPid(kp, ki, kd, -1.0f, 0.0f, VREF, d_tau);

// Setpoint in ml/min
float targetFlow = 400.0f;

// -------------------- Runtime --------------------

unsigned long lastMicros_;
String inputLine;

bool LIVE_FLOW = false;
bool LIVE_PWM  = false;

uint32_t maxDuty = 0;

// Set true if your driver is inverted:
// 0% command -> full on
// 100% command -> full off
bool PWM_INVERT = true;

// Manual PWM override mode
bool MANUAL_PWM_MODE = false;
float manualPwmPercent = 0.0f;

// -------------------- PWM Helpers --------------------

static inline float voltsToDutyPercent(float v) {
  if (v <= 0.0f) return 0.0f;
  if (v >= VREF) return 100.0f;
  return (v / VREF) * 100.0f;
}

void setPWMDutyPercent(float percent)
{
  percent = constrain(percent, 0.0f, 100.0f);

  float appliedPercent = percent;
  if (PWM_INVERT) {
    appliedPercent = 100.0f - percent;
  }

  uint32_t duty = (uint32_t)lroundf((appliedPercent / 100.0f) * maxDuty);

  if (!ledcWrite(PWM_PIN, duty)) {
    Serial.println("Failed to update PWM duty");
    return;
  }
}

// -------------------- Serial Handling --------------------
// Supports:
//   pwm <0..100>      manual PWM mode
//   auto              return to PID control
//   kp,ki,kd
//   kp,ki,kd,dtau
//   single number -> targetFlow (ml/min)

void handleLine(const String& lineIn) {
  String line = lineIn;
  line.trim();
  if (line.length() == 0) return;

  // ---------- manual PWM command ----------
  // Example: pwm 35
  if (line.startsWith("pwm ") || line.startsWith("PWM ")) {
    String valueStr = line.substring(4);
    valueStr.trim();

    float pwm = valueStr.toFloat();

    bool looksNumeric = false;
    for (size_t i = 0; i < valueStr.length(); i++) {
      char ch = valueStr[i];
      if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-' || ch == '+') {
        looksNumeric = true;
        break;
      }
    }

    if (looksNumeric && pwm >= 0.0f && pwm <= 100.0f) {
      MANUAL_PWM_MODE = true;
      manualPwmPercent = pwm;

      if (LIVE_PWM) {
        setPWMDutyPercent(manualPwmPercent);
      }

      Serial.print("Manual PWM mode enabled: ");
      Serial.print(manualPwmPercent, 2);
      Serial.println("%");
    } else {
      Serial.println("Invalid PWM value. Use: pwm <0 to 100>");
    }
    return;
  }

  // ---------- back to auto PID mode ----------
  if (line.equalsIgnoreCase("auto")) {
    MANUAL_PWM_MODE = false;
    flowPid.reset();
    Serial.println("Returned to AUTO PID mode");
    return;
  }

  // ---------- PID gains ----------
  float newKp = 0, newKi = 0, newKd = 0, newDtau = 0;
  int n = sscanf(line.c_str(), "%f,%f,%f,%f", &newKp, &newKi, &newKd, &newDtau);

  if (n == 3 || n == 4) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
    if (n == 4) {
      d_tau = newDtau;
    }

    flowPid.setGains(kp, ki, kd);
    if (n == 4) {
      flowPid.setDerivativeFilterTau(d_tau);
    }
    flowPid.reset();

    Serial.print("Updated PID: kp="); Serial.print(kp);
    Serial.print(" ki="); Serial.print(ki);
    Serial.print(" kd="); Serial.print(kd);
    if (n == 4) {
      Serial.print(" dtau="); Serial.print(d_tau);
    }
    Serial.println();
    return;
  }

  // ---------- setpoint ----------
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
  Serial.println("  pwm <0..100>    manual PWM mode");
  Serial.println("  auto            return to PID control");
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
      if (inputLine.length() > 80) {
        inputLine.remove(0, inputLine.length() - 80);
      }
    }
  }
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ);

  // PWM init
  maxDuty = (1UL << PWM_RES_BITS) - 1;

  if (ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES_BITS)) {
    LIVE_PWM = true;
    setPWMDutyPercent(0.0f);
    Serial.println("PWM attached OK");
  } else {
    LIVE_PWM = false;
    Serial.println("PWM attach failed");
  }

  // Flow sensor init
  LIVE_FLOW = flowSensor.startWater();

  if (LIVE_FLOW) {
    Serial.println("Flow Address Ack");
  } else {
    Serial.println("Flow Address Failed");
  }

  flowPid.reset();
  lastMicros_ = micros();

  Serial.println();
  Serial.println("Ready.");
  Serial.println("Commands:");
  Serial.println("  pwm 30          manual PWM at 30%");
  Serial.println("  auto            return to PID control");
  Serial.println("  400             set targetFlow to 400 ml/min");
  Serial.println("  0.005,0,0       set kp,ki,kd");
  Serial.println("  0.005,0,0,0.05  set kp,ki,kd,dtau");
}

// -------------------- Loop --------------------

void loop() {
  checkSerial();

  float dt = (micros() - lastMicros_) * 1e-6f;
  lastMicros_ = micros();
  if (dt <= 0.0f) dt = 1e-3f;

  float corrected_flow_mlpm = 0.0f;
  float raw_flow = 0.0f;
  float temp = 0.0f;
  uint16_t flag = 0;

  flowSensor.getCorrectedFlowRate_mlpm(corrected_flow_mlpm);
  flowSensor.read(raw_flow, temp, flag);

  float flow_meas = corrected_flow_mlpm;

  // ---- PID update ----
  auto pidOut = flowPid.update(targetFlow, flow_meas);
  float vcmd = pidOut.first;

  if (targetFlow <= 0.0f) {
    vcmd = 0.0f;
  }

  // Convert voltage command to PWM percentage
  float pwmPercent = voltsToDutyPercent(vcmd);

  // Manual override
  if (MANUAL_PWM_MODE) {
    pwmPercent = manualPwmPercent;
  }

  // Apply PWM
  if (LIVE_PWM) {
    setPWMDutyPercent(pwmPercent);
  }

  // ---- Print ----
  Serial.print("target:");      Serial.print(targetFlow, 3);      Serial.print(" ");
  Serial.print("meas:");        Serial.print(flow_meas, 3);       Serial.print(" ");
  Serial.print("vcmdV:");       Serial.print(vcmd, 6);            Serial.print(" ");
  Serial.print("pwmPercent:");  Serial.print(pwmPercent, 2);      Serial.print(" ");
  Serial.print("mode:");

  if (MANUAL_PWM_MODE) {
    Serial.print("MANUAL");
  } else {
    Serial.print("AUTO");
  }

  Serial.println();

  delay(50);
}