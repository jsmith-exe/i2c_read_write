#include <Arduino.h>
#include <math.h>

// -------------------- PWM Config --------------------

#define PWM_PIN 18
#define MIN_WATER_PIN 8
#define MAX_WATER_PIN 9
#define OVERFLOW_WATER_PIN 10
#define SOL_TANK_PIN 14

const uint32_t PWM_FREQ = 20000;     // 20 kHz
const uint8_t  PWM_RES_BITS = 8;     // 8-bit => duty 0..255

uint32_t maxDuty = 0;

// Set true if your driver is inverted:
// 0% command -> full on
// 100% command -> full off
bool PWM_INVERT = true;

bool MIN_WATER_STATE = false;
bool MAX_WATER_STATE = false;
bool OVERFLOW_WATER_STATE = false;
uint8_t WATER_STATE = 0;
uint8_t PREV_WATER_STATE = 0;
bool INITIAL_RUN = true;

String inputLine;

// -------------------- PWM Helper --------------------

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

  Serial.print("Commanded PWM: ");
  Serial.print(percent, 2);
  Serial.print("% | Applied PWM: ");
  Serial.print(appliedPercent, 2);
  Serial.print("% | Duty: ");
  Serial.print(duty);
  Serial.print("/");
  Serial.println(maxDuty);
}

// -------------------- Serial Handling --------------------

void handleLine(const String& lineIn)
{
  String line = lineIn;
  line.trim();

  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("help")) {
    Serial.println("Commands:");
    Serial.println("  pwm <0..100>    set PWM percentage");
    Serial.println("  invert on       enable PWM inversion");
    Serial.println("  invert off      disable PWM inversion");
    Serial.println("  help            show commands");
    return;
  }

  if (line.equalsIgnoreCase("invert on")) {
    PWM_INVERT = true;
    Serial.println("PWM inversion enabled");
    return;
  }

  if (line.equalsIgnoreCase("invert off")) {
    PWM_INVERT = false;
    Serial.println("PWM inversion disabled");
    return;
  }

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
      setPWMDutyPercent(pwm);
    } else {
      Serial.println("Invalid PWM value. Use: pwm <0 to 100>");
    }

    return;
  }

  Serial.println("Unknown command. Type: help");
}

void checkSerial()
{
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

void updateWaterStates()
{
  MIN_WATER_STATE = !digitalRead(MIN_WATER_PIN);
  MAX_WATER_STATE = !digitalRead(MAX_WATER_PIN);
  OVERFLOW_WATER_STATE = !digitalRead(OVERFLOW_WATER_PIN);


  WATER_STATE = ( (MIN_WATER_STATE) | (MAX_WATER_STATE << 1) | (OVERFLOW_WATER_STATE << 2) );
}

void printWaterState()
{
  if (OVERFLOW_WATER_STATE)
  {
    Serial.println("Water at OVERFLOW level");
  }
  else if (MAX_WATER_STATE)
  {
    Serial.println("Water at MAX level");
  }
  else if (MIN_WATER_STATE)
  {
    Serial.println("Water at MIN level");
  }
}

void waterPrintingLogic()
{
  if (INITIAL_RUN)
  {
    printWaterState();
  }

  else if (WATER_STATE != PREV_WATER_STATE)
  {
    printWaterState();
  }

}

void waterDetectProgram()
{
  updateWaterStates();
  waterPrintingLogic();

  // Update prev states
  PREV_WATER_STATE = WATER_STATE;

}

// -------------------- Setup --------------------

void setup()
{
  Serial.begin(115200);
  delay(500);

  maxDuty = (1UL << PWM_RES_BITS) - 1;

  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);

  pinMode(MIN_WATER_PIN, INPUT);
  pinMode(MAX_WATER_PIN, INPUT);
  pinMode(OVERFLOW_WATER_PIN, INPUT);
  pinMode(SOL_TANK_PIN, OUTPUT);

  Serial.println();
  Serial.println("Pure PWM Test");
  Serial.println("--------------------");

  if (ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES_BITS)) {
    Serial.println("PWM attached OK");
    setPWMDutyPercent(0.0f);
  } else {
    Serial.println("PWM attach failed");
  }

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  pwm 0");
  Serial.println("  pwm 25");
  Serial.println("  pwm 50");
  Serial.println("  pwm 75");
  Serial.println("  pwm 100");
  Serial.println("  invert on");
  Serial.println("  invert off");
}

// -------------------- Loop --------------------

void loop()
{
  checkSerial();
  waterDetectProgram();

  if (OVERFLOW_WATER_STATE)
  {
    setPWMDutyPercent(0.0f);
    digitalWrite(SOL_TANK_PIN, LOW);

  }
  else if (!WATER_STATE)
  {
    digitalWrite(SOL_TANK_PIN, HIGH);
    setPWMDutyPercent(100);
  }
  INITIAL_RUN = false;
}