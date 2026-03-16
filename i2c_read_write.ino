#include <Arduino.h>
// ESP32-S3 PWM example
// Arduino IDE

const int pwmPin = 18;             // Change if needed
const uint32_t pwmFreq = 20000;   // 20 kHz
const uint8_t pwmResolution = 8;  // 8-bit => 0..255

uint32_t maxDuty = 0;
String inputBuffer = "";

void setPWMDutyPercent(float percent)
{
  percent = constrain(percent, 0.0, 100.0);

  uint32_t duty = (uint32_t)((percent / 100.0) * maxDuty);

  if (!ledcWrite(pwmPin, duty)) {
    Serial.println("Failed to update PWM duty");
    return;
  }

  Serial.print("Duty cycle set to: ");
  Serial.print(percent, 1);
  Serial.print("%  (duty=");
  Serial.print(duty);
  Serial.println(")");
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  maxDuty = (1UL << pwmResolution) - 1;

  // New ESP32 Arduino core 3.x style
  if (!ledcAttach(pwmPin, pwmFreq, pwmResolution)) {
    Serial.println("Failed to attach PWM to pin");
    while (true) {
      delay(1000);
    }
  }

  setPWMDutyPercent(0);

  Serial.println("ESP32-S3 PWM controller ready");
  Serial.println("Type a number from 0 to 100, then press Enter");
}

void loop()
{
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        float value = inputBuffer.toFloat();

        // Reject non-numeric garbage like "abc"
        bool looksNumeric = false;
        for (size_t i = 0; i < inputBuffer.length(); i++) {
          char ch = inputBuffer[i];
          if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-' || ch == '+') {
            looksNumeric = true;
            break;
          }
        }

        if (looksNumeric && value >= 0.0 && value <= 100.0) {
          setPWMDutyPercent(100-value);
        } else {
          Serial.println("Invalid input. Enter a number from 0 to 100.");
        }

        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}