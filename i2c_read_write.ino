#include <Arduino.h>

// -------------------- Solenoid Pins --------------------

#define SOLENOID_1_PIN 14
#define SOLENOID_2_PIN 15

String inputLine;

// -------------------- Helpers --------------------

void setSolenoid(uint8_t pin, bool state)
{
  digitalWrite(pin, state ? HIGH : LOW);

  Serial.print("Pin ");
  Serial.print(pin);
  Serial.print(" set ");
  Serial.println(state ? "HIGH" : "LOW");
}

void printHelp()
{
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  s1 on       pin 14 HIGH");
  Serial.println("  s1 off      pin 14 LOW");
  Serial.println("  s2 on       pin 15 HIGH");
  Serial.println("  s2 off      pin 15 LOW");
  Serial.println("  both on     pins 14 and 15 HIGH");
  Serial.println("  both off    pins 14 and 15 LOW");
  Serial.println("  status      print current pin states");
  Serial.println("  help        show commands");
  Serial.println();
}

// -------------------- Serial Handling --------------------

void handleLine(const String& lineIn)
{
  String line = lineIn;
  line.trim();
  line.toLowerCase();

  if (line.length() == 0) return;

  if (line == "s1 on") {
    setSolenoid(SOLENOID_1_PIN, true);
  }
  else if (line == "s1 off") {
    setSolenoid(SOLENOID_1_PIN, false);
  }
  else if (line == "s2 on") {
    setSolenoid(SOLENOID_2_PIN, true);
  }
  else if (line == "s2 off") {
    setSolenoid(SOLENOID_2_PIN, false);
  }
  else if (line == "both on") {
    setSolenoid(SOLENOID_1_PIN, true);
    setSolenoid(SOLENOID_2_PIN, true);
  }
  else if (line == "both off") {
    setSolenoid(SOLENOID_1_PIN, false);
    setSolenoid(SOLENOID_2_PIN, false);
  }
  else if (line == "status") {
    Serial.print("Pin 14: ");
    Serial.println(digitalRead(SOLENOID_1_PIN) ? "HIGH" : "LOW");

    Serial.print("Pin 15: ");
    Serial.println(digitalRead(SOLENOID_2_PIN) ? "HIGH" : "LOW");
  }
  else if (line == "help") {
    printHelp();
  }
  else {
    Serial.println("Unknown command. Type: help");
  }
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

// -------------------- Setup --------------------

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(SOLENOID_1_PIN, OUTPUT);
  pinMode(SOLENOID_2_PIN, OUTPUT);

  digitalWrite(SOLENOID_1_PIN, LOW);
  digitalWrite(SOLENOID_2_PIN, LOW);

  Serial.println();
  Serial.println("Solenoid GPIO Test");
  Serial.println("--------------------");
  Serial.println("Pins initialised LOW");
  printHelp();
}

// -------------------- Loop --------------------

void loop()
{
  checkSerial();
}