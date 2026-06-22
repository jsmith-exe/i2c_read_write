#include "Water_Level_Detection.h"



TANK::TANK()
    : LOW_WATER_STATE(false),
    MAX_WATER_STATE(false),
    OVERFLOW_WATER_STATE(false),
    WATER_STATE(0),
    PREV_WATER_STATE(0),
    INITIAL_RUN(true)
{
}

void TANK::begin()
{
    pinMode(MIN_WATER_PIN, INPUT);
    pinMode(MAX_WATER_PIN, INPUT);
    pinMode(OVERFLOW_WATER_PIN, INPUT);
}

void TANK::updateWaterStates()
{
  // Update prev states
  PREV_WATER_STATE = WATER_STATE;

  LOW_WATER_STATE = !digitalRead(LOW_WATER_PIN);
  MAX_WATER_STATE = !digitalRead(MAX_WATER_PIN);
  OVERFLOW_WATER_STATE = !digitalRead(OVERFLOW_WATER_PIN);


  WATER_STATE = ( (LOW_WATER_STATE) | (MAX_WATER_STATE << 1) | (OVERFLOW_WATER_STATE << 2) );
}

void TANK::printWaterState()
{
  if (OVERFLOW_WATER_STATE)
  {
    Serial.println("Water at OVERFLOW level");
  }
  else if (MAX_WATER_STATE)
  {
    Serial.println("Water at MAX level");
  }
  else if (LOW_WATER_STATE)
  {
    Serial.println("Water at MIN level");
  }
}

void TANK::waterPrintingLogic()
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

void TANK::waterDetectProgram()
{
  updateWaterStates();
  waterPrintingLogic();

  INITIAL_RUN = false;
}