#pragma once

#include <Arduino.h>

class TANK
{
public:
    TANK();
    void begin();
    void updateWaterStates();
    void printWaterStates();
    void waterPrintingLogic();
    void waterDetectProgram();

    bool LOW_WATER_STATE;
    bool MAX_WATER_STATE;
    bool OVERFLOW_WATER_STATE;

    uint8_t WATER_STATE;
    uint8_t PREV_WATER_STATE;

private:
    static constexpr uint8_t LOW_WATER_PIN = 8;
    static constexpr uint8_t MAX_WATER_PIN = 9;
    static constexpr uint8_t OVERFLOW_WATER_PIN = 10;

    bool INITIAL_RUN;
};

