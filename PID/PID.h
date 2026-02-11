#pragma once

#include <Arduino.h>   // micros()
#include <math.h>      // fabsf, isfinite
#include <utility>     // std::pair

class PID {
public:
  PID(float Kp,
      float Ki,
      float Kd,
      float integral_reset = -1.0f,   // <0 disables (like None)
      float max_value = INFINITY);

  // Returns {output, latency_ms}
  std::pair<float, float> update(float error);

  void reset();
  void setGains(float Kp, float Ki, float Kd);

  float getIntegral() const { return integral_; }
  float getPrevError() const { return prev_error_; }

private:
  float Kp_, Ki_, Kd_;
  float integral_;
  float prev_error_;
  float prev_value_;

  uint32_t last_us_;

  float integral_reset_;
  float max_value_;

  static float clampf(float x, float lo, float hi);
};
