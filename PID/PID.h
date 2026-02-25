#pragma once

#include <Arduino.h>
#include <math.h>
#include <utility>

class PID {
public:
  // out_min/out_max: clamp output to your DAC range (e.g., 0..3.28)
  // integral_reset: <0 disables; otherwise integral reset when |error| <= integral_reset
  // d_tau: derivative filter time-constant (seconds). 0 disables filtering.
  PID(float Kp, float Ki, float Kd,
      float integral_reset = -1.0f,
      float out_min = -INFINITY,
      float out_max = INFINITY,
      float d_tau = 0.05f);

  // ✅ Derivative on measurement (no derivative kick from setpoint steps)
  // Returns {output, latency_ms}
  std::pair<float, float> update(float setpoint, float measurement);

  void reset();
  void setGains(float Kp, float Ki, float Kd);
  void setDerivativeFilterTau(float tau_sec);

  float getIntegral() const { return integral_; }

private:
  float Kp_, Ki_, Kd_;

  float integral_;
  float prev_meas_;
  float dMeas_filt_;

  uint32_t last_us_;

  float integral_reset_;
  float out_min_;
  float out_max_;
  float d_tau_;

  static float clampf(float x, float lo, float hi);
};
