#pragma once
#include <Arduino.h>
#include <utility>
#include <math.h>

class PIDVelocity {
public:
  PIDVelocity(float Kp, float Ki, float Kd,
              float integral_reset,
              float out_min, float out_max,
              float d_tau);

  // Returns {delta_output, latency_ms}
  std::pair<float, float> update(float setpoint, float measurement);

  void reset();
  void setGains(float Kp, float Ki, float Kd);
  void setDerivativeFilterTau(float tau_sec);

  // Optional helpers for bumpless transfer / debugging
  void setOutputState(float u);
  float getOutputState() const;

private:
  static float clampf(float x, float lo, float hi);

  float Kp_, Ki_, Kd_;

  float integral_;
  float prev_meas_;
  float dMeas_filt_;
  uint32_t last_us_;

  float integral_reset_;
  float out_min_, out_max_;
  float d_tau_;

  // Previous term values for incremental output
  float prev_P_;
  float prev_I_;
  float prev_D_;

  // Internal output state to support saturation/anti-windup
  float output_state_;
};