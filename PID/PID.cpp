#include "PID.h"

PID::PID(float Kp,
         float Ki,
         float Kd,
         float integral_reset,
         float max_value)
  : Kp_(Kp),
    Ki_(Ki),
    Kd_(Kd),
    integral_(0.0f),
    prev_error_(0.0f),
    prev_value_(0.0f),
    last_us_(micros()),
    integral_reset_(integral_reset),
    max_value_(max_value) {}

std::pair<float, float> PID::update(float error) {
  const uint32_t tic_us = micros();

  const uint32_t now_us = micros();
  const uint32_t dt_us_u = (uint32_t)(now_us - last_us_); // handles wrap via unsigned math
  const float dt = (float)dt_us_u * 1e-6f;

  // Integral reset band (enabled if integral_reset_ >= 0)
  if (integral_reset_ >= 0.0f) {
    if (fabsf(error) <= integral_reset_) {
      integral_ = 0.0f;
    }
  }

  // Integrate
  integral_ += error * dt;

  // Derivative
  float derivative = 0.0f;
  if (dt > 0.0f) {
    derivative = (error - prev_error_) / dt;
  }

  // PID output
  float output = (Kp_ * error) + (Ki_ * integral_) + (Kd_ * derivative);

  // Clamp (np.clip equivalent)
  if (isfinite(max_value_)) {
    output = clampf(output, -max_value_, max_value_);
  }

  // Update state
  prev_error_ = error;
  last_us_ = micros();

  const uint32_t toc_us = micros();
  const uint32_t lat_us = (uint32_t)(toc_us - tic_us);
  const float latency_ms = (float)lat_us * 1e-3f;

  prev_value_ = output;
  return {output, latency_ms};
}

void PID::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  prev_value_ = 0.0f;
  last_us_ = micros();
}

void PID::setGains(float Kp, float Ki, float Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

float PID::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
