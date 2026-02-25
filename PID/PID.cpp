#include "PID.h"

PID::PID(float Kp, float Ki, float Kd,
         float integral_reset,
         float out_min, float out_max,
         float d_tau)
  : Kp_(Kp), Ki_(Ki), Kd_(Kd),
    integral_(0.0f),
    prev_meas_(0.0f),
    dMeas_filt_(0.0f),
    last_us_(micros()),
    integral_reset_(integral_reset),
    out_min_(out_min), out_max_(out_max),
    d_tau_(d_tau) {}

std::pair<float, float> PID::update(float setpoint, float measurement) {
  const uint32_t tic_us = micros();

  const uint32_t now_us = micros();
  const uint32_t dt_us_u = (uint32_t)(now_us - last_us_); // wrap-safe unsigned math
  const float dt = (float)dt_us_u * 1e-6f;

  // If dt is tiny/zero, just output P term safely
  if (dt <= 1e-9f) {
    const float error = setpoint - measurement;
    float out = Kp_ * error;
    if (isfinite(out_min_) || isfinite(out_max_)) out = clampf(out, out_min_, out_max_);

    const uint32_t toc_us = micros();
    const float latency_ms = (float)((uint32_t)(toc_us - tic_us)) * 1e-3f;
    last_us_ = now_us;
    prev_meas_ = measurement;
    return {out, latency_ms};
  }

  const float error = setpoint - measurement;

  // Optional integral reset band
  if (integral_reset_ >= 0.0f && fabsf(error) <= integral_reset_) {
    integral_ = 0.0f;
  }

  // Derivative on measurement: d(meas)/dt
  float dMeas = (measurement - prev_meas_) / dt;

  // Low-pass filter derivative (recommended for noisy sensors)
  if (d_tau_ > 0.0f) {
    const float a = d_tau_ / (d_tau_ + dt);              // 0..1
    dMeas_filt_ = a * dMeas_filt_ + (1.0f - a) * dMeas;  // filtered derivative
  } else {
    dMeas_filt_ = dMeas;
  }

  // Integrate (tentatively)
  const float integral_candidate = integral_ + error * dt;

  // Compute unclamped output using derivative on measurement (note the minus sign)
  const float unclamped =
      (Kp_ * error) +
      (Ki_ * integral_candidate) -
      (Kd_ * dMeas_filt_);

  // Clamp to limits (DAC range)
  float output = unclamped;
  if (isfinite(out_min_) || isfinite(out_max_)) {
    output = clampf(output, out_min_, out_max_);
  }

  // Simple anti-windup: only accept integral update if not saturating further
  // (prevents integrator from "pushing" into the clamp)
  const bool saturated = (output != unclamped);
  if (!saturated) {
    integral_ = integral_candidate;
  }
  // If you want slightly softer anti-windup, you can allow integration when error would
  // move output back toward range — but this simple version is usually good enough.

  // Update state
  prev_meas_ = measurement;
  last_us_ = now_us;

  const uint32_t toc_us = micros();
  const float latency_ms = (float)((uint32_t)(toc_us - tic_us)) * 1e-3f;

  return {output, latency_ms};
}

void PID::reset() {
  integral_ = 0.0f;
  prev_meas_ = 0.0f;
  dMeas_filt_ = 0.0f;
  last_us_ = micros();
}

void PID::setGains(float Kp, float Ki, float Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::setDerivativeFilterTau(float tau_sec) {
  d_tau_ = tau_sec;
}

float PID::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
