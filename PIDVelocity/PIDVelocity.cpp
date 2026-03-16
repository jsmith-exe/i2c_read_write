#include "PIDVelocity.h"

PIDVelocity::PIDVelocity(float Kp, float Ki, float Kd,
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
    d_tau_(d_tau),
    prev_P_(0.0f),
    prev_I_(0.0f),
    prev_D_(0.0f),
    output_state_(0.0f) {}

std::pair<float, float> PIDVelocity::update(float setpoint, float measurement) {
  const uint32_t tic_us = micros();

  const uint32_t now_us = micros();
  const uint32_t dt_us_u = (uint32_t)(now_us - last_us_);   // wrap-safe
  const float dt = (float)dt_us_u * 1e-6f;

  // Protect against zero / tiny dt
  if (dt <= 1e-9f) {
    const uint32_t toc_us = micros();
    const float latency_ms = (float)((uint32_t)(toc_us - tic_us)) * 1e-3f;
    last_us_ = now_us;
    prev_meas_ = measurement;
    return {0.0f, latency_ms};
  }

  const float error = setpoint - measurement;

  // Optional integral reset band
  if (integral_reset_ >= 0.0f && fabsf(error) <= integral_reset_) {
    integral_ = 0.0f;
  }

  // Derivative on measurement
  const float dMeas = (measurement - prev_meas_) / dt;

  // Low-pass filter derivative
  if (d_tau_ > 0.0f) {
    const float a = d_tau_ / (d_tau_ + dt);
    dMeas_filt_ = a * dMeas_filt_ + (1.0f - a) * dMeas;
  } else {
    dMeas_filt_ = dMeas;
  }

  // Tentative integral update
  const float integral_candidate = integral_ + error * dt;

  // Current candidate PID terms
  const float P = Kp_ * error;
  const float I = Ki_ * integral_candidate;
  const float D = -Kd_ * dMeas_filt_;

  // Incremental change in output
  const float delta_unclamped =
      (P - prev_P_) +
      (I - prev_I_) +
      (D - prev_D_);

  // Predict next absolute output state
  const float predicted_unclamped = output_state_ + delta_unclamped;
  float predicted_clamped = predicted_unclamped;

  if (isfinite(out_min_) || isfinite(out_max_)) {
    predicted_clamped = clampf(predicted_unclamped, out_min_, out_max_);
  }

  // Actual delta after clamp
  const float delta_output = predicted_clamped - output_state_;

  // Saturation test
  const bool saturated = (predicted_clamped != predicted_unclamped);

  // Anti-windup:
  // only accept the integral update if we are not saturating further
  if (!saturated) {
    integral_ = integral_candidate;
    prev_I_ = I;
  } else {
    // Keep previous I term if saturated
    // Recompute actual I term from stored integral_ for consistency
    prev_I_ = Ki_ * integral_;
  }

  // Always update P and D history because those are based on current sample
  prev_P_ = P;
  prev_D_ = D;

  // Update internal output state to the clamped value
  output_state_ = predicted_clamped;

  // Update timing / measurement state
  prev_meas_ = measurement;
  last_us_ = now_us;

  const uint32_t toc_us = micros();
  const float latency_ms = (float)((uint32_t)(toc_us - tic_us)) * 1e-3f;

  return {delta_output, latency_ms};
}

void PIDVelocity::reset() {
  integral_ = 0.0f;
  prev_meas_ = 0.0f;
  dMeas_filt_ = 0.0f;
  prev_P_ = 0.0f;
  prev_I_ = 0.0f;
  prev_D_ = 0.0f;
  output_state_ = 0.0f;
  last_us_ = micros();
}

void PIDVelocity::setGains(float Kp, float Ki, float Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PIDVelocity::setDerivativeFilterTau(float tau_sec) {
  d_tau_ = tau_sec;
}

void PIDVelocity::setOutputState(float u) {
  output_state_ = u;
}

float PIDVelocity::getOutputState() const {
  return output_state_;
}

float PIDVelocity::clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}