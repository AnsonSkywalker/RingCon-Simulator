// AIGC note: motion.cpp - smoothstep twist curve + idle jitter.
#include "motion.h"
#include <math.h>

namespace joycon {

void TwistMotion::arm(float angle_deg, uint32_t duration_ms) {
  if (duration_ms < 50) duration_ms = 50;
  angle_deg_ = angle_deg;
  duration_ms_ = duration_ms;
  t_ms_ = 0;
  w_dps_ = 0;
  active_ = true;
}

float TwistMotion::noise() {
  // xorshift32, centered; cheap determinism beats float rand on a timer task
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return (float)(int32_t)rng_ / 2147483648.f;  // [-1, 1)
}

void TwistMotion::tick(uint16_t dt_ms, ReportState& st) {
  float omega = 0.f;
  if (active_) {
    t_ms_ += dt_ms;
    if (t_ms_ >= duration_ms_) {
      t_ms_ = duration_ms_;
      active_ = false;
    }
    float u = t_ms_ / (float)duration_ms_;
    float su = u * u * (3.f - 2.f * u);          // smoothstep position
    (void)su;
    // omega = d(theta)/dt; S'(u) = 6u - 6u^2 => zero at both ends (R5-friendly)
    omega = angle_deg_ * (6.f * u - 6.f * u * u) / (duration_ms_ / 1000.f);
  }
  w_dps_ = omega;

  float n_g = noise() * cfg.gyro_noise_dps;
  for (int i = 0; i < 3; i++) st.gyro_dps[i] = n_g;
  st.gyro_dps[cfg.yaw_axis] += cfg.yaw_sign * omega;

  // centrifugal wobble on one horizontal axis; yaw around gravity leaves the
  // gravity vector itself untouched, so idle acc stays near the resting pose
  float a_wob = cfg.yaw_sign *
                (omega * 0.01745329f) * (omega * 0.01745329f) *
                cfg.wobble_radius_m / 9.81f;
  uint8_t wob_axis = (cfg.yaw_axis + 1) % 3;
  if (cfg.wobble_axis > 0.5f) wob_axis = (cfg.yaw_axis + 2) % 3;
  float n_a = noise() * cfg.acc_noise_g;
  for (int i = 0; i < 3; i++) st.acc_g[i] = cfg.idle_acc_g[i] + n_a;
  st.acc_g[wob_axis] += a_wob;
}

}  // namespace joycon
