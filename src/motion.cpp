// AIGC note: motion.cpp - smoothstep twist segments (one-way pulse or the
// calibrated two-way Ring-Con gesture) + idle jitter.
#include "motion.h"
#include <math.h>

namespace joycon {

void TwistMotion::startSeg(Phase p, float from_deg, float to_deg, uint32_t dur_ms) {
  phase_ = p;
  seg_from_ = from_deg;
  seg_to_ = to_deg;
  if (dur_ms < 1) dur_ms = 1;
  seg_total_ms_ = dur_ms;
  t_ms_ = 0;
}

void TwistMotion::arm(float angle_deg, uint32_t duration_ms) {
  if (duration_ms < 50) duration_ms = 50;
  startSeg(kSingle, 0.f, angle_deg, duration_ms);
  active_ = true;
}

void TwistMotion::armTwist() {
  startSeg(kOut, 0.f, cfg.out_angle_deg, cfg.out_ms);
  active_ = true;
}

float TwistMotion::noise() {
  // xorshift32, centered; cheap determinism beats float rand on a timer task
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return (float)(int32_t)rng_ / 2147483648.f;  // [-1, 1)
}

float TwistMotion::progress() const {
  uint32_t total = cfg.out_ms + cfg.hold_ms + cfg.back_ms + cfg.settle_ms;
  if (phase_ == kSingle) return seg_total_ms_ ? t_ms_ / (float)seg_total_ms_ : 1.f;
  uint32_t elapsed = t_ms_;
  switch (phase_) {
    case kHold: elapsed += cfg.out_ms; break;
    case kBack: elapsed += cfg.out_ms + cfg.hold_ms; break;
    case kSettle: elapsed += cfg.out_ms + cfg.hold_ms + cfg.back_ms; break;
    default: break;
  }
  return total ? elapsed / (float)total : 1.f;
}

void TwistMotion::tick(uint16_t dt_ms, ReportState& st) {
  float omega = 0.f;
  if (active_) {
    t_ms_ += dt_ms;
    if (t_ms_ >= seg_total_ms_) {  // segment boundary -> next phase
      t_ms_ = seg_total_ms_;
      switch (phase_) {
        case kOut:
          startSeg(kHold, seg_to_, seg_to_, cfg.hold_ms);
          break;
        case kHold:  // return stroke ends past center (measured 5-9 deg over)
          startSeg(kBack, seg_to_, -cfg.overshoot_deg, cfg.back_ms);
          break;
        case kBack:  // pull the overshoot back to net-zero twist
          startSeg(kSettle, seg_to_, 0.f, cfg.settle_ms);
          break;
        case kSingle:
        case kSettle:
          phase_ = kIdle;
          active_ = false;
          break;
        default:
          break;
      }
    }
    if (active_ && phase_ != kHold) {
      float u = t_ms_ / (float)seg_total_ms_;
      // omega = d(theta)/dt; S'(u) = 6u - 6u^2 => zero at both ends (R5-friendly)
      float span_s = (seg_to_ - seg_from_) / (seg_total_ms_ / 1000.f);
      omega = span_s * (6.f * u - 6.f * u * u);
    }
  }
  w_dps_ = omega;

  float n_g = noise() * cfg.gyro_noise_dps;
  for (int i = 0; i < 3; i++) st.gyro_dps[i] = n_g;
  st.gyro_dps[cfg.yaw_axis] += cfg.yaw_sign * omega;
  // secondary-axis bleed: real recordings show x/z coupling p-p ≤ 184 dps on
  // the axis opposite the acc-wobble one
  uint8_t couple_axis = (cfg.yaw_axis + 2) % 3;
  if (cfg.wobble_axis > 0.5f) couple_axis = (cfg.yaw_axis + 1) % 3;
  st.gyro_dps[couple_axis] += cfg.coupling_ratio * omega;

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
