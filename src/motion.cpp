// AIGC note: motion.cpp - smoothstep twist segments (calibrated two-way
// Ring-Con gesture / legacy one-way pulse / persistent-orientation body-axis
// rotations) + idle jitter. The virtual orientation is a quaternion so the
// game's IMU integration stays physically consistent across rot/reset.
#include "motion.h"
#include <math.h>

namespace joycon {
namespace {

const float kPi = 3.14159265f;

void quatMul(const float a[4], const float b[4], float out[4]) {
  out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void quatConj(const float a[4], float out[4]) {
  out[0] = a[0];
  out[1] = -a[1];
  out[2] = -a[2];
  out[3] = -a[3];
}

void quatNormalize(float q[4]) {
  float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n < 1e-9f) return;
  for (int i = 0; i < 4; i++) q[i] /= n;
}

void quatAxisAngle(const float axis[3], float deg, float out[4]) {
  float h = deg * 0.5f * kPi / 180.f;
  float s = sinf(h);
  out[0] = cosf(h);
  out[1] = axis[0] * s;
  out[2] = axis[1] * s;
  out[3] = axis[2] * s;
}

// out = q * v * q^-1  (rotate vector v by unit quaternion q)
void quatRotVec(const float q[4], const float v[3], float out[3]) {
  float tx = 2.f * (q[2]*v[2] - q[3]*v[1]);
  float ty = 2.f * (q[3]*v[0] - q[1]*v[2]);
  float tz = 2.f * (q[1]*v[1] - q[2]*v[0]);
  out[0] = v[0] + q[0]*tx + (q[2]*tz - q[3]*ty);
  out[1] = v[1] + q[0]*ty + (q[3]*tx - q[1]*tz);
  out[2] = v[2] + q[0]*tz + (q[1]*ty - q[2]*tx);
}

}  // namespace

void TwistMotion::startSeg(Phase p, float from_deg, float to_deg, uint32_t dur_ms) {
  phase_ = p;
  seg_from_ = from_deg;
  seg_to_ = to_deg;
  if (dur_ms < 1) dur_ms = 1;
  seg_total_ms_ = dur_ms;
  t_ms_ = 0;
}

void TwistMotion::beginRot(const float axis[3], float deg, uint32_t dur_ms) {
  for (int i = 0; i < 3; i++) rot_axis_[i] = axis[i];
  rot_deg_ = deg;
  float c[4];
  quatConj(q_, c);
  quatRotVec(c, cfg.idle_acc_g, acc_start_);  // gravity at the start pose
  for (int i = 0; i < 4; i++) q_start_[i] = q_[i];
  startSeg(kRot, 0.f, 0.f, dur_ms);
  active_ = true;
}

void TwistMotion::arm(float angle_deg, uint32_t duration_ms) {
  if (duration_ms < 50) duration_ms = 50;
  twist_sign_ = cfg.yaw_sign;
  startSeg(kSingle, 0.f, angle_deg, duration_ms);
  active_ = true;
}

void TwistMotion::armTwist(int dir) {
  twist_sign_ = (dir == 0) ? cfg.yaw_sign : (dir < 0 ? -1.f : 1.f);
  startSeg(kOut, 0.f, cfg.out_angle_deg, cfg.out_ms);
  active_ = true;
}

void TwistMotion::armRot(uint8_t axis, float sign, uint32_t duration_ms) {
  if (axis > 2) return;
  float a[3] = {0.f, 0.f, 0.f};
  a[axis] = 1.f;
  beginRot(a, 90.f * sign, duration_ms);
}

void TwistMotion::armReset(uint32_t duration_ms) {
  float c[4];
  quatConj(q_, c);
  if (c[0] < 0.f)  // shortest arc back to identity
    for (int i = 0; i < 4; i++) c[i] = -c[i];
  float xyz = sqrtf(c[1]*c[1] + c[2]*c[2] + c[3]*c[3]);
  float theta = 2.f * atan2f(xyz, c[0]) * 180.f / kPi;
  if (theta < 2.f || xyz < 1e-5f) {  // close enough: snap, no motion
    q_[0] = 1.f;
    q_[1] = q_[2] = q_[3] = 0.f;
    return;
  }
  float axis[3] = {c[1] / xyz, c[2] / xyz, c[3] / xyz};
  beginRot(axis, theta, duration_ms);
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
    default: break;  // kOut / kRot: first segment
  }
  return total ? elapsed / (float)total : 1.f;
}

void TwistMotion::tick(uint16_t dt_ms, ReportState& st) {
  // base gravity in body frame for the persistent orientation
  float acc0[3];
  {
    float c[4];
    quatConj(q_, c);
    quatRotVec(c, cfg.idle_acc_g, acc0);
  }
  float omega[3] = {0.f, 0.f, 0.f};  // body angular velocity this tick (deg/s)
  float omega_yaw = 0.f;             // scalar along cfg.yaw_axis (twist/single)
  bool rot_mode = false;

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
        case kRot:  // orientation was integrated live; land exactly on target
          {
            float p[4];
            quatAxisAngle(rot_axis_, rot_deg_, p);
            quatMul(q_start_, p, q_);
            quatNormalize(q_);
          }
          phase_ = kIdle;
          active_ = false;
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
      float shape = 6.f * u - 6.f * u * u;  // S'(u), zero at both ends
      if (phase_ == kRot) {
        rot_mode = true;
        float s = u * u * (3.f - 2.f * u);
        float p[4];
        quatAxisAngle(rot_axis_, s * rot_deg_, p);
        quatMul(q_start_, p, q_);  // live orientation update
        quatNormalize(q_);
        float dsdt = shape / (seg_total_ms_ / 1000.f);
        for (int i = 0; i < 3; i++) omega[i] = rot_axis_[i] * rot_deg_ * dsdt;
        float pc[4];
        quatConj(p, pc);
        quatRotVec(pc, acc_start_, acc0);  // gravity follows the rotation
      } else {
        // omega = d(theta)/dt; S'(u) = 6u - 6u^2 => zero at both ends (R5)
        float span_s = (seg_to_ - seg_from_) / (seg_total_ms_ / 1000.f);
        omega_yaw = span_s * shape;
      }
    }
  }
  w_dps_ = rot_mode ? sqrtf(omega[0]*omega[0] + omega[1]*omega[1] +
                            omega[2]*omega[2])
                    : omega_yaw;

  float n_g = noise() * cfg.gyro_noise_dps;
  for (int i = 0; i < 3; i++) st.gyro_dps[i] = n_g;
  if (rot_mode) {
    for (int i = 0; i < 3; i++) st.gyro_dps[i] += omega[i];
  } else {
    st.gyro_dps[cfg.yaw_axis] += twist_sign_ * omega_yaw;
    // secondary-axis bleed: real recordings show x/z coupling p-p ≤ 184 dps on
    // the axis opposite the acc-wobble one
    uint8_t couple_axis = (cfg.yaw_axis + 2) % 3;
    if (cfg.wobble_axis > 0.5f) couple_axis = (cfg.yaw_axis + 1) % 3;
    st.gyro_dps[couple_axis] += cfg.coupling_ratio * omega_yaw;
  }

  if (!rot_mode) {
    // centrifugal wobble on one horizontal axis; yaw around gravity leaves the
    // gravity vector itself untouched, so idle acc stays near the resting pose
    float a_wob = twist_sign_ *
                  (omega_yaw * 0.01745329f) * (omega_yaw * 0.01745329f) *
                  cfg.wobble_radius_m / 9.81f;
    uint8_t wob_axis = (cfg.yaw_axis + 1) % 3;
    if (cfg.wobble_axis > 0.5f) wob_axis = (cfg.yaw_axis + 2) % 3;
    float n_a = noise() * cfg.acc_noise_g;
    for (int i = 0; i < 3; i++) st.acc_g[i] = acc0[i] + n_a;
    st.acc_g[wob_axis] += a_wob;
  } else {
    float n_a = noise() * cfg.acc_noise_g;
    for (int i = 0; i < 3; i++) st.acc_g[i] = acc0[i] + n_a;
  }
}

}  // namespace joycon
