// AIGC note: motion - synthetic "squat twist" motion curve generator.
// Shapes:
//  - arm(): single smoothstep S-pulse whose integral equals the requested
//    angle (legacy one-way "turn left 90", kept for the Arduino CLI).
//  - armTwist(dir): the calibrated Ring-Con two-way gesture (HANDOFF §〇.6):
//    outbound ~100 deg in ~360 ms (peak ~417 dps), 50 ms hold, return stroke
//    crossing center with an 8 deg overshoot, then settling back to 0. Net
//    angle = 0, cycle ~0.97 s. dir: +1 left / -1 right / 0 = cfg.yaw_sign.
//  - armRot(axis, sign): one-way 90 deg rotation around a body axis. The
//    synthetic controller carries a PERSISTENT orientation (quaternion): the
//    rotation really accumulates, gravity in body frame follows, and the
//    game-side IMU integration stays physically consistent.
//  - armReset(): smooth shortest-arc return to the resting orientation
//    (Ring-Con placed flat, button face up = acc (0,0,1) g).
// Axis/sign are parameters because Joy-Con R IMU axes differ from L (R4).
#pragma once
#include <stdint.h>
#include "joycon_report.h"

namespace joycon {

class TwistMotion {
 public:
  struct Config {
    uint8_t yaw_axis = 1;    // dominant gyro axis = y in Joy-Con R body frame
                             // (ring-grip posture calibration, HANDOFF §〇.6)
    float yaw_sign = 1.f;    // +1 / -1: flip if the game reads the turn backwards
    float out_angle_deg = 100.f;  // outbound twist angle (measured 100-125)
    uint32_t out_ms = 360;        // peak = 1.5*100/0.36 ≈ 417 dps (measured 390-455)
    uint32_t hold_ms = 50;        // pause at full twist before returning
    uint32_t back_ms = 430;       // return stroke, ends at -overshoot_deg
    float overshoot_deg = 8.f;    // swing past center (measured 5-9)
    uint32_t settle_ms = 130;     // pull the overshoot back to 0
    float coupling_ratio = 0.15f; // gyro bleed onto the secondary axis
                                  // (measured x/z coupling p-p ≤ 184 dps)
    float wobble_axis = 0.f; // 0 => acc wobble on (yaw_axis+1)%3; 1 => (yaw_axis+2)%3
    float wobble_radius_m = 0.12f;  // ring-con lever arm for centrifugal accel
    float idle_acc_g[3] = {0.f, 0.f, 1.f};  // resting gravity in body frame
    float gyro_noise_dps = 10.f;  // uniform half-span, sigma ≈ 5.8 dps
                                  // (measured idle 1σ ≈ 6 dps incl. hand tremor)
    float acc_noise_g = 0.01f;
  };
  Config cfg;

  // Queue one one-way twist pulse. angle 90 deg / 500 ms = "turn left 90".
  void arm(float angle_deg, uint32_t duration_ms);
  // Queue one calibrated two-way twist (Ring-Con gesture, cfg parameters).
  // dir: +1 = left, -1 = right, 0 = use cfg.yaw_sign.
  void armTwist(int dir = 0);
  // Queue a one-way 90 deg rotation around body axis 0/1/2; the persistent
  // orientation (and thus the resting acc the game sees) really changes.
  void armRot(uint8_t axis, float sign, uint32_t duration_ms = 400);
  // Smooth shortest-arc return to the resting orientation (face-up flat).
  void armReset(uint32_t duration_ms = 500);
  void stop() { active_ = false; phase_ = kIdle; }
  bool active() const { return active_; }
  float progress() const;
  float lastOmegaDps() const { return w_dps_; }
  // Persistent virtual orientation quaternion (w, x, y, z).
  const float* orient() const { return q_; }

  // Advance the simulation by dt_ms (call with 5 ms, three times per 0x30
  // report, to keep the 3-frame batching semantics) and fill the report state.
  void tick(uint16_t dt_ms, ReportState& st);

 private:
  enum Phase : uint8_t { kIdle, kSingle, kOut, kHold, kBack, kSettle, kRot };
  void startSeg(Phase p, float from_deg, float to_deg, uint32_t dur_ms);
  void beginRot(const float axis[3], float deg, uint32_t dur_ms);
  float noise();
  Phase phase_ = kIdle;
  bool active_ = false;
  uint32_t t_ms_ = 0;        // time inside the current segment
  uint32_t seg_total_ms_ = 0;
  float seg_from_ = 0.f, seg_to_ = 0.f;  // twist/single segment endpoints, deg
  float w_dps_ = 0.f;
  uint32_t rng_ = 0xA5A5A5A5;
  float twist_sign_ = 1.f;   // effective sign of the running twist/single
  // Persistent virtual orientation, quaternion (w, x, y, z), identity =
  // Ring-Con resting pose (button face up).
  float q_[4] = {1.f, 0.f, 0.f, 0.f};
  // Active kRot segment: body-frame axis/angle from q_start_, and the acc
  // vector at segment start (gravity rotated into the start orientation).
  float q_start_[4] = {1.f, 0.f, 0.f, 0.f};
  float rot_axis_[3] = {1.f, 0.f, 0.f};
  float rot_deg_ = 0.f;
  float acc_start_[3] = {0.f, 0.f, 1.f};
};

}  // namespace joycon
