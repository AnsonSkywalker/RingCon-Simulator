// AIGC note: motion - synthetic "squat twist" motion curve generator.
// Two shapes:
//  - arm(): single smoothstep S-pulse whose integral equals the requested
//    angle (legacy one-way "turn left 90", kept for the Arduino CLI).
//  - armTwist(): the calibrated Ring-Con gesture (HANDOFF §〇.6, 2026-09-06
//    on-ring recordings): outbound ~100 deg in ~360 ms (peak ~417 dps),
//    50 ms hold, return stroke crossing center with an 8 deg overshoot, then
//    settling back to 0. Net angle ≈ 0, cycle ≈ 0.97 s; matches the measured
//    peak 390-455 dps / cycle 1.0-1.15 s / overshoot 5-9 deg.
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
  void armTwist();
  void stop() { active_ = false; phase_ = kIdle; }
  bool active() const { return active_; }
  float progress() const;
  float lastOmegaDps() const { return w_dps_; }

  // Advance the simulation by dt_ms (call with 5 ms, three times per 0x30
  // report, to keep the 3-frame batching semantics) and fill the report state.
  void tick(uint16_t dt_ms, ReportState& st);

 private:
  enum Phase : uint8_t { kIdle, kSingle, kOut, kHold, kBack, kSettle };
  void startSeg(Phase p, float from_deg, float to_deg, uint32_t dur_ms);
  float noise();
  Phase phase_ = kIdle;
  bool active_ = false;
  uint32_t t_ms_ = 0;        // time inside the current segment
  uint32_t seg_total_ms_ = 0;
  float seg_from_ = 0.f, seg_to_ = 0.f;  // segment endpoints, degrees
  float w_dps_ = 0.f;
  uint32_t rng_ = 0xA5A5A5A5;
};

}  // namespace joycon
