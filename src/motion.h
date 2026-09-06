// AIGC note: motion - synthetic "squat twist" motion curve generator.
// Produces a smooth S-curve (smoothstep) yaw pulse whose integral equals the
// requested angle, with a physically-plausible lateral acceleration wobble
// (centrifugal r*omega^2) synced to the turn (research report risk R3).
// Axis/sign are parameters because Joy-Con R IMU axes differ from L (R4).
#pragma once
#include <Arduino.h>
#include "joycon_report.h"

namespace joycon {

class TwistMotion {
 public:
  struct Config {
    uint8_t yaw_axis = 2;    // gyro index carrying yaw (Z for Joy-Con R vertical)
    float yaw_sign = 1.f;    // +1 / -1: flip if the game reads the turn backwards
    float wobble_axis = 0.f; // 0 => use (yaw_axis+1)%3; 1 => use (yaw_axis+2)%3
    float wobble_radius_m = 0.12f;  // ring-con lever arm for centrifugal accel
    float idle_acc_g[3] = {0.f, 0.f, 1.f};  // resting gravity in body frame
    float gyro_noise_dps = 0.25f;   // idle jitter, far below the real ~4.6 dps
    float acc_noise_g = 0.01f;
  };
  Config cfg;

  // Queue one twist. angle 90 deg / 500 ms reproduces "turn left 90".
  void arm(float angle_deg, uint32_t duration_ms);
  void stop() { active_ = false; }
  bool active() const { return active_; }
  float progress() const { return duration_ms_ ? t_ms_ / (float)duration_ms_ : 1.f; }
  float lastOmegaDps() const { return w_dps_; }

  // Advance the simulation by dt_ms (call with 5 ms, three times per 0x30
  // report, to keep the 3-frame batching semantics) and fill the report state.
  void tick(uint16_t dt_ms, ReportState& st);

 private:
  float noise();
  bool active_ = false;
  float angle_deg_ = 90.f;
  uint32_t duration_ms_ = 500;
  uint32_t t_ms_ = 0;
  float w_dps_ = 0.f;
  uint32_t rng_ = 0xA5A5A5A5;
};

}  // namespace joycon
