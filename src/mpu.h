// AIGC note: mpu - real GY-521 (MPU6050) passthrough mapped into the Joy-Con
// frame. Probes WHO_AM_I at boot; if the sensor is absent the caller keeps
// using the synthetic motion generator (no reflash needed when wiring later).
#pragma once
#include <Arduino.h>
#include "joycon_report.h"

namespace joycon {

class MpuSource {
 public:
  struct Config {
    // Joy-Con axis j <- MPU axis map_[j] with sign sign_[j] (identity default)
    int8_t gyro_map[3] = {0, 1, 2};
    int8_t gyro_sign[3] = {1, 1, 1};
    int8_t acc_map[3] = {0, 1, 2};
    int8_t acc_sign[3] = {1, 1, 1};
  };
  Config cfg;

  bool begin(int sda, int scl);  // probe, set ranges/DLPF, zero-bias calibrate
  bool detected() const { return ok_; }
  void setEnabled(bool v) { enabled_ = v; }
  bool enabled() const { return enabled_; }

  // One 5ms frame sample into st (physical g / dps, bias-corrected).
  void sample(ReportState& st);

 private:
  bool ok_ = false;
  bool enabled_ = true;
  float gb_[3] = {0, 0, 0};  // gyro zero-rate bias (raw LSB)
  float ab_[3] = {0, 0, 0};  // acc bias (raw LSB)
};

}  // namespace joycon
