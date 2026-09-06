// AIGC note: mpu.cpp - MPU6050 (i2cdevlib API) sampling + zero-bias calibration.
// Ranges chosen to mirror the Joy-Con IMU: gyro +/-2000 dps (16.4 LSB/dps),
// accel +/-8 g (4096 LSB/g), DLPF 98 Hz to keep the 5 ms frame stream clean.
#include "mpu.h"
#include <MPU6050.h>
#include <Wire.h>

namespace joycon {
namespace {
MPU6050 mpu;
constexpr float kLsbPerDps = 16.4f;   // FS +/-2000 dps
constexpr float kLsbPerG = 4096.f;    // FS +/-8 g
}  // namespace

bool MpuSource::begin(int sda, int scl) {
  Wire.begin(sda, scl);
  Wire.setClock(400000);
  mpu.initialize();
  ok_ = mpu.testConnection();
  if (!ok_) {
    Serial.printf("[mpu] not detected (who_am_i=0x%02x, wired to SDA=%d SCL=%d?)\n",
                  mpu.getDeviceID(), sda, scl);
    return false;
  }
  mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_8);
  mpu.setDLPFMode(MPU6050_IMU::MPU6050_DLPF_BW_98);

  // Zero-rate calibration: board must be still during boot
  for (int i = 0; i < 100; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float g[3] = {(float)gx, (float)gy, (float)gz};
    float a[3] = {(float)ax, (float)ay, (float)az};
    for (int j = 0; j < 3; j++) {
      gb_[j] += g[j] / 100.f;
      ab_[j] += a[j] / 100.f;
    }
    delay(2);
  }
  Serial.printf("[mpu] MPU6050 ready, bias g=(%.1f %.1f %.1f) a=(%.1f %.1f %.1f)\n",
                gb_[0], gb_[1], gb_[2], ab_[0], ab_[1], ab_[2]);
  return true;
}

void MpuSource::sample(ReportState& st) {
  if (!ok_) return;
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float g[3] = {(float)gx, (float)gy, (float)gz};
  float a[3] = {(float)ax, (float)ay, (float)az};
  for (int j = 0; j < 3; j++) {
    int m = cfg.gyro_map[j];
    st.gyro_dps[j] = cfg.gyro_sign[j] * (g[m] - gb_[m]) / kLsbPerDps;
    m = cfg.acc_map[j];
    st.acc_g[j] = cfg.acc_sign[j] * (a[m] - ab_[m]) / kLsbPerG;
  }
}

}  // namespace joycon
