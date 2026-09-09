// AIGC note: joycon_report - input report packer for Switch HID protocol
// Format from dekuNukem/Nintendo_Switch_Reverse_Engineering (bluetooth_hid_notes,
// imu_sensor_notes); re-written, not copied. Report 0x21 (subcmd reply) and
// 0x30 (standard full + IMU). Byte offsets follow joycontrol/report.py findings.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>

namespace joycon {

struct ImuFrame {  // 12 bytes on the wire: acc xyz, gyro xyz, int16 LE
  int16_t acc[3];
  int16_t gyro[3];
};

// Raw 0x30 button bits (report bytes 4/5), per joycontrol ButtonState /
// dekuNukem: byte4 = Y X B A SR SL R ZR, byte5 = Minus Plus RStick LStick
// Home Capture (bit0..5); byte6 (unused here) carries the Pro/L-JC d-pad.
enum JoyConRButtons : uint32_t {
  BTN_Y = 1u << 0, BTN_X = 1u << 1, BTN_B = 1u << 2, BTN_A = 1u << 3,
  BTN_SR = 1u << 4, BTN_SL = 1u << 5, BTN_R = 1u << 6, BTN_ZR = 1u << 7,
  BTN_MINUS = 1u << 8, BTN_PLUS = 1u << 9, BTN_R_STICK = 1u << 10,
  BTN_L_STICK = 1u << 11, BTN_HOME = 1u << 12, BTN_CAPTURE = 1u << 13,
};

struct ReportState {
  uint32_t buttons = 0;         // JoyConRButtons bits
  uint8_t stick_x = 0x80, stick_y = 0x80;  // 0..255, 0x80 = center
  // IMU in physical units; packer converts to raw int16 LE
  float acc_g[3] = {0.f, 0.f, 1.f};   // g
  float gyro_dps[3] = {0.f, 0.f, 0.f};
  // Ring-Con strain: when on, overwrites the 3rd IMU frame's accel-Y slot
  // (wire bytes 40-41 of the 0xA1-prefixed frame; parse.ts offset 39 without
  // report id). Values only flow while the host enabled ExtDev polling.
  bool strain_on = false;
  uint16_t strain_raw = 0;
};

// Serializes into a 0xA1-prefixed HID input report buffer.
// report 0x30: 50 bytes total, IMU spans bytes [14..49] (an earlier 49-byte
// total was an off-by-one that truncated the last IMU byte); 0x21: 51 bytes.
class ReportPacker {
 public:
  void begin(uint8_t joycon_type);  // 0x02 = Joy-Con R
  // Fills buf with input report. Returns total bytes to notify.
  size_t pack0x30(uint8_t* buf, const ReportState& s, uint8_t timer,
                  bool imu_enabled) const;
  // 0x30 with three independently sampled 5 ms frames (real Joy-Con semantics).
  size_t pack0x30Frames(uint8_t* buf, const ReportState (&st)[3], uint8_t timer,
                        bool imu_enabled) const;
  size_t pack0x21(uint8_t* buf, const ReportState& s, uint8_t timer,
                  uint8_t ack, uint8_t subcmd_id,
                  const std::vector<uint8_t>& reply) const;
  void setGyroScale(float dps_per_lsb);  // 0.06103 default; 0.070 alternative
  float gyroScale() const { return gyro_scale_; }

 private:
  void putCommon(uint8_t* buf, const ReportState& s, uint8_t timer) const;
  uint8_t joycon_type_ = 0x02;
  float gyro_scale_ = 0.06103f;
  float acc_scale_ = 0.000244f;
};

// SPI flash emulation: joycontrol-style defaults (0xFF blank + documented
// factory cal at 0x603D/0x6046 sticks / 0x6020 IMU). No real dump needed.
class SpiFlash {
 public:
  // Reads emulated flash; fills 0xFF for unmapped regions (caller sends it as-is).
  bool read(uint32_t addr, uint8_t* out, uint8_t len) const;
  static constexpr uint32_t kImuCalAddr = 0x6020;
  static constexpr uint32_t kDevTypeAddr = 0x6012;
  static constexpr uint32_t kStickCalAddr = 0x603D;
  static constexpr uint32_t kStickCalRAddr = 0x6046;
  static constexpr uint32_t kColorExistAddr = 0x601B;
  static constexpr uint32_t kColorAddr = 0x6050;
};

}  // namespace joycon
