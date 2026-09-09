// AIGC note: joycon_report.cpp - input report serialization + SPI flash emulation.
// Protocol facts from dekuNukem reverse-engineering docs (public), re-implemented.
#include "joycon_report.h"
#include <string.h>
#include <math.h>

namespace joycon {

// Colors MUST match the claimed controller type: Switch 2 rejects a Joy-Con R
// wearing Pro-gray (UARTSwitchCon's 0x232323 passes on Switch 1 but loops
// forever on Switch 2, 2026-09-08). Values = official Neon Red (#FF4554)
// body/grips, dark buttons; 0x605C-0x605D unknown = 0.
static const uint8_t kColors[14] = {0xFF, 0x45, 0x54, 0x25, 0x26, 0x26,
                                    0xFF, 0x45, 0x54, 0xFF, 0x45, 0x54,
                                    0x00, 0x00};
// 0x603D L-stick(9) + 0x6046 R-stick(9) factory calibration
static const uint8_t kSticks18[18] = {0x00, 0x07, 0x70, 0x00, 0x08, 0x80,
                                      0x00, 0x07, 0x70, 0x00, 0x08, 0x80,
                                      0x00, 0x07, 0x70, 0x00, 0x07, 0x70};

static void put16le(uint8_t* p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void putF(float v, float scale, uint8_t* p) {
  float raw = v / scale;
  if (raw > 32767.f) raw = 32767.f;
  if (raw < -32768.f) raw = -32768.f;
  put16le(p, (int16_t)lroundf(raw));
}

void ReportPacker::begin(uint8_t jc_type) { joycon_type_ = jc_type; }

void ReportPacker::setGyroScale(float s) { gyro_scale_ = s; }

void ReportPacker::putCommon(uint8_t* buf, const ReportState& s, uint8_t timer) const {
  buf[0] = 0xA1;      // HID data input
  buf[1] = 0x30;      // report id (caller may overwrite to 0x21)
  buf[2] = timer;     // free-running ~5ms tick
  buf[3] = 0x8E;      // battery full, normal connection info
  // button bytes 4..6: genuine bit layout from bluetooth_hid_notes
  buf[4] = (uint8_t)(s.buttons & 0xFF);          // Y X B A SL SR + reserved
  buf[5] = (uint8_t)((s.buttons >> 8) & 0xFF);   // - + stick home capture R ZR
  buf[6] = 0;
  // sticks: only right stick exists on Joy-Con R
  buf[7] = 0x80; buf[8] = 0x80; buf[9] = 0;      // left (unused)
  buf[10] = s.stick_x; buf[11] = s.stick_y; buf[12] = 0;
  buf[13] = 0x80;                                 // vibrator input per joycontrol
}

size_t ReportPacker::pack0x30Frames(uint8_t* buf, const ReportState (&st)[3],
                                    uint8_t timer, bool imu_enabled) const {
  putCommon(buf, st[0], timer);
  buf[1] = 0x30;
  if (imu_enabled) {
    for (int f = 0; f < 3; f++) {   // 3 frames, 5ms sampling delta semantics
      uint8_t* p = buf + 14 + f * 12;
      putF(st[f].acc_g[0], acc_scale_, p + 0);
      putF(st[f].acc_g[1], acc_scale_, p + 1);
      putF(st[f].acc_g[2], acc_scale_, p + 2);
      putF(st[f].gyro_dps[0], gyro_scale_, p + 3);
      putF(st[f].gyro_dps[1], gyro_scale_, p + 4);
      putF(st[f].gyro_dps[2], gyro_scale_, p + 5);
    }
  } else {
    memset(buf + 14, 0, 36);
  }
  // Ring-Con strain rides frame 3's accel-Y slot once ExtDev polling is on
  // (0x5C format config points at wire offset 37 len 6 = frame-3 accel; the
  // strain is its middle 2 bytes = parse.ts offset 39 / full-frame 40-41).
  if (st[2].strain_on)
    put16le(buf + 14 + 2 * 12 + 2, (int16_t)st[2].strain_raw);
  return 50;  // 0xA1 + id + timer..IMU end (bytes [0..49])
}

size_t ReportPacker::pack0x30(uint8_t* buf, const ReportState& s, uint8_t timer,
                              bool imu_enabled) const {
  const ReportState frames[3] = {s, s, s};
  return pack0x30Frames(buf, frames, timer, imu_enabled);
}

size_t ReportPacker::pack0x21(uint8_t* buf, const ReportState& s, uint8_t timer,
                              uint8_t ack, uint8_t subcmd_id,
                              const std::vector<uint8_t>& reply) const {
  putCommon(buf, s, timer);
  buf[1] = 0x21;
  buf[14] = ack;
  buf[15] = subcmd_id;
  size_t n = reply.size() < 35 ? reply.size() : 35;
  memcpy(buf + 16, reply.data(), n);
  memset(buf + 16 + n, 0, 35 - n);  // real frames end in zeros, not stack garbage
  return 51;  // 0xA1 + id + timer + battery + btns + sticks + vib + ack + subid + 35
}

bool SpiFlash::read(uint32_t addr, uint8_t* out, uint8_t len) const {
  memset(out, 0xFF, len);
  if (addr == kStickCalAddr && len <= 27) {
    // 0x603D..0x6057 composite: L-stick cal(9) + R-stick cal(9) + 0x604F
    // unknown + body/buttons colors(6). The host reads this range both as
    // 9-byte stick cal and as a 25-byte block spanning into the colors
    // (UARTSwitchCon captured reads of size 0x19), so serve the whole span.
    uint8_t img[27];
    memcpy(img, kSticks18, 18);
    img[18] = 0x00;               // 0x604F unknown
    memcpy(img + 19, kColors, 6); // 0x6050 body + 0x6053 buttons
    memcpy(out, img, len);
    return true;
  }
  if (addr == kStickCalRAddr && len == 9) {
    // 0x6046..0x604E = R-stick factory cal; a Joy-Con R gets asked for this
    memcpy(out, kSticks18 + 9, 9);
    return true;
  }
  if (addr == kDevTypeAddr && len == 1) {
    out[0] = 0x02;  // Joy-Con R
    return true;
  }
  if (addr == kColorExistAddr && len == 1) {
    out[0] = 0x01;  // color info exists (else the host may distrust the colors)
    return true;
  }
  if (addr == kColorAddr && len >= 12 && len <= 14) {
    // Body/buttons/left-grip/right-grip RGB. Switch 2 (unlike Switch 1)
    // rejects the all-0xFF "blank" colors joycontrol gets away with and
    // retries the 0x6050 read forever - 2026-09-08 on-device finding.
    // Values = UARTSwitchCon's field-tested body/buttons/grips set.
    memcpy(out, kColors, len);
    return true;
  }
  if (addr == kImuCalAddr && len == 24) {
    // factory IMU cal: acc origin/sens, gyro origin/sens (dekuNukem defaults)
    static const uint8_t imu[24] = {
        0x76, 0x00, 0xA6, 0xFE, 0xEA, 0x02,             // acc origin
        0x00, 0x40, 0x00, 0x40, 0x00, 0x40,             // acc sensitivity
        0x0E, 0x00, 0xFC, 0xFF, 0xE0, 0xFF,             // gyro origin
        0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34};            // gyro sensitivity
    memcpy(out, imu, 24);
    return true;
  }
  return false;  // 0xFF blank elsewhere
}

}  // namespace joycon
