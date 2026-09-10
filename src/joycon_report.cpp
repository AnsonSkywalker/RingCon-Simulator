// AIGC note: joycon_report.cpp - input report serialization + SPI flash emulation.
// Protocol facts from dekuNukem reverse-engineering docs (public), re-implemented.
#include "joycon_report.h"
#include <string.h>
#include <math.h>

namespace joycon {

// Colors MUST match the claimed controller type: Switch 2 rejects a Joy-Con R
// wearing Pro-gray (UARTSwitchCon's 0x232323 passes on Switch 1 but loops
// forever on Switch 2, 2026-09-08). Values = the real Ring-Con bundle JC(R)'s
// own factory dump (probe31 v4 0x6050 read, 2026-09-10): neon-yellow body
// #E6FF00, dark buttons #142800, white grips, 0x605C unknown = 0xFF.
static const uint8_t kColors[14] = {0xE6, 0xFF, 0x00, 0x14, 0x28, 0x00,
                                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                    0xFF, 0x00};
// 0x603D L-stick(9) + 0x6046 R-stick(9) factory calibration. Real JC(R) dump
// (probe31 v4 2026-09-10): no L stick exists, its region reads as unwritten
// 0xFF; the R-stick cal is real data.
static const uint8_t kSticks18[18] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                      0xFF, 0xFF, 0xFF, 0xF7, 0xC8, 0x7C,
                                      0x18, 0xF5, 0x46, 0x08, 0x65, 0x4A};
// Regions the NS2 game reads every handshake that the joycontrol-style table
// left as 0xFF blank. Real JC(R) dumps (probe31 v4, 2026-09-10):
// 0x6000 serial IS mirrored as of #8: 00 00 + ASCII serial. It was once
// reverted (test #4 0x08-stall) but that stall is now attributed to "game not
// launched yet" (benign system-level init), and #7 proved the game's mode-0
// hammer persists with everything else mirrored - the serial is the last
// untested identity variable (fw stays on the invented 0x4803 to isolate it).
static const uint8_t kSerial[16] = {  // 0x6000: 0x00 0x00 + ASCII serial
    0x00, 0x00, 0x58, 0x43, 0x57, 0x34, 0x30, 0x30,
    0x34, 0x36, 0x32, 0x32, 0x35, 0x33, 0x38, 0x38};
static const uint8_t kUnk6080[24] = {  // 0x6080: 6B header + 18B cal record
    0x5E, 0x01, 0x00, 0x00, 0x0F, 0xF0,
    0x19, 0xD0, 0x4C, 0xAE, 0x40, 0xE1, 0xEE, 0xE2, 0x2E, 0xEE,
    0xE2, 0x2E, 0xB4, 0x4A, 0xAB, 0x96, 0x64, 0x49};
static const uint8_t kUnk6098[18] = {  // 0x6098: the same 18B record repeats
    0x19, 0xD0, 0x4C, 0xAE, 0x40, 0xE1, 0xEE, 0xE2, 0x2E,
    0xEE, 0xE2, 0x2E, 0xB4, 0x4A, 0xAB, 0x96, 0x64, 0x49};
static const uint8_t kUnk8010[56] = {  // 0x8010..0x8047: blank flash, then a
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   // b2a1-marked 2nd IMU
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   // cal record the NS2
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,               // game follows the chain
    0xB2, 0xA1,                                       // 0x8026 marker
    0x2E, 0xFA, 0x72, 0xFF, 0xA3, 0x00, 0x00, 0x40,   // 0x8028 cal (same 24B
    0x00, 0x40, 0x00, 0x40, 0x24, 0x00, 0xD9, 0xFF,   // layout as 0x6020)
    0xF0, 0xFF, 0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // 0x8040 blank

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
  // sticks/vibrator: byte mirror of a real JC(R) at rest (probe31 2026-09-10).
  // The left-stick region reads as zeros on a JC R (no left stick), the right
  // stick carries its real 12-bit center bytes, and [13] is 0x0A - joycontrol
  // ships 80 80 00 / 80 80 00 / 80 here, which no real frame shows.
  buf[7] = 0x00; buf[8] = 0x00; buf[9] = 0x00;     // left (unused on JC R)
  buf[10] = 0xB4; buf[11] = 0x88; buf[12] = 0x7F;  // right stick center
  buf[13] = 0x0A;                                  // vibrator input byte
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
  // Ring-Con strain rides frame 3's accel block once ExtDev polling is on
  // (0x5C format config points at wire offset 37 len 6 = frame-3 accel).
  // Real Ring-Con ground truth (tools/ring_probe_log.txt, 2026-09-09): the
  // whole block stops being an accelerometer - X=0x0000, Y=strain,
  // Z=0x2000 constant marker, gyro stays live. parse.ts only reads Y at
  // offset 39, but a game validating the block would look for the marker.
  if (st[2].strain_on) {
    uint8_t* p = buf + 14 + 2 * 12;
    put16le(p, 0x0000);
    put16le(p + 2, (int16_t)st[2].strain_raw);
    put16le(p + 4, 0x2000);
  } else {
    // Frame-3 accel slot is reserved for ExtDev data on a real JC: zero-filled
    // until polling is on (probe31 v4 preflight 2026-09-10 - pre-script frames
    // read 00*6 there, never live accelerometer samples). Streaming synthetic
    // accel here reads as "ring data nobody asked for" to the game.
    memset(buf + 14 + 2 * 12, 0, 6);
  }
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
    img[18] = 0xFF;               // 0x604F unknown (real device reads 0xFF)
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
  if (addr == kUnk6080Addr && len <= 24) {
    memcpy(out, kUnk6080, len);
    return true;
  }
  if (addr == kUnk6098Addr && len <= 18) {
    memcpy(out, kUnk6098, len);
    return true;
  }
  if (addr == kSerialAddr && len <= 16) {
    memcpy(out, kSerial, len);
    return true;
  }
  if (addr >= kUnk8010Addr && addr + len <= kUnk8010Addr + sizeof(kUnk8010)) {
    memcpy(out, kUnk8010 + (addr - kUnk8010Addr), len);
    return true;
  }
  if (addr == kImuCalAddr && len == 24) {
    // factory IMU cal - real JC(R) dump (probe31 v4 2026-09-10). Gyro sens
    // matches the old joycontrol values; only the tiny origin offsets differ.
    static const uint8_t imu[24] = {
        0x1D, 0x00, 0x2D, 0xFF, 0xA3, 0x00,             // acc origin
        0x00, 0x40, 0x00, 0x40, 0x00, 0x40,             // acc sensitivity
        0x10, 0x00, 0xE8, 0xFF, 0xD6, 0xFF,             // gyro origin
        0x3B, 0x34, 0x3B, 0x34, 0x3B, 0x34};            // gyro sensitivity
    memcpy(out, imu, 24);
    return true;
  }
  return false;  // 0xFF blank elsewhere
}

}  // namespace joycon
