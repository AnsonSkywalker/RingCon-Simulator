// AIGC note: joycon_subcmd - transport-agnostic output-report (0xA2) parser and
// sub-command dispatcher. Shared by the BLE (NimBLE HOGP) and Bluetooth Classic
// (Bluedroid esp_hidd) transports; only the reply byte-plumbing differs.
// Ack values follow joycontrol's field-tested table: 0x82 device info,
// 0x83 trigger-times, 0x90 SPI flash read, 0xA0 NFC/IR MCU config, else 0x80.
#pragma once
#include <stdint.h>
#include "joycon_report.h"

namespace joycon {

struct SubCmdState {
  bool imu_enabled = false;
  // 0x3F (simple HID) is the real power-on report mode; the host must send
  // subcmd 0x03 with 0x30 before any full input reports flow. Streaming
  // 0x30 unsolicited derails the Switch 2 handshake (2026-09-08 finding).
  uint8_t report_mode = 0x3F;
  uint8_t joycon_type = 0x02;  // 0x02 = Joy-Con R
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};  // transport fills (little-endian native)
  // Ring-Con (ExtDev) state: the game enables MCU polling (0x5A) after a
  // 0x59 GetExtDevInfo reply carrying ext-device ID 0x20; from then on the
  // 0x30 report's 3rd IMU frame accel-Y slot carries the strain value
  // (refs/ringcon/connectRingCon.ts + ringrunnermg joycon.hpp, byte-verified).
  bool extdev_polling = false;
};

// Parses one output report (counter, rumble*8, subcmd, args...). Accepts an
// optional leading report-id byte (0x01/0x10/0x11/0x12). On success writes a
// complete classic 0x21 frame (0xA1 + id + ack..) into out51 and returns the
// frame length (always 51); returns 0 when the report is malformed.
size_t dispatchOutputReport(const uint8_t* v, size_t n, SubCmdState& st,
                            const ReportPacker& packer, uint8_t* out51);

}  // namespace joycon
