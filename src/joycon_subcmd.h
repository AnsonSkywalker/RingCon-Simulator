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
  // MCU mode register, set via subcmd 0x21 payload [0x21][subcmd][mode]:
  // 0x01 standby, 0x03 Ring-Con (NS2 game flow). The 0x21 ack body carries
  // live MCU state (body[7] = mode before the command). NOTE: the mode-0
  // suspend does NOT touch this register (probe31 v3 2026-09-10: body[7]
  // stays across two mode-0s and the following mode-3 ack).
  // #9 (2026-09-10 evening): init = 0x06, not 0x01. Live-device replay of the
  // game's exact #8 sequence showed the real JC's FIRST mode-0 ack reads
  // body[7]=0x06 (follow-up bare mode-0s read 0x01) - 0x06 = "an ExtDev
  // session happened and was parked" (our board never reaches it because the
  // game never sends 0x5A to us). The real device in the user's flow ALWAYS
  // has that history (previous gameplay), so the game's ring check plausibly
  // expects 0x06; a virgin 0x01 is what we shipped for 8 failing tests.
  uint8_t mcu_mode = 0x06;
  // Fresh-resume marker: 0x22[0x01] (MCU resume) sets it, the FIRST 0x21
  // MCU-config ack carries body[2]=0xFF and consumes it (probe31 v3: real
  // mode-0 #1 ack = 01 00 FF 00 09 00 20 01, later acks carry 00). The NS2
  // game rejects a mode-0 ack without the FF (retries then drops the link).
  bool mcu_resume_fresh = false;
};

// Parses one output report (counter, rumble*8, subcmd, args...). Accepts an
// optional leading report-id byte (0x01/0x10/0x11/0x12). On success writes a
// complete classic 0x21 frame (0xA1 + id + ack..) into out51 and returns the
// frame length (always 51); returns 0 when the report is malformed.
size_t dispatchOutputReport(const uint8_t* v, size_t n, SubCmdState& st,
                            const ReportPacker& packer, uint8_t* out51);

}  // namespace joycon
