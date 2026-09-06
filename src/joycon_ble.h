// AIGC note: joycon_ble - NimBLE HID-over-GATT transport presenting as Joy-Con R.
// GATT: HID service (0x1812) with Report Map (genuine Joy-Con descriptor),
// two input report chars (report IDs 0x30 / 0x21) and one output report char
// (report ID 0x01). Security: bonding + LE Secure Connections, Just Works.
//
// Framing: HOGP report characteristic values carry neither the 0xA1 HID marker
// nor the report ID (routing is done via the Report Reference descriptors), so
// 0x30 notifies are 48 bytes and 0x21 replies are 49 bytes. Output reports are
// dispatched by joycon_subcmd (shared with the classic-BT transport).
// Research context (2026-09): real Joy-Con 1 pairs over Bluetooth Classic; the
// Switch 2 grip screen ignored this BLE advertisement (R1 result 2026-09-06),
// so this transport is kept as the PC-testable reference implementation.
#pragma once
#include <Arduino.h>
#include "joycon_report.h"
#include "joycon_subcmd.h"

namespace joycon {

class JoyConBle {
 public:
  void begin(const char* name, uint8_t joycon_type);  // 0x02 = Joy-Con R
  void notify30(const ReportState (&st)[3], uint8_t timer, bool imu_enabled);
  bool connected() const { return connected_; }
  bool subscribed() const { return subscribed_; }
  uint8_t reportMode() const { return st_.report_mode; }
  bool imuEnabled() const { return st_.imu_enabled; }
  void setImuEnabled(bool v) { st_.imu_enabled = v; }
  void setGyroScale(float s) { packer_.setGyroScale(s); }
  float gyroScale() const { return packer_.gyroScale(); }

 private:
  friend class ServerCbs;
  friend class CharCbs;
  void handleOutput(const uint8_t* v, size_t n);

  ReportPacker packer_;
  SubCmdState st_;
  bool connected_ = false;
  bool subscribed_ = false;
  uint16_t conn_handle_ = 0xFFFF;
};

}  // namespace joycon
