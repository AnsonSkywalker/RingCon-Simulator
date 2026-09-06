// AIGC note: joycon_ble - NimBLE HID-over-GATT transport presenting as Joy-Con R.
// GATT: HID service (0x1812) with Report Map (genuine Joy-Con descriptor),
// two input report chars (report IDs 0x30 / 0x21) and one output report char
// (report ID 0x01). Security: bonding + LE Secure Connections, Just Works.
//
// Framing: HOGP report characteristic values carry neither the 0xA1 HID marker
// nor the report ID (routing is done via the Report Reference descriptors), so
// 0x30 notifies are 48 bytes and 0x21 replies are 49 bytes. On the output side
// some hosts still prepend the report ID; both forms are accepted.
// Research context (2026-09): real Joy-Cons pair over Bluetooth Classic, and no
// public project verifies the Joy-Con handshake over BLE - treat pairing with a
// real Switch as the R1 experiment this firmware exists to run.
#pragma once
#include <Arduino.h>
#include "joycon_report.h"

namespace joycon {

class ServerCbs;
class CharCbs;

class JoyConBle {
 public:
  void begin(const char* name, uint8_t joycon_type);  // 0x02 = Joy-Con R
  void notify30(const ReportState (&st)[3], uint8_t timer, bool imu_enabled);
  bool connected() const { return connected_; }
  bool subscribed() const { return subscribed_; }
  uint8_t reportMode() const { return report_mode_; }
  bool imuEnabled() const { return imu_enabled_; }
  void setImuEnabled(bool v) { imu_enabled_ = v; }
  void setGyroScale(float s) { packer_.setGyroScale(s); }
  float gyroScale() const { return packer_.gyroScale(); }

 private:
  friend class ServerCbs;
  friend class CharCbs;
  void handleOutput(const uint8_t* v, size_t n);
  void notify21(uint8_t ack, uint8_t subcmd, const uint8_t* data, size_t len);

  ReportPacker packer_;
  uint8_t joycon_type_ = 0x02;
  bool connected_ = false;
  bool subscribed_ = false;
  bool imu_enabled_ = false;
  uint8_t report_mode_ = 0x30;
  uint16_t conn_handle_ = 0xFFFF;
};

}  // namespace joycon
