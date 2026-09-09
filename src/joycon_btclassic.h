// AIGC note: joycon_btclassic - Bluetooth Classic (BR/EDR) HID-device transport
// for the original ESP32, presenting as Joy-Con R. This is the path real
// Joy-Cons use and the one all field-proven emulators (BlueCubeMod /
// UARTSwitchCon / SwitchCon lineage) speak; ESP32-C3 has no BR/EDR radio.
// Uses the esp_bt_hid_device_* API of Arduino core 2.0.x (IDF 4.4). The older
// esp_hid_device_* API this project originally ported from UARTSwitchCon was
// removed before core 2.0; the flow (SDP app params, CoD, intr reports) is the
// same. Sub-command dispatch is shared via joycon_subcmd.
#pragma once
#include <stdint.h>
#include "joycon_report.h"
#include "joycon_subcmd.h"

namespace joycon {

struct BtClassicHooks;  // friend; bridges the C-style event callback

class JoyConBtClassic {
 public:
  void begin(const char* name, uint8_t joycon_type);  // 0x02 = Joy-Con R
  void notify30(const ReportState (&st)[3], uint8_t timer, bool imu_enabled);
  // Simple-HID (0x3F) report with both button bytes, driven remotely from the
  // PC frontend via the 'kb' command. b1 bits: 0x01=Down(A) 0x02=Right(X)
  // 0x04=Left(B) 0x08=Up(Y) 0x10=SL 0x20=SR (sideways right Joy-Con).
  void notify3F(uint8_t btn1, uint8_t btn2);
  // Simulate Home-held sleep: drop the HID connection (Switch icon goes
  // away); we stay discoverable so any later pairing attempt re-links.
  void disconnect();
  // Device-initiated page (real Joy-Con "press any key to reconnect"):
  // rings back the last host over BR/EDR using the stored link key - this is
  // how "one pairing, permanent reconnect" actually works on the wire.
  bool reconnect();
  void setHidden(bool h) { hidden_ = h; }  // sleep: don't re-advertise on CLOSE
  bool haveHost() const { return have_host_; }
  void forgetHost(); // erase NVS host memory: fresh pairing (sync long-press)
  bool pagingStale(int64_t now_us) const {
    return paging_ && now_us - page_sent_us_ > 5000000;
  }
  void clearStalePage() { paging_ = false; }
  bool connected() const { return connected_; }
  // Set when an OPEN_EVT reports failure: the ESP-HIDD BTC layer then refuses
  // every further esp_bt_hid_device_connect with "busy now" until reboot.
  bool pageWedge() const { return page_wedge_; }
  // Classic HID has no CCCD: once the L2CAP interrupt channel is up, reports
  // simply flow. Kept for parity with the BLE transport's interface.
  bool subscribed() const { return connected_; }
  uint8_t reportMode() const { return st_.report_mode; }
  bool imuEnabled() const { return st_.imu_enabled; }
  bool extdevPolling() const { return st_.extdev_polling; }
  void setImuEnabled(bool v) { st_.imu_enabled = v; }
  void setGyroScale(float s) { packer_.setGyroScale(s); }
  float gyroScale() const { return packer_.gyroScale(); }

 private:
  friend struct BtClassicHooks;
  void handleOutput(const uint8_t* v, size_t n);
  void hostSave();   // persist last_host_ in NVS (survive reboots)
  void hostLoad();   // restore on boot

  ReportPacker packer_;
  SubCmdState st_;
  volatile bool connected_ = false;
  uint8_t last_host_[6] = {0};
  bool have_host_ = false;
  bool hidden_ = false;      // sleep state: CLOSE must not re-advertise
  bool paging_ = false;      // device-initiated page in flight
  bool page_wedge_ = false;  // BTC layer stuck after a failed page
  int64_t page_sent_us_ = 0;  // when the page was issued (stale-clear)
};

}  // namespace joycon
