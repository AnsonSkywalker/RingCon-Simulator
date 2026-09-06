// AIGC note: joycon_ble.cpp - NimBLE HOGP server. Sub-command dispatch moved to
// joycon_subcmd (shared with the classic-BT transport); this file only wires
// GATT plumbing, security, and advertising.
#include "joycon_ble.h"
#include "joycon_hid_map.h"

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include <NimBLEHIDDevice.h>
#include <NimBLESecurity.h>

namespace joycon {

namespace {

JoyConBle* g_ble = nullptr;
NimBLEHIDDevice* g_hid = nullptr;
NimBLECharacteristic* g_in30 = nullptr;   // report ID 0x30 (standard full + IMU)
NimBLECharacteristic* g_in21 = nullptr;   // report ID 0x21 (sub-command reply)
NimBLECharacteristic* g_out = nullptr;    // report ID 0x01 (0xA2 output reports)

// HOGP report values omit the 0xA1 marker and the report ID. Set to 0 to send
// classic-style frames (0xA1 + ID + payload) if a host turns out to want them.
#define JC_HOGP_STRIP_ID 1

void notifyBytes(NimBLECharacteristic* ch, const uint8_t* frame, size_t frame_len) {
#if JC_HOGP_STRIP_ID
  ch->notify(frame + 2, frame_len - 2, true);
#else
  ch->notify(frame, frame_len, true);
#endif
}

}  // namespace

// These two live at joycon scope (not anonymous) because the header declares
// them as friends to reach the private connection state.
class ServerCbs : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
    g_ble->connected_ = true;
    g_ble->conn_handle_ = desc->conn_handle;
    // R5: ask for a fast connection interval (7.5..15 ms) for steady 66 Hz.
    server->updateConnParams(desc->conn_handle, 6, 12, 0, 600);
    Serial.printf("[ble] connected, handle=%u\n", desc->conn_handle);
  }
  void onDisconnect(NimBLEServer* server, ble_gap_conn_desc* desc) override {
    (void)server;
    g_ble->connected_ = false;
    g_ble->subscribed_ = false;
    g_ble->conn_handle_ = 0xFFFF;
    Serial.println("[ble] disconnected, advertising again");
    NimBLEDevice::startAdvertising();
  }
};

class CharCbs : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* ch, ble_gap_conn_desc* desc,
                   uint16_t subValue) override {
    (void)desc;
    g_ble->subscribed_ = ch->getSubscribedCount() > 0;
    Serial.printf("[ble] subscribe handle=%u val=0x%04x (any=%d)\n",
                  ch->getHandle(), subValue, g_ble->subscribed_);
  }
  void onWrite(NimBLECharacteristic* ch) override {
    NimBLEAttValue v = ch->getValue();
    g_ble->handleOutput((const uint8_t*)v.data(), v.length());
  }
};

void JoyConBle::begin(const char* name, uint8_t joycon_type) {
  g_ble = this;
  st_.joycon_type = joycon_type;
  packer_.begin(joycon_type);

  NimBLEDevice::init(name);
  NimBLEDevice::setSecurityAuth(true, false, true);           // bond + SC
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);  // Just Works
  NimBLEAddress addr = NimBLEDevice::getAddress();
  memcpy(st_.mac, addr.getNative(), 6);  // little-endian native

  ServerCbs* srv_cbs = new ServerCbs();
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(srv_cbs);

  g_hid = new NimBLEHIDDevice(server);
  g_hid->reportMap(const_cast<uint8_t*>(kJoyConReportMap), kJoyConReportMapLen);
  g_hid->manufacturer("Nintendo");
  g_hid->pnp(0x01, 0x057e, 0x2007, 0x0001);  // vendor Nintendo, Joy-Con R pid
  g_hid->hidInfo(0x00, 0x02);                // country 0, normally connectable
  g_hid->setBatteryLevel(0x64);

  CharCbs* ch_cbs = new CharCbs();
  g_in30 = g_hid->inputReport(0x30);
  g_in21 = g_hid->inputReport(0x21);
  g_out = g_hid->outputReport(0x01);
  g_out->setCallbacks(ch_cbs);

  g_hid->startServices();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NimBLEUUID((uint16_t)0x1812));
  adv->setAppearance(HID_GAMEPAD);
  adv->setScanResponse(true);
  adv->start();
  Serial.println("[ble] advertising as Joy-Con (R)");
}

void JoyConBle::notify30(const ReportState (&st)[3], uint8_t timer, bool imu_enabled) {
  if (!connected_ || !subscribed_) return;
  uint8_t buf[50];
  packer_.pack0x30Frames(buf, st, timer, imu_enabled);
  notifyBytes(g_in30, buf, sizeof(buf));
}

void JoyConBle::handleOutput(const uint8_t* v, size_t n) {
  uint8_t out51[51];
  size_t len = dispatchOutputReport(v, n, st_, packer_, out51);
  if (len > 0 && connected_ && subscribed_) {
    notifyBytes(g_in21, out51, len);
  }
}

}  // namespace joycon
