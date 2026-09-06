// AIGC note: joycon_ble.cpp - NimBLE HOGP server + sub-command dispatcher.
// Protocol answers follow docs/protocol_notes.md (dekuNukem notes + joycontrol
// memory.py defaults). Re-implemented, not copied (GPL sources kept at arm's length).
#include "joycon_ble.h"
#include "joycon_hid_map.h"

#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLECharacteristic.h>
#include <NimBLEHIDDevice.h>
#include <NimBLESecurity.h>
#include <vector>

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
  joycon_type_ = joycon_type;
  packer_.begin(joycon_type);

  NimBLEDevice::init(name);
  NimBLEDevice::setSecurityAuth(true, false, true);       // bonding + SC, no MITM
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);  // Just Works

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

void JoyConBle::notify21(uint8_t ack, uint8_t subcmd, const uint8_t* data, size_t len) {
  if (!connected_ || !subscribed_) return;
  uint8_t buf[51];
  ReportState idle;  // buttons/sticks static, IMU zeroed in 0x21 replies
  std::vector<uint8_t> r(data, data + (data ? len : 0));
  size_t n = packer_.pack0x21(buf, idle, 0, ack, subcmd, r);
  notifyBytes(g_in21, buf, n);
}

void JoyConBle::handleOutput(const uint8_t* v, size_t n) {
  // Accept both "report ID + payload" and "payload only" framings.
  const uint8_t* p = v;
  if (n > 0 && (p[0] == 0x01 || p[0] == 0x10 || p[0] == 0x11 || p[0] == 0x12)) {
    p++;
    n--;
  }
  if (n < 10) {  // counter(1) + rumble(8) + subcmd id(1)
    Serial.printf("[sub] short output report (%u bytes)\n", (unsigned)n);
    return;
  }
  uint8_t sub = p[9];
  const uint8_t* arg = n > 10 ? p + 10 : nullptr;
  size_t argn = n > 10 ? n - 10 : 0;
  Serial.printf("[sub] 0x%02x (args %u)\n", sub, (unsigned)argn);

  switch (sub) {
    case 0x02: {  // REQUEST_DEVICE_INFO: fw(2) type(1) 0x02 mac(6) 0x01 0x01
      uint8_t r[12] = {0x03, 0x48, joycon_type_, 0x02, 0, 0, 0, 0, 0, 0, 0x01, 0x01};
      const uint8_t* mac = NimBLEDevice::getAddress().getNative();  // little-endian
      for (int i = 0; i < 6; i++) r[4 + i] = mac[5 - i];            // big-endian out
      notify21(0x80, sub, r, sizeof(r));
      break;
    }
    case 0x03:  // SET_INPUT_REPORT_MODE
      if (argn >= 1) report_mode_ = arg[0];
      notify21(0x80, sub, nullptr, 0);
      break;
    case 0x04: {  // TRIGGER_BUTTONS_ELAPSED_TIME: 7 x uint16 zeros
      uint8_t r[14] = {0};
      notify21(0x80, sub, r, sizeof(r));
      break;
    }
    case 0x08:  // SET_SHIPMENT_STATE
    case 0x21:  // SET_NFC_IR_MCU_CONFIG
    case 0x22:  // SET_NFC_IR_MCU_STATE
    case 0x30:  // SET_PLAYER_LIGHTS
    case 0x48:  // ENABLE_VIBRATION (no rumble motor here)
      notify21(0x80, sub, nullptr, 0);
      break;
    case 0x40:  // ENABLE_6AXIS_SENSOR
      imu_enabled_ = argn >= 1 && arg[0] != 0;
      Serial.printf("[sub] 6-axis %s\n", imu_enabled_ ? "on" : "off");
      notify21(0x80, sub, nullptr, 0);
      break;
    case 0x10: {  // SPI_FLASH_READ: addr u32le + size u8, echo addr+size+data
      if (argn < 5) {
        notify21(0x80, sub, nullptr, 0);
        break;
      }
      uint32_t addr = (uint32_t)arg[0] | ((uint32_t)arg[1] << 8) |
                      ((uint32_t)arg[2] << 16) | ((uint32_t)arg[3] << 24);
      uint8_t size = arg[4] > 0x1D ? 0x1D : arg[4];
      uint8_t r[4 + 1 + 0x1D];
      memcpy(r, &addr, 4);
      r[4] = size;
      SpiFlash flash;
      flash.read(addr, r + 5, size);  // fills 0xFF for unmapped regions
      notify21(0x80, sub, r, 5 + size);
      break;
    }
    default:  // unknown sub-command: ack so the handshake never stalls
      notify21(0x80, sub, nullptr, 0);
      break;
  }
}

}  // namespace joycon
