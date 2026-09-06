// AIGC note: joycon_btclassic.cpp - Bluedroid classic HID device (IDF 4.4 API).
// Sequence: NVS -> controller (CLASSIC_BT only, BLE memory released) ->
// bluedroid -> HID app registration (genuine Joy-Con report map, Nintendo
// provider, CoD major 5 minor 2 service 1) -> discoverable, name "Joy-Con (R)".
// Output reports arrive on ESP_HIDD_INTR_DATA_EVT without a leading report-id
// byte and go through the shared dispatcher; replies and the 15 ms 0x30 stream
// leave via esp_bt_hid_device_send_report with payload = classic frame minus
// 0xA1 and report-id bytes (48 B for 0x30, 49 B for 0x21).
#include "joycon_btclassic.h"
#include "joycon_hid_map.h"
#include "jc_log.h"
#include <string.h>

#include <nvs_flash.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_bt_api.h>
#include <esp_hidd_api.h>

namespace joycon {

namespace {
JoyConBtClassic* g_self = nullptr;
}  // namespace

struct BtClassicHooks {
  static void onEvent(esp_hidd_cb_event_t event, esp_hidd_cb_param_t* param) {
    switch (event) {
      case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS &&
            param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
          g_self->connected_ = true;
          // one host is enough; stop being discoverable while linked
          esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE,
                                   ESP_BT_NON_DISCOVERABLE);
          JCLOG("[bt] connected\n");
        }
        break;
      case ESP_HIDD_CLOSE_EVT:
        if (param->close.conn_status != ESP_HIDD_CONN_STATE_CONNECTED) {
          g_self->connected_ = false;
          esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                   ESP_BT_GENERAL_DISCOVERABLE);
          JCLOG("[bt] disconnected, discoverable again\n");
        }
        break;
      case ESP_HIDD_INTR_DATA_EVT:
        g_self->handleOutput(param->intr_data.data, param->intr_data.len);
        break;
      case ESP_HIDD_REPORT_ERR_EVT:
        JCLOG("[bt] report handshake error, status=%d\n",
                      param->report_err.status);
        break;
      default:
        break;
    }
  }
};

void JoyConBtClassic::begin(const char* name, uint8_t joycon_type) {
  g_self = this;
  st_.joycon_type = joycon_type;
  packer_.begin(joycon_type);

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  // classic-only: the BLE controller memory is dead weight here
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
  ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
  ESP_ERROR_CHECK(esp_bluedroid_init());
  ESP_ERROR_CHECK(esp_bluedroid_enable());

  const uint8_t* mac = esp_bt_dev_get_address();
  if (mac) memcpy(st_.mac, mac, 6);

  static esp_hidd_app_param_t app_param;
  static esp_hidd_qos_param_t both_qos;
  app_param.name = const_cast<char*>("Wireless Gamepad");
  app_param.description = const_cast<char*>("Gamepad");
  app_param.provider = const_cast<char*>("Nintendo");
  app_param.subclass = 0x8;
  app_param.desc_list = const_cast<uint8_t*>(kJoyConReportMap);
  app_param.desc_list_len = static_cast<int>(kJoyConReportMapLen);
  memset(&both_qos, 0, sizeof(both_qos));

  ESP_ERROR_CHECK(esp_bt_hid_device_register_callback(&BtClassicHooks::onEvent));
  ESP_ERROR_CHECK(esp_bt_hid_device_init());
  ESP_ERROR_CHECK(
      esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos));

  esp_bt_dev_set_device_name(name);
  esp_bt_cod_t cod = {};  // major 5 = peripheral, minor 2, service 1
  cod.minor = 2;
  cod.major = 5;
  cod.service = 1;
  esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  JCLOG("[bt] discoverable as %s\n", name);
}

void JoyConBtClassic::notify30(const ReportState (&st)[3], uint8_t timer,
                               bool imu_enabled) {
  if (!connected_) return;
  uint8_t buf[50];
  packer_.pack0x30Frames(buf, st, timer, imu_enabled);
  // payload = classic frame minus 0xA1 and report-id (the API takes the id)
  esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30, 48, buf + 2);
}

void JoyConBtClassic::handleOutput(const uint8_t* v, size_t n) {
  uint8_t out51[51];
  size_t len = dispatchOutputReport(v, n, st_, packer_, out51);
  if (len > 0 && connected_) {
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                  static_cast<uint16_t>(len - 2), out51 + 2);
  }
}

}  // namespace joycon
