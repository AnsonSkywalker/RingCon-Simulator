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
#include <esp_timer.h>

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
          g_self->paging_ = false;
          memcpy(g_self->last_host_, param->open.bd_addr, 6);
          g_self->have_host_ = true;  // remember the host for later page-back
          g_self->hostSave();         // persist across reboots
          // one host is enough; stop being discoverable while linked
          esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE,
                                   ESP_BT_NON_DISCOVERABLE);
          JCLOG("[bt] connected\n");
        } else if (param->open.status != ESP_HIDD_SUCCESS) {
          g_self->paging_ = false;
          JCLOG("[bt] open evt, status=%d conn=%d\n", param->open.status,
                        param->open.conn_status);
        }
        break;
      case ESP_HIDD_CLOSE_EVT:
        if (param->close.conn_status != ESP_HIDD_CONN_STATE_CONNECTED) {
          g_self->connected_ = false;
          g_self->paging_ = false;
          // real Joy-Con semantics: after a link drop the controller wakes up
          // clean - the host re-runs the full handshake (mode/IMU) from zero
          g_self->st_.report_mode = 0x3F;
          g_self->st_.imu_enabled = false;
          // while asleep (rst) stay off the air; otherwise re-advertise so a
          // scanning host can find us (first-ever pairing has no host to page)
          if (!g_self->hidden_)
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                     ESP_BT_GENERAL_DISCOVERABLE);
          JCLOG("[bt] disconnected (state reset)%s\n",
                        g_self->hidden_ ? ", asleep" : ", discoverable");
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
        // app registration, control-channel data, suspend/resume, ...
        // (6 = SEND_REPORT echo of our own 66 Hz stream - too noisy to print)
        if ((int)event != 6) JCLOG("[bt] hidd evt %d\n", (int)event);
        break;
    }
  }

  // Observability for pairing: the Switch's connect attempts show up here
  // (ACL up/down, SSP events) even when the HID channels fail.
  static void onGapEvent(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t* param) {
    switch (event) {
      case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        JCLOG("[gap] acl connected, st=%d\n",
                      param->acl_conn_cmpl_stat.stat);
        break;
      case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        JCLOG("[gap] acl disconnected, reason=0x%x\n",
                      param->acl_disconn_cmpl_stat.reason);
        break;
      case ESP_BT_GAP_AUTH_CMPL_EVT:
        JCLOG("[gap] auth complete, stat=%d key_type=%d\n",
                      param->auth_cmpl.stat, param->auth_cmpl.lk_type);
        break;
      case ESP_BT_GAP_ENC_CHG_EVT:
        JCLOG("[gap] encryption mode=%d\n", param->enc_chg.enc_mode);
        break;
      default:
        JCLOG("[gap] evt %d\n", (int)event);
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
  // NoInputNoOutput -> SSP Just Works, same as the real Joy-Con
  uint8_t io_cap = ESP_BT_IO_CAP_NONE;
  esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &io_cap, sizeof(io_cap));
  esp_bt_gap_register_callback(&BtClassicHooks::onGapEvent);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  hostLoad();  // restore a previously-bonded host for instant page-back
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

void JoyConBtClassic::notify3F(uint8_t btn1, uint8_t btn2) {
  if (!connected_) return;
  // [0xA1][0x3F][btn1][btn2][hat=8 center][filler 00 80 00 80 00 80 00 80]
  uint8_t buf[13] = {0xA1, 0x3F, btn1, btn2, 0x08,
                     0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80};
  esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x3F, 11,
                                buf + 2);
}

void JoyConBtClassic::disconnect() {
  if (connected_) esp_bt_hid_device_disconnect();
}

bool JoyConBtClassic::reconnect() {
  if (connected_ || !have_host_ || paging_) return false;
  paging_ = true;  // one page in flight; OPEN/CLOSE clears it
  page_sent_us_ = (int64_t)esp_timer_get_time();
  esp_err_t rc = esp_bt_hid_device_connect(last_host_);
  if (rc != ESP_OK) {
    paging_ = false;
    JCLOG("[bt] page host rc=%d\n", rc);
  }
  return rc == ESP_OK;
}

void JoyConBtClassic::hostSave() {
  nvs_handle_t h;
  if (nvs_open("ringcon", NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_blob(h, "host", last_host_, 6);
    nvs_commit(h);
    nvs_close(h);
  }
}

void JoyConBtClassic::hostLoad() {
  nvs_handle_t h;
  if (nvs_open("ringcon", NVS_READONLY, &h) == ESP_OK) {
    size_t len = 6;
    if (nvs_get_blob(h, "host", last_host_, &len) == ESP_OK && len == 6)
      have_host_ = true;
    nvs_close(h);
  }
  if (have_host_)
    JCLOG("[bt] loaded last host from NVS: %02X:%02X:%02X:%02X:%02X:%02X\n",
          last_host_[0], last_host_[1], last_host_[2],
          last_host_[3], last_host_[4], last_host_[5]);
}

void JoyConBtClassic::handleOutput(const uint8_t* v, size_t n) {
  // Wire diagnosis: dump what the host actually sends after the stack strips
  // the 0xA2 DATA-OUTPUT transaction header (Switch 2 framing unknown at the
  // time of writing - its frames ran 10 bytes longer than Switch 1's).
  char hex[3 * 34 + 1];
  size_t m = n < 16 ? n : 16;
  for (size_t i = 0; i < m; i++) sprintf(hex + 3 * i, "%02X ", v[i]);
  hex[3 * m] = 0;
  JCLOG("[rx] n=%u %s\n", (unsigned)n, hex);

  uint8_t out51[51];
  size_t len = dispatchOutputReport(v, n, st_, packer_, out51);
  if (len > 0 && connected_) {
    size_t t = len < 34 ? len : 34;
    for (size_t i = 0; i < t; i++) sprintf(hex + 3 * i, "%02X ", out51[i]);
    hex[3 * t] = 0;
    JCLOG("[tx] l=%u %s\n", (unsigned)len, hex);
    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x21,
                                  static_cast<uint16_t>(len - 2), out51 + 2);
  }
}

}  // namespace joycon
