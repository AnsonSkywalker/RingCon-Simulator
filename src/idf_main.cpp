// AIGC note: idf_main - ESP-IDF entry point for the Bluetooth Classic
// transport (env:esp32classic). Streams the calibrated two-way squat-twist
// curve (HANDOFF §〇.6) instead of the old one-way 90/500 pulse, and mirrors
// a trimmed-down serial CLI (no MPU commands) on UART0 so axis/sign/scale/
// repeat can be tuned from the PC without reflashing.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_gap_bt_api.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "jc_log.h"
#include "joycon_btclassic.h"
#include "motion.h"

static joycon::JoyConBtClassic transport;
static joycon::TwistMotion motion;
static joycon::RingStrain strain;
static uint8_t g_timer = 0;
static bool g_motion_on = true;       // 'm' command: synthetic motion vs still
static bool g_repeat = true;          // 'r' command: twist every 1.5 s
static int64_t g_next_repeat_us = 0;
static uint32_t g_reports = 0;
// Remote-controlled button bytes (PC frontend / serial). 0x3F simple mode:
// b1 = Down(A)/Right(X)/Left(B)/Up(Y)/SL/SR, b2 = Minus/Plus/Home/Capture/R/ZR.
// 0x30 full mode: b4/b5 are report bytes 4/5 (A=0x08 in b4).
static uint8_t g_kb1 = 0, g_kb2 = 0;
static uint8_t g_30_b4 = 0, g_30_b5 = 0;

static int64_t now_us() { return esp_timer_get_time(); }

// Real Joy-Con sleep/wake semantics: standby = radio silent (invisible and
// non-connectable, like a sleeping Joy-Con); ONLY a button press wakes it.
// Wake pages the bonded host inside a bounded search window (the blinking
// phase) and returns to standby if the host never answers. Non-button
// commands do NOT wake the device - a real Joy-Con ignores everything but
// its keys while asleep.
static bool g_asleep = false;
static int64_t g_last_reconnect_us = 0;  // rate-limit page retries
static int64_t g_search_deadline_us = 0;  // 0 = not searching
static constexpr int64_t kSearchWindowUs = 15000000;  // blink ~15 s, then sleep
static bool g_was_connected = false;

static void sleepNow() {
  g_asleep = true;
  g_search_deadline_us = 0;
  transport.setHidden(true);  // CLOSE_EVT must not re-advertise
  esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
  g_kb1 = g_kb2 = g_30_b4 = g_30_b5 = 0;  // sleep clears pressed buttons
  strain.reset();  // and the ring strain channel (polling resets on CLOSE)
}

static void wake() {
  if (!g_asleep) return;
  g_asleep = false;
  transport.setHidden(false);
  if (transport.connected()) return;  // a late page already re-linked us
  if (transport.haveHost()) {
    // real Joy-Con wake = page the host and re-establish the link (stored
    // link key, no re-pairing); connectable-only so nothing else grabs us
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    g_search_deadline_us = now_us() + kSearchWindowUs;
    transport.clearStalePage();
    JCLOG("[cmd] awake, paging host (search window %d s, page=%d)\n",
          (int)(kSearchWindowUs / 1000000), transport.reconnect());
    return;
  }
  // no bonded host yet: pairing mode, wait for the console to find us
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  JCLOG("[cmd] awake, discoverable (no host)\n");
}

// Abstract button -> report bit routing. The frontend never mentions report
// modes; whichever phase the host put us in (0x3F grip screen / 0x30 full)
// decides where a named button lands. 0x3F d-pad bits follow the user-verified
// sideways mapping (physical A reads as "X" in the paired view => d-pad Up);
// 0x30 bits are the raw joycontrol-verified layout.
namespace {
struct BtnRoute {
  const char* name;
  uint8_t f_byte, f_bit;    // 0x3F simple report bytes 1/2
  uint8_t full_byte, full_bit;  // 0x30 report bytes 4/5
};
const BtnRoute kBtnRoutes[] = {
    {"A", 1, 0x08, 4, 0x08},  // sideways A = d-pad Up (verified: pairs-view X)
    {"B", 1, 0x02, 4, 0x04},  // sideways B = d-pad Right
    {"X", 1, 0x04, 4, 0x02},  // sideways X = d-pad Left
    {"Y", 1, 0x01, 4, 0x01},  // sideways Y = d-pad Down
    {"SL", 1, 0x10, 4, 0x20},
    {"SR", 1, 0x20, 4, 0x10},
    {"R", 2, 0x40, 4, 0x40},   // grip-screen pair button (pair mode uses L/R)
    {"ZR", 2, 0x80, 4, 0x80},
    {"Plus", 2, 0x02, 5, 0x02},
    {"RStick", 2, 0x04, 5, 0x04},
    {"Home", 2, 0x10, 5, 0x10},
};
void setBit(uint8_t& byte, uint8_t bit, bool on) {
  if (on) byte |= bit;
  else byte &= ~bit;
}
}  // namespace

// On-board BOOT key (GPIO0, active low) = the Joy-Con sync key (short press
// = disconnect, hold >=3 s = wipe pairing memory and reboot, see reportTask).
static bool bootPressed() { return gpio_get_level(GPIO_NUM_0) == 0; }

// SYNC-key long-press / 'unpair': forget the page-back host (NVS) and drop
// every classic-BT bond (link keys), then reboot straight into pairing mode.
// This is the way out of a stale state - e.g. the board once connected to a
// PC during debugging and kept paging the wrong host ever since.
static int64_t g_sync_down_us = 0;
static bool g_sync_long_fired = false;

static void wipePairingAndReboot() {
  JCLOG("[sync] wiping host memory + all bonds, rebooting to pairing\n");
  transport.forgetHost();
  esp_bd_addr_t list[8];
  int n = esp_bt_gap_get_bond_device_num();
  n = n < 8 ? n : 8;
  if (n > 0 && esp_bt_gap_get_bond_device_list(&n, list) == ESP_OK)
    for (int i = 0; i < n; i++) esp_bt_gap_remove_bond_device(list[i]);
  vTaskDelay(pdMS_TO_TICKS(200));  // let the stack's NVS writes land
  esp_restart();
}

static void reportTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    // physical BOOT = the sync key: short press <1 s disconnects (stays
    // paired), hold >=3 s wipes pairing memory and reboots into pairing
    // mode; 1-3 s is a dead zone so a sloppy hold does nothing
    bool sync_down = bootPressed();
    if (sync_down && !g_sync_down_us) g_sync_down_us = now_us();
    if (sync_down && g_sync_down_us && !g_sync_long_fired &&
        now_us() - g_sync_down_us >= 3000000) {
      g_sync_long_fired = true;
      wipePairingAndReboot();  // never returns
    }
    if (!sync_down && g_sync_down_us) {
      int64_t held_us = now_us() - g_sync_down_us;
      g_sync_down_us = 0;
      if (g_sync_long_fired) {
        // long press already fired; wait for the reboot
      } else if (held_us < 1000000 && transport.connected()) {
        JCLOG("[sync] short press: disconnect (stays paired)\n");
        transport.disconnect();  // link machine runs the 15 s search window
      } else if (held_us < 1000000) {
        JCLOG("[sync] short press ignored (not connected)\n");
      }
    }
    // The motion state machine (incl. persistent orientation) advances in
    // real time even while disconnected, so rot/reset settle before the
    // host ever sees a report; only the reporting itself is gated.
    joycon::ReportState frames[3];
    uint32_t btn = (uint32_t)g_30_b4 | ((uint32_t)g_30_b5 << 8);
    strain.tick(5);  // Ring-Con strain channel (slews toward slider/push goal)
    for (int i = 0; i < 3; i++) {
      frames[i] = joycon::ReportState{};
      frames[i].buttons = btn;
      if (g_motion_on) motion.tick(5, frames[i]);
      // strain only rides the stream after the game enabled ExtDev polling
      frames[i].strain_on = transport.extdevPolling();
      frames[i].strain_raw = strain.raw();
    }
    if (transport.connected() && transport.reportMode() == 0x30) {
      transport.notify30(frames, g_timer++, transport.imuEnabled());
      g_reports++;
    } else if (transport.connected() && transport.reportMode() == 0x3F) {
      // pre-handshake simple mode: buttons come from the PC frontend
      // (tools/jc_remote.py) via 'kb'; the physical BOOT key is the sync
      // key now, NOT a button
      uint8_t b1 = g_kb1, b2 = g_kb2;
      transport.notify3F(b1, b2);
    }
    if (g_repeat && !motion.active() && now_us() >= g_next_repeat_us) {
      motion.armTwist();
      g_next_repeat_us = now_us() + 1500000;
    }
    // link state machine: connected -> clear the search window; host vanished
    // -> blink window with page retries; window expired -> radio-silent
    // standby (a real Joy-Con does not search forever, it goes back to sleep)
    bool conn = transport.connected();
    if (conn) {
      if (g_asleep) {  // a late page landed while asleep: adopt the link
        g_asleep = false;
        transport.setHidden(false);
      }
      g_search_deadline_us = 0;
    } else if (g_was_connected && !g_asleep) {
      g_search_deadline_us = now_us() + kSearchWindowUs;
      JCLOG("[bt] link lost, searching for host (%d s)\n",
            (int)(kSearchWindowUs / 1000000));
    }
    g_was_connected = conn;
    // device-initiated page-back: only inside an active search window
    if (!conn && !g_asleep) {
      if (transport.pagingStale(now_us())) {
        transport.clearStalePage();  // failed page emits no event
        JCLOG("[cmd] page stale, cleared\n");
      }
      if (transport.haveHost() &&
          now_us() - g_last_reconnect_us > 3000000) {
        g_last_reconnect_us = now_us();
        JCLOG("[cmd] page retry ok=%d\n", transport.reconnect());
      }
      if (g_search_deadline_us && now_us() >= g_search_deadline_us) {
        JCLOG("[bt] no host in %d s, back to standby (any button wakes)\n",
              (int)(kSearchWindowUs / 1000000));
        sleepNow();
      }
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(15));
  }
}

static void printHelp() {
  printf("[cmd] bp <btn> / br <btn>  press/release A B X Y SL SR R ZR Plus RStick Home (press wakes)\n");
  printf("[cmd] unpair           wipe host memory + bonds, reboot to pairing\n");
  printf("[cmd]                  (long-press the BOOT/sync key does the same)\n");
  printf("[cmd] rst               sleep: drop the link, radio silent until a button press\n");
  printf("[cmd] gr                gyro orientation reset to face-up rest\n");
  printf("[cmd] tl / tr           twist left / right (two-way)\n");
  printf("[cmd] rot <0..2> <1|-1> rotate 90 deg around body axis X/Y/Z\n");
  printf("[cmd] t [deg] [out_ms]  one calibrated two-way twist\n");
  printf("[cmd] y <0..2> <1|-1>   yaw gyro axis / sign (Joy-Con R flip)\n");
  printf("[cmd] s <float>         gyro scale dps/lsb (0.06103 or 0.07)\n");
  printf("[cmd] m <0|1>           synthetic motion off/on\n");
  printf("[cmd] r <0|1>           repeat twist every 1.5 s\n");
  printf("[cmd] kb30 <b4> [b5]    raw 0x30 button bytes (escape hatch)\n");
  printf("[cmd] sq <-100..100>    ring strain level (+=squeeze, -=pull)\n");
  printf("[cmd] sqp               ring push-once (squeeze-hold-release)\n");
  printf("[cmd] p                 show twist curve parameters\n");
  printf("[cmd] i                 status\n");
}

static void printStatus() {
  printf("[st] bt: connected=%d mode=0x%02x imu=%d reports=%u host=%d awake=%d\n",
                transport.connected(), transport.reportMode(),
                transport.imuEnabled(), (unsigned)g_reports,
                transport.haveHost(), g_asleep ? 0 : 1);
  printf("[st] orient: w=%.2f x=%.2f y=%.2f z=%.2f\n",
                motion.orient()[0], motion.orient()[1], motion.orient()[2],
                motion.orient()[3]);
  printf("[st] motion: on=%d repeat=%d active=%d prog=%.2f last_w=%.1f dps\n",
                g_motion_on, g_repeat, motion.active(), motion.progress(),
                motion.lastOmegaDps());
  printf("[st] strain: raw=%u level=%d poll=%d\n",
                strain.raw(), strain.level(),
                transport.extdevPolling() ? 1 : 0);
  printf("[st] cfg: yaw_axis=%u scale=%.5f\n",
                motion.cfg.yaw_axis, transport.gyroScale());
}

static void handleCmd(const char* line) {
  char buf[64];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  char* save = nullptr;
  char* tok = strtok_r(buf, " \t", &save);
  if (!tok) return;
  // NOTE: no blanket wake() here - only button commands wake the device, so
  // the frontend's 3 s status poll cannot keep it awake against its will.
  if (!strcmp(tok, "h")) {
    printHelp();
  } else if (!strcmp(tok, "t")) {
    float deg = motion.cfg.out_angle_deg;
    long ms = (long)motion.cfg.out_ms;
    if ((tok = strtok_r(nullptr, " \t", &save))) deg = atof(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) ms = atol(tok);
    if (deg > 0.f && ms > 50) {  // apply to the curve, then fire once
      motion.cfg.out_angle_deg = deg;
      motion.cfg.out_ms = (uint32_t)ms;
    }
    motion.armTwist();
    printf("[cmd] twist: out %.1f deg / %u ms, peak %.0f dps\n",
                  motion.cfg.out_angle_deg, (unsigned)motion.cfg.out_ms,
                  1.5f * motion.cfg.out_angle_deg /
                      (motion.cfg.out_ms / 1000.f));
  } else if (!strcmp(tok, "y")) {
    long axis = 1, sign = 1;
    if ((tok = strtok_r(nullptr, " \t", &save))) axis = atol(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) sign = atol(tok);
    if (axis < 0 || axis > 2 || (sign != 1 && sign != -1)) {
      printf("[cmd] usage: y <0..2> <1|-1>\n");
    } else {
      motion.cfg.yaw_axis = (uint8_t)axis;
      motion.cfg.yaw_sign = (float)sign;
      printf("[cmd] yaw axis=%u sign=%ld\n", (unsigned)axis, sign);
    }
  } else if (!strcmp(tok, "s")) {
    if ((tok = strtok_r(nullptr, " \t", &save)) && atof(tok) > 0) {
      transport.setGyroScale(atof(tok));
      printf("[cmd] gyro scale %.5f dps/lsb\n", transport.gyroScale());
    } else {
      printf("[cmd] usage: s <dps_per_lsb>\n");
    }
  } else if (!strcmp(tok, "m")) {
    if ((tok = strtok_r(nullptr, " \t", &save))) g_motion_on = atoi(tok) != 0;
    printf("[cmd] motion %s\n", g_motion_on ? "on" : "off");
  } else if (!strcmp(tok, "r")) {
    if ((tok = strtok_r(nullptr, " \t", &save))) g_repeat = atoi(tok) != 0;
    g_next_repeat_us = 0;
    printf("[cmd] repeat %s\n", g_repeat ? "on" : "off");
  } else if (!strcmp(tok, "bp") || !strcmp(tok, "br")) {
    // Abstract press/release: routed to the live report mode's bit layout.
    bool down = tok[1] == 'p';
    if (down) wake();  // a button press is what wakes a sleeping Joy-Con
    if (!(tok = strtok_r(nullptr, " \t", &save))) {
      printf("[cmd] usage: bp <A|B|X|Y|SL|SR|R|ZR|Plus|RStick|Home>\n");
    } else {
      int idx = -1;
      for (unsigned i = 0; i < sizeof(kBtnRoutes) / sizeof(kBtnRoutes[0]); i++)
        if (!strcmp(tok, kBtnRoutes[i].name)) { idx = (int)i; break; }
      if (idx < 0) {
        printf("[cmd] unknown button '%s'\n", tok);
      } else {
        const BtnRoute& r = kBtnRoutes[idx];
        if (transport.reportMode() == 0x30) {
          if (r.full_byte == 4) setBit(g_30_b4, r.full_bit, down);
          else setBit(g_30_b5, r.full_bit, down);
        } else {
          if (r.f_byte == 1) setBit(g_kb1, r.f_bit, down);
          else setBit(g_kb2, r.f_bit, down);
        }
        printf("[cmd] %s %s via 0x%02X%s\n", tok, down ? "down" : "up",
                      transport.reportMode(), transport.connected() ? "" : " (queued, not connected)");
      }
    }
  } else if (!strcmp(tok, "unpair")) {  // sync long-press equivalent
    printf("[cmd] unpair: wiping pairing memory, rebooting\n");
    wipePairingAndReboot();  // never returns
  } else if (!strcmp(tok, "rst")) {  // real-JoyCon sleep: vanish from the air
    transport.disconnect();
    sleepNow();
    printf("[cmd] sleeping (radio silent; press any button to wake)\n");
  } else if (!strcmp(tok, "gr")) {  // smooth orientation return to face-up
    motion.armReset();
    printf("[cmd] orientation reset\n");
  } else if (!strcmp(tok, "tl") || !strcmp(tok, "tr")) {
    motion.armTwist(tok[1] == 'l' ? 1 : -1);
    printf("[cmd] twist %s\n", tok[1] == 'l' ? "left(+)" : "right(-)");
  } else if (!strcmp(tok, "rot")) {  // 90 deg one-way body-axis rotation
    long axis = 0, sign = 1;
    if ((tok = strtok_r(nullptr, " \t", &save))) axis = atol(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) sign = atol(tok);
    if (axis < 0 || axis > 2 || (sign != 1 && sign != -1)) {
      printf("[cmd] usage: rot <0..2> <1|-1>\n");
    } else {
      motion.armRot((uint8_t)axis, (float)sign);
      printf("[cmd] rot axis=%ld sign=%ld\n", axis, sign);
    }
  } else if (!strcmp(tok, "kb")) {  // 0x3F simple-mode button bytes (hex)
    wake();  // raw button setters count as key presses too
    long v1 = g_kb1, v2 = g_kb2;
    if ((tok = strtok_r(nullptr, " \t", &save))) v1 = strtol(tok, nullptr, 16);
    if ((tok = strtok_r(nullptr, " \t", &save))) v2 = strtol(tok, nullptr, 16);
    g_kb1 = (uint8_t)v1;
    g_kb2 = (uint8_t)v2;
    printf("[cmd] 3F btn b1=%02X b2=%02X\n", g_kb1, g_kb2);
  } else if (!strcmp(tok, "kb30")) {  // 0x30 full-mode button bytes 4/5 (hex)
    wake();
    long v1 = g_30_b4, v2 = g_30_b5;
    if ((tok = strtok_r(nullptr, " \t", &save))) v1 = strtol(tok, nullptr, 16);
    if ((tok = strtok_r(nullptr, " \t", &save))) v2 = strtol(tok, nullptr, 16);
    g_30_b4 = (uint8_t)v1;
    g_30_b5 = (uint8_t)v2;
    printf("[cmd] 30 btn b4=%02X b5=%02X\n", g_30_b4, g_30_b5);
  } else if (!strcmp(tok, "sq")) {  // Ring-Con strain level, -100..100
    long lv = 0;
    if ((tok = strtok_r(nullptr, " \t", &save))) lv = atol(tok);
    if (lv < -100 || lv > 100) {
      printf("[cmd] usage: sq <-100..100>  (+=squeeze, -=pull)\n");
    } else {
      strain.setLevel((int)lv);
      printf("[cmd] strain level %ld -> raw %u\n", lv, strain.raw());
    }
  } else if (!strcmp(tok, "sqp")) {  // push-once: squeeze-hold-release
    strain.armPush();
    printf("[cmd] ring push-once (rest %u -> press %u, hold %u ms)\n",
                  strain.cfg.rest_raw, strain.cfg.press_raw,
                  (unsigned)strain.cfg.hold_ms);
  } else if (!strcmp(tok, "p")) {
    printf("[cfg] out=%.1f deg/%u ms (peak %.0f dps) hold=%u ms\n",
                  motion.cfg.out_angle_deg, (unsigned)motion.cfg.out_ms,
                  1.5f * motion.cfg.out_angle_deg /
                      (motion.cfg.out_ms / 1000.f),
                  (unsigned)motion.cfg.hold_ms);
    printf("[cfg] back=%u ms overshoot=%.1f deg settle=%u ms coupling=%.2f\n",
                  (unsigned)motion.cfg.back_ms, motion.cfg.overshoot_deg,
                  (unsigned)motion.cfg.settle_ms, motion.cfg.coupling_ratio);
    printf("[cfg] yaw_axis=%u sign=%.0f noise=%.1f dps\n",
                  motion.cfg.yaw_axis, motion.cfg.yaw_sign,
                  motion.cfg.gyro_noise_dps);
  } else if (!strcmp(tok, "i")) {
    printStatus();
  } else {
    printf("[cmd] unknown, 'h' for help\n");
  }
}

static void cliTask(void*) {
  uart_config_t uc = {};
  uc.baud_rate = 115200;
  uc.data_bits = UART_DATA_8_BITS;
  uc.parity = UART_PARITY_DISABLE;
  uc.stop_bits = UART_STOP_BITS_1;
  uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_param_config(UART_NUM_0, &uc);
  uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);

  char line[64];
  size_t len = 0;
  uint8_t c;
  for (;;) {
    while (uart_read_bytes(UART_NUM_0, &c, 1, 0) == 1) {
      if (c == '\r') continue;
      if (c == '\n') {
        if (len) {
          line[len] = 0;
          handleCmd(line);
          len = 0;
        }
      } else if (len < sizeof(line) - 1) {
        line[len++] = (char)c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

extern "C" void app_main(void) {
  JCLOG("[ringcon] Joy-Con R classic BT emulator (IDF)\n");
  gpio_config_t io = {};  // BOOT key as a general input (physical A fallback)
  io.pin_bit_mask = 1ULL << GPIO_NUM_0;
  io.mode = GPIO_MODE_INPUT;
  io.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io);
  transport.begin("Joy-Con (R)", 0x02);
  if (transport.haveHost()) {
    sleepNow();  // bonded: boot radio-silent like a sleeping Joy-Con
    JCLOG("[bt] standby (press any button to wake)\n");
  }
  xTaskCreatePinnedToCore(reportTask, "rpt30", 4096, nullptr, 5, nullptr, 0);
  xTaskCreate(cliTask, "cli", 4096, nullptr, 3, nullptr);
  printHelp();
}
