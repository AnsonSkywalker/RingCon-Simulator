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
#include "esp_timer.h"
#include "jc_log.h"
#include "joycon_btclassic.h"
#include "motion.h"

static joycon::JoyConBtClassic transport;
static joycon::TwistMotion motion;
static uint8_t g_timer = 0;
static bool g_motion_on = true;       // 'm' command: synthetic motion vs still
static bool g_repeat = true;          // 'r' command: twist every 1.5 s
static int64_t g_next_repeat_us = 0;
static uint32_t g_reports = 0;

static int64_t now_us() { return esp_timer_get_time(); }

static void reportTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    if (transport.connected() && transport.reportMode() == 0x30) {
      joycon::ReportState frames[3];
      for (int i = 0; i < 3; i++) {
        frames[i] = joycon::ReportState{};  // resting pose, zero gyro
        if (g_motion_on) motion.tick(5, frames[i]);
      }
      transport.notify30(frames, g_timer++, transport.imuEnabled());
      g_reports++;
    }
    if (g_repeat && !motion.active() && now_us() >= g_next_repeat_us) {
      motion.armTwist();
      g_next_repeat_us = now_us() + 1500000;
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(15));
  }
}

static void printHelp() {
  printf("[cmd] t [deg] [out_ms]  one calibrated two-way twist\n");
  printf("[cmd] y <0..2> <1|-1>   yaw gyro axis / sign (Joy-Con R flip)\n");
  printf("[cmd] s <float>         gyro scale dps/lsb (0.06103 or 0.07)\n");
  printf("[cmd] m <0|1>           synthetic motion off/on\n");
  printf("[cmd] r <0|1>           repeat twist every 1.5 s\n");
  printf("[cmd] p                 show twist curve parameters\n");
  printf("[cmd] i                 status\n");
}

static void printStatus() {
  printf("[st] bt: connected=%d mode=0x%02x imu=%d reports=%u\n",
                transport.connected(), transport.reportMode(),
                transport.imuEnabled(), (unsigned)g_reports);
  printf("[st] motion: on=%d repeat=%d active=%d prog=%.2f last_w=%.1f dps\n",
                g_motion_on, g_repeat, motion.active(), motion.progress(),
                motion.lastOmegaDps());
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
  transport.begin("Joy-Con (R)", 0x02);
  xTaskCreatePinnedToCore(reportTask, "rpt30", 4096, nullptr, 5, nullptr, 0);
  xTaskCreate(cliTask, "cli", 4096, nullptr, 3, nullptr);
  printHelp();
}
