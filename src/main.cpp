// AIGC note: firmware entry point - BLE Joy-Con R emulator with a synthetic
// "turn left 90" squat-twist motion generator, tuned over serial.
// 15 ms FreeRTOS task packs three 5 ms motion frames into each 0x30 report
// (66 Hz, matching the real Joy-Con report cadence, research report R5).
#include <Arduino.h>
#include "joycon_ble.h"
#include "motion.h"
#include "mpu.h"

static joycon::JoyConBle ble;
static joycon::TwistMotion motion;
static joycon::MpuSource mpu;
static bool g_use_real_imu = false;   // 'im' command: real MPU vs synthetic curve
static uint8_t g_timer = 0;
static bool g_motion_on = true;       // 'm' command: synthetic motion vs still
static bool g_repeat = false;         // 'r' command: twist every 1.5 s
static uint32_t g_next_repeat_ms = 0;
static uint32_t g_reports = 0;

static void reportTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    if (ble.connected() && ble.subscribed() && ble.reportMode() == 0x30) {
      joycon::ReportState frames[3];
      for (int i = 0; i < 3; i++) {
        frames[i] = joycon::ReportState{};  // resting pose, zero gyro
        if (g_use_real_imu && mpu.detected()) {
          mpu.sample(frames[i]);
        } else if (g_motion_on) {
          motion.tick(5, frames[i]);
        }
      }
      ble.notify30(frames, g_timer++, ble.imuEnabled());
      g_reports++;
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(15));
  }
}

static void printHelp() {
  Serial.println("[cmd] t [deg] [ms]  arm twist (default 90 500 = turn left 90)");
  Serial.println("[cmd] y <0..2> <1|-1>   yaw gyro axis / sign (Joy-Con R flip)");
  Serial.println("[cmd] s <float>     gyro scale dps/lsb (0.06103 or 0.07)");
  Serial.println("[cmd] m <0|1>       synthetic motion off/on");
  Serial.println("[cmd] im <0|1>      imu source: synthetic / real MPU6050");
  Serial.println("[cmd] mg|ma <jc> <m> <sign>  gyro/acc axis map (Joy-Con<-MPU)");
  Serial.println("[cmd] r <0|1>       repeat twist every 1.5 s");
  Serial.println("[cmd] i             status");
}

static void printStatus() {
  Serial.printf("[st] ble: connected=%d subscribed=%d mode=0x%02x imu=%d\n",
                ble.connected(), ble.subscribed(), ble.reportMode(),
                ble.imuEnabled());
  Serial.printf("[st] motion: on=%d repeat=%d active=%d prog=%.2f last_w=%.1f dps\n",
                g_motion_on, g_repeat, motion.active(), motion.progress(),
                motion.lastOmegaDps());
  Serial.printf("[st] imu: mode=%s mpu_detected=%d reports=%u\n",
                g_use_real_imu ? "real" : "synthetic", mpu.detected(),
                (unsigned)g_reports);
  Serial.printf("[st] cfg: yaw_axis=%u scale=%.5f\n",
                motion.cfg.yaw_axis, ble.gyroScale());
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
    float deg = 90.f;
    uint32_t ms = 500;
    if ((tok = strtok_r(nullptr, " \t", &save))) deg = atof(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) ms = (uint32_t)atol(tok);
    motion.arm(deg, ms);
    Serial.printf("[cmd] twist %.1f deg over %u ms\n", deg, (unsigned)ms);
  } else if (!strcmp(tok, "y")) {
    long axis = 2, sign = 1;
    if ((tok = strtok_r(nullptr, " \t", &save))) axis = atol(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) sign = atol(tok);
    if (axis < 0 || axis > 2 || (sign != 1 && sign != -1)) {
      Serial.println("[cmd] usage: y <0..2> <1|-1>");
    } else {
      motion.cfg.yaw_axis = (uint8_t)axis;
      motion.cfg.yaw_sign = (float)sign;
      Serial.printf("[cmd] yaw axis=%u sign=%ld\n", (unsigned)axis, sign);
    }
  } else if (!strcmp(tok, "s")) {
    if ((tok = strtok_r(nullptr, " \t", &save)) && atof(tok) > 0) {
      ble.setGyroScale(atof(tok));
      Serial.printf("[cmd] gyro scale %.5f dps/lsb\n", ble.gyroScale());
    } else {
      Serial.println("[cmd] usage: s <dps_per_lsb>");
    }
  } else if (!strcmp(tok, "m")) {
    if ((tok = strtok_r(nullptr, " \t", &save))) g_motion_on = atoi(tok) != 0;
    Serial.printf("[cmd] motion %s\n", g_motion_on ? "on" : "off");
  } else if (!strcmp(tok, "r")) {
    if ((tok = strtok_r(nullptr, " \t", &save))) g_repeat = atoi(tok) != 0;
    g_next_repeat_ms = 0;
    Serial.printf("[cmd] repeat %s\n", g_repeat ? "on" : "off");
  } else if (!strcmp(tok, "im")) {  // imu source: 0 synthetic, 1 real MPU
    if ((tok = strtok_r(nullptr, " \t", &save))) {
      g_use_real_imu = atoi(tok) != 0;
    }
    if (g_use_real_imu && !mpu.detected()) {
      Serial.println("[cmd] MPU6050 not detected, staying synthetic");
      g_use_real_imu = false;
    } else {
      Serial.printf("[cmd] imu source: %s\n", g_use_real_imu ? "real mpu" : "synthetic");
    }
  } else if (!strcmp(tok, "mg") || !strcmp(tok, "ma")) {  // axis map: <jc> <mpu> <sign>
    bool is_gyro = tok[1] == 'g';
    long jc = 0, m = 0, sg = 1;
    if ((tok = strtok_r(nullptr, " \t", &save))) jc = atol(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) m = atol(tok);
    if ((tok = strtok_r(nullptr, " \t", &save))) sg = atol(tok);
    if (jc < 0 || jc > 2 || m < 0 || m > 2 || (sg != 1 && sg != -1)) {
      Serial.println("[cmd] usage: mg|ma <jc_axis 0..2> <mpu_axis 0..2> <1|-1>");
    } else {
      if (is_gyro) {
        mpu.cfg.gyro_map[jc] = (int8_t)m;
        mpu.cfg.gyro_sign[jc] = (int8_t)sg;
      } else {
        mpu.cfg.acc_map[jc] = (int8_t)m;
        mpu.cfg.acc_sign[jc] = (int8_t)sg;
      }
      Serial.printf("[cmd] %s map jc%ld <- mpu%ld sign=%ld\n",
                    is_gyro ? "gyro" : "acc", jc, m, sg);
    }
  } else if (!strcmp(tok, "i")) {
    printStatus();
  } else {
    Serial.println("[cmd] unknown, 'h' for help");
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[ringcon] Joy-Con R BLE emulator, " __FILE__ " " __DATE__);
  Serial.println("[ringcon] R1 experiment: stock Switch pairing over BLE HOGP");
  mpu.begin(4, 5);  // GY-521: SDA=GPIO4, SCL=GPIO5 (auto-detected, optional)
  g_use_real_imu = mpu.detected();
  ble.begin("Joy-Con (R)", 0x02);
  xTaskCreate(reportTask, "rpt30", 4096, nullptr, 5, nullptr);
  printHelp();
}

void loop() {
  static char line[64];
  static size_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = 0;
      handleCmd(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  if (g_repeat && millis() >= g_next_repeat_ms) {
    motion.arm(90.f, 500);
    g_next_repeat_ms = millis() + 1500;
  }
  delay(5);
}
