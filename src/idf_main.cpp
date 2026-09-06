// AIGC note: idf_main - ESP-IDF entry point for the Bluetooth Classic
// transport (env:esp32classic). Serial CLI from the Arduino build is not
// wired here yet; instead a synthetic 90-degree left twist fires every 2.5 s
// so game-side behavior is testable the moment the board pairs.
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "jc_log.h"
#include "joycon_btclassic.h"
#include "motion.h"

static joycon::JoyConBtClassic transport;
static joycon::TwistMotion motion;
static uint8_t g_timer = 0;
static bool g_motion_on = true;

static void reportTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t ticks = 0;
  for (;;) {
    if (transport.connected() && transport.reportMode() == 0x30) {
      joycon::ReportState frames[3];
      for (int i = 0; i < 3; i++) {
        frames[i] = joycon::ReportState{};  // resting pose, zero gyro
        if (g_motion_on) motion.tick(5, frames[i]);
      }
      transport.notify30(frames, g_timer++, transport.imuEnabled());
    }
    if (++ticks % 166 == 0) motion.arm(90.f, 500);  // ~2.5 s period
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(15));
  }
}

extern "C" void app_main(void) {
  JCLOG("[ringcon] Joy-Con R classic BT emulator (IDF)\n");
  transport.begin("Joy-Con (R)", 0x02);
  xTaskCreatePinnedToCore(reportTask, "rpt30", 4096, nullptr, 5, nullptr, 0);
}
