// AIGC note: jc_log - printf-style logging that works in both the Arduino and
// ESP-IDF builds of the shared protocol/transport files. Keyed on JC_ARDUINO
// (set in platformio.ini), not on ARDUINO: the espidf builder defines that one
// too, even without the Arduino framework present.
#pragma once

#ifdef JC_ARDUINO
#include <Arduino.h>
#define JCLOG(...) Serial.printf(__VA_ARGS__)
#else
#include <stdio.h>
#define JCLOG(...) printf(__VA_ARGS__)
#endif
