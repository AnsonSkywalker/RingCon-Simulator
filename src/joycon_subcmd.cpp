// AIGC note: joycon_subcmd.cpp - sub-command dispatch, transport independent.
// SPI flash answers come from the SpiFlash emulation (joycontrol-style default
// calibration tables, no real dump needed).
#include "joycon_subcmd.h"
#include "jc_log.h"
#include <string.h>

namespace joycon {

namespace {

uint8_t ackFor(uint8_t sub) {
  switch (sub) {
    case 0x02: return 0x82;   // REQUEST_DEVICE_INFO
    case 0x04: return 0x83;   // TRIGGER_BUTTONS_ELAPSED_TIME
    case 0x10: return 0x90;   // SPI_FLASH_READ
    case 0x21: return 0xA0;   // SET_NFC_IR_MCU_CONFIG
    default:   return 0x80;
  }
}

// Replies with 34 data bytes; the real one carries MCU state, zero-fill is
// enough for the handshake (joycontrol replies with zeros as well).
const size_t kSpiReplyMax = 0x1D;

}  // namespace

size_t dispatchOutputReport(const uint8_t* v, size_t n, SubCmdState& st,
                            const ReportPacker& packer, uint8_t* out51) {
  // Switch 2 sends fixed 48-byte output reports WITHOUT a report-id byte
  // (on-device verified 2026-09-08): [counter][rumble*8][subcmd][args..].
  // The old heuristic stripped a leading 0x01/0x10/0x11/0x12 as a "report
  // id", which on Switch 2 corrupted every frame whose counter byte hit
  // those values - the host then got a garbage reply and retried forever.
  // Only try the id-strip for non-48-byte (Switch 1 style) frames.
  const uint8_t* p = v;
  if (n != 48 && n > 0 &&
      (p[0] == 0x01 || p[0] == 0x10 || p[0] == 0x11 || p[0] == 0x12)) {
    p++;
    n--;
  }
  if (n < 10) {  // counter(1) + rumble(8) + subcmd id(1)
    JCLOG("[sub] short output report (%u bytes)\n", (unsigned)n);
    return 0;
  }
  uint8_t sub = p[9];
  const uint8_t* arg = n > 10 ? p + 10 : nullptr;
  size_t argn = n > 10 ? n - 10 : 0;
  JCLOG("[sub] 0x%02x (args %u)\n", sub, (unsigned)argn);

  uint8_t reply[kSpiReplyMax] = {0};
  size_t len = 0;
  switch (sub) {
    case 0x02: {  // REQUEST_DEVICE_INFO: fw(2) type(1) 0x02 mac(6) 0x01 0x01
      uint8_t r[12] = {0x03, 0x48, st.joycon_type, 0x02, 0, 0, 0, 0, 0, 0, 0x01, 0x01};
      for (int i = 0; i < 6; i++) r[4 + i] = st.mac[5 - i];  // big-endian out
      len = sizeof(r);
      memcpy(reply, r, len);
      break;
    }
    case 0x03:  // SET_INPUT_REPORT_MODE
      if (argn >= 1) st.report_mode = arg[0];
      break;
    case 0x04: {  // TRIGGER_BUTTONS_ELAPSED_TIME: 7 x uint16 zeros
      len = 14;
      memset(reply, 0, len);
      break;
    }
    case 0x10: {  // SPI_FLASH_READ: addr u32le + size u8, echo addr+size+data
      if (argn >= 5) {
        uint32_t addr = (uint32_t)arg[0] | ((uint32_t)arg[1] << 8) |
                        ((uint32_t)arg[2] << 16) | ((uint32_t)arg[3] << 24);
        uint8_t size = arg[4] > 0x1D ? 0x1D : arg[4];
        uint8_t r[4 + 1 + 0x1D];
        memcpy(r, &addr, 4);
        r[4] = size;
        SpiFlash flash;
        flash.read(addr, r + 5, size);  // fills 0xFF for unmapped regions
        len = 5 + size;
        memcpy(reply, r, len);
        break;
      }
      break;
    }
    case 0x40:  // ENABLE_6AXIS_SENSOR
      st.imu_enabled = argn >= 1 && arg[0] != 0;
      JCLOG("[sub] 6-axis %s\n", st.imu_enabled ? "on" : "off");
      break;
    case 0x08:   // SET_SHIPMENT_STATE
    case 0x21:   // SET_NFC_IR_MCU_CONFIG
    case 0x22:   // SET_NFC_IR_MCU_STATE
    case 0x30:   // SET_PLAYER_LIGHTS
    case 0x48:   // ENABLE_VIBRATION (no rumble motor here)
    default:     // unknown sub-command: ack so the handshake never stalls
      break;
  }

  uint8_t ack = ackFor(sub);
  ReportState idle;  // static buttons/sticks, IMU zeroed in 0x21 replies
  static uint8_t s_timer = 1;  // 0x21 timer: free-running like the real thing
  return packer.pack0x21(out51, idle, s_timer++, ack, sub,
                         {reply + 0, reply + len});
}

}  // namespace joycon
