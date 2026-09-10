// AIGC note: joycon_subcmd.cpp - sub-command dispatch, transport independent.
// SPI flash answers come from the SpiFlash emulation (real JC(R) factory dump
// via probe31 v4 where probed, joycontrol-style defaults elsewhere).
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
    case 0x59: return 0xD9;   // GET_EXT_DEV_INFO - real JC ack byte is 0x80|0x59
                              // (probe31 v3/v4), not the generic 0x80
    default:   return 0x80;
  }
}

// pack0x21 carries up to 35 reply bytes (buf[16..50]); the SPI-read echo
// needs 5 + 0x1D. (The old cap of 29 overflowed by up to 5 bytes on the
// 25-byte color read the Switch 2 handshake performs.)
const size_t kSpiReplyMax = 35;

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
                  // fw 0x4803 = invented NS2-era version; the real JC(R)'s
                  // 0x2004 was tried (test #4) and is on the revert list with
                  // the serial - identity mirrors confuse the NS2, 0x4803 is
                  // the value that reliably reached the mode-0 stage
      uint8_t r[12] = {0x03, 0x48, st.joycon_type, 0x02, 0, 0, 0, 0, 0, 0, 0x01, 0x01};
      for (int i = 0; i < 6; i++) r[4 + i] = st.mac[5 - i];  // big-endian out
      len = sizeof(r);
      memcpy(reply, r, len);
      break;
    }
    case 0x03:  // SET_INPUT_REPORT_MODE
      if (argn >= 1) st.report_mode = arg[0];
      break;
    case 0x04: {  // TRIGGER_BUTTONS_ELAPSED_TIME: 7 x uint16, DYNAMIC on a
                  // real JC (v5 probe: ff-ff slots after button fiddling;
                  // 2026-09-10 evening replay: all zeros on a fresh state).
                  // Zeros = the clean "no recent presses" handshake value.
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
    case 0x59: {  // GET_EXT_DEV_INFO - THE ring-detection point. Reply data:
                  // [0]=status (0x00 with accessory attached, 0xFE without),
                  // [1]=ext device id 0x20 = Ring-Con. Checkers verified in
                  // refs/ringcon/connectRingCon.ts (data[15]==0x20) and
                  // ringrunnermg joycon.hpp (buf[16]==0x20, buf[15]=0xFE
                  // "no ringcon" / 0x00 with ring). Ground truth for the full
                  // real reply body comes from tools/ring_probe.py.
      // Real JC(R) reply body (probe31 v3/v4): 00 20 00 00 - status 0x00 =
      // accessory attached, ext id 0x20 = Ring-Con, then 2 zero bytes.
      uint8_t r[4] = {0x00, 0x20, 0x00, 0x00};
      len = sizeof(r);
      memcpy(reply, r, len);
      break;
    }
    case 0x5A:  // EXT_DEV_POLLING_ENABLE: strain starts riding the 0x30 stream
      st.extdev_polling = true;
      JCLOG("[sub] extdev polling on\n");
      break;
    case 0x5B:  // EXT_DEV_POLLING_DISABLE
      st.extdev_polling = false;
      JCLOG("[sub] extdev polling off\n");
      break;
    case 0x22:  // SET_NFC_IR_MCU_STATE: 0x00 = suspend -> strain flow stops;
                // 0x01 = resume -> arms the fresh-resume marker the first
                // 0x21 MCU-config ack reports as body[2]=0xFF
      if (argn >= 1 && arg[0] == 0)
        st.extdev_polling = false;
      else if (argn >= 1 && arg[0] == 0x01)
        st.mcu_resume_fresh = true;
      break;
    case 0x5C:  // EXT_DEV_IN_FORMAT_CONFIG: embeds ExtDev data in 0x30; we
                // always use the reference layout (3rd frame accel block),
                // args visible in the [rx] log if the game ever differs
    case 0x08:   // SET_SHIPMENT_STATE
    case 0x30:   // SET_PLAYER_LIGHTS
    case 0x48:   // ENABLE_VIBRATION (no rumble motor here)
    case 0x21:   // SET_NFC_IR_MCU_CONFIG - the MCU path the NS2 game uses
                 // for ring detection (2026-09-09 console capture): payload
                 // [0x21][subcmd][mode][...]. Observed flow: [.,0x00,0x00]
                 // reset on reconnect, [.,0x00,0x03] Ring-Con mode at the
                 // game's ring check, [.,0x01,0x01...] external ready.
                 // Real-device ack (tools/ring_probe31_log.txt): 8 bytes
                 // {01 00 00 00 09 00 20 <mode>} where <mode> is the register
                 // value BEFORE the command applies - a mode-3 set acks with
                 // 0x01 and only the NEXT command's ack carries 0x03 (the
                 // external-ready ack then reads 01 00 00 00 09 00 20 03).
                 // ringrunnermg's host-side while-loop checks rely on this.
                 // Exception, probe31 v3 2026-09-10: body[2] carries the
                 // fresh-resume marker 0xFF on the FIRST config ack after a
                 // 0x22 resume, 0x00 afterwards - and the NS2 game rejects
                 // a mode-0 ack lacking it (see the arg[2]==0 branch).
                 // Deep byte r[46] varies per capture (0x6E/0x49/0xEE) -
                 // state noise, emitted as 0.
      if (sub == 0x21 && argn >= 3 && arg[0] == 0x21) {
        uint8_t pre = st.mcu_mode;
        uint8_t fresh = st.mcu_resume_fresh;
        st.mcu_resume_fresh = false;  // consumed by the first config ack
        // console-capture aid: which MCU command + the args-CRC the game
        // actually sends (mode-0 carries crc=0x00 = crc8 over 36 zero bytes)
        JCLOG("[sub] 0x21 mcu cmd=0x%02x mode=0x%02x pre=0x%02x crc=0x%02x\n",
              arg[1], arg[2], pre, argn >= 39 ? arg[38] : 0);
        if (arg[1] == 0x00) {            // set MCU mode
          if (arg[2] == 0x00) {
            // suspend/reset: the mode register is NOT touched (probe31 v3
            // ground truth: real body[7] stable across two mode-0s AND the
            // following mode-3 ack; the VALUE itself is session-dependent -
            // 0x01 virgin, 0x06 with a parked ExtDev session, see subcmd.h);
            // only the ExtDev data path goes down
            st.extdev_polling = false;
          } else {
            st.mcu_mode = arg[2];
            if (st.mcu_mode != 0x03)     // leaving Ring-Con mode resets the
              st.extdev_polling = false; // ExtDev data path (5C/5A state)
          }
        } else if (arg[1] == 0x01 && arg[2] == 0x01) {
          // external ready: engages the ExtDev data path, mode unchanged
        }
        // Running-MCU status body, byte-matched to probe31 v3 real device:
        // mode-0 #1 (first config ack after 0x22 resume) = 01 00 FF 00 09
        // 00 20 01, later acks carry 00 in body[2]. NS2 captures 2026-09-10:
        // a mode-0 ack with body[2]=00 (pre-state body) is REJECTED - the
        // game retries it then drops the link ~12s later; an all-zero body
        // fares the same. What always passed is the FF body (the old
        // firmware's kMcuCfgAck carried it for every MCU config).
        reply[0] = 0x01;                 // status
        reply[2] = fresh ? 0xFF : 0x00;  // fresh-resume marker
        reply[4] = 0x09;                 // fixed field (all captures)
        reply[6] = 0x20;                 // ext device id (Ring-Con)
        reply[7] = pre;                  // mode before this command
        len = 8;
      }
      break;
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
