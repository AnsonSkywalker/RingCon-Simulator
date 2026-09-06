# AIGC note: ble_selftest - PC-side BLE HID host that exercises the emulator the
# way a Switch HOGP host would: connect, pair/encrypt, read the report map,
# classify report characteristics via Report Reference descriptors, send
# sub-commands on the output report and check the 0x21 replies, then subscribe
# to the 0x30 stream and verify the synthetic twist shows up in the IMU frames.
# Usage: python tools/ble_selftest.py [serial_port]
import asyncio
import struct
import sys

from bleak import BleakScanner, BleakClient

HID_SVC = "00001812-0000-1000-8000-00805f9b34fb"
HID_INFO = "00002a4a-0000-1000-8000-00805f9b34fb"
REPORT_MAP = "00002a4b-0000-1000-8000-00805f9b34fb"
REPORT_CHAR = "00002a4d-0000-1000-8000-00805f9b34fb"
REPORT_REF = "00002908-0000-1000-8000-00805f9b34fb"

results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(("PASS " if ok else "FAIL ") + name + (" :: " + detail if detail else ""), flush=True)


def log(msg):
    print(msg, flush=True)


async def find_jc(timeout=15.0):
    found = asyncio.Event()
    holder = {}

    def cb(d, adv):
        name = adv.local_name or d.name or ""
        if name == "Joy-Con (R)" and "d" not in holder:
            holder["d"] = d
            found.set()

    scanner = BleakScanner(detection_callback=cb)
    await scanner.start()
    try:
        await asyncio.wait_for(found.wait(), timeout=timeout)
    except asyncio.TimeoutError:
        pass
    finally:
        await scanner.stop()
    return holder.get("d")


async def main():
    serial_port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    # Optional fixed address: bypasses discovery entirely. Useful when a bonded
    # Windows host instantly re-connects and eats the advertising window.
    fixed_addr = sys.argv[2] if len(sys.argv) > 2 else None

    if fixed_addr:
        dev = fixed_addr
        check("advertiser visible", True, f"direct address {dev}")
    else:
        dev = await find_jc()
        if dev is None:
            all_devs = await BleakScanner.discover(timeout=4.0)
            names = sorted(d.name for d in all_devs if d.name)
            check("advertiser visible", False,
                  f"not in scan; others: {names[:12]}")
            return 1
        check("advertiser visible", True, dev.address)

    # Opening the USB-Serial-JTAG port resets the board, so open it BEFORE the
    # BLE session and let the firmware come back up advertising.
    serial = None
    try:
        import serial as pyserial
        serial = pyserial.Serial(serial_port, 115200, timeout=1)
        serial.setDTR(False)
        serial.setRTS(False)
        await asyncio.sleep(1.5)  # reboot + re-advertise
    except Exception as e:
        log(f"serial unavailable ({e}); repeat-mode steps will be skipped")

    async with BleakClient(dev) as client:
        log("connected")
        try:
            paired = await client.pair()
            check("pair/encrypt", bool(paired), str(paired))
        except Exception as e:
            check("pair/encrypt", False, repr(e))

        svcs = list(client.services)
        hid = next((s for s in svcs if s.uuid.lower() == HID_SVC), None)
        check("HID service present", hid is not None)
        if hid is None:
            log("services seen: " + ", ".join(s.uuid for s in svcs))
            return 1

        try:
            info = await client.read_gatt_char(HID_INFO)
            check("HID Info readable", len(info) == 4, info.hex())
        except Exception as e:
            check("HID Info readable", False, repr(e))

        rmap = await client.read_gatt_char(REPORT_MAP)
        check("Report Map = 170B Joy-Con descriptor", len(rmap) == 170,
              f"{len(rmap)} bytes")

        inputs, outputs = {}, {}
        for ch in hid.characteristics:
            if ch.uuid != REPORT_CHAR:
                continue
            for d in ch.descriptors:
                val = await client.read_gatt_descriptor(d)
                rid, rtype = val[0], val[1]
                (inputs if rtype == 1 else outputs)[rid] = ch
        check("report chars classified",
              sorted(inputs) == [0x21, 0x30] and sorted(outputs) == [0x01],
              f"input={sorted(inputs)} output={sorted(outputs)}")

        replies = []

        def on_reply(ch, data: bytearray):
            replies.append(bytes(data))

        await client.start_notify(inputs[0x21], on_reply)

        async def subcommand(sub, args=b""):
            frame = bytes([0x01, 0x37]) + bytes(
                [0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40]) + bytes([sub]) + args
            replies.clear()
            await client.write_gatt_char(outputs[0x01], frame, response=True)
            for _ in range(20):
                await asyncio.sleep(0.05)
                for r in replies:
                    if len(r) >= 14 and r[13] == sub:
                        return r
        # HOGP-stripped 0x21 value = classic[2..50]:
        #   idx0 timer, 1 battery, 2..4 buttons, 5..10 sticks, 11 vib,
        #   12 ack, 13 subcmd id, 14.. data
        r = await subcommand(0x02)
        ok = bool(r) and r[12] == 0x82 and r[13] == 0x02 and len(r) == 49
        detail = ""
        if ok:
            fw, jctype, mac = r[14:16], r[16], r[18:24]
            ok = jctype == 0x02 and r[17] == 0x02 and r[24:26] == b"\x01\x01"
            detail = f"fw={fw.hex()} type=0x{jctype:02x} mac={mac.hex()}"
        check("0x02 device info reply", ok, detail)

        r = await subcommand(0x10, struct.pack("<IB", 0x6020, 24))
        want = bytes.fromhex("7600a6feea02")
        ok = bool(r) and r[12] == 0x90 and r[13] == 0x10 \
            and r[14:18] == struct.pack("<IB", 0x6020, 24) and r[19:25] == want
        check("0x10 SPI read IMU cal", ok, r.hex() if r else "no reply")

        r = await subcommand(0x10, struct.pack("<IB", 0x6046, 9))
        ok = bool(r) and r[12] == 0x90 and r[19:22] == bytes.fromhex("000880")
        check("0x10 SPI read R-stick cal", ok, r.hex() if r else "no reply")

        r = await subcommand(0x03, bytes([0x30]))
        check("0x03 input report mode ack", bool(r) and r[12] == 0x80 and r[13] == 0x03)

        r = await subcommand(0x40, bytes([0x01]))
        check("0x40 six-axis enable ack", bool(r) and r[12] == 0x80 and r[13] == 0x40)

        r = await subcommand(0x30, bytes([0x00]))
        check("0x30 player lights ack", bool(r) and r[12] == 0x80 and r[13] == 0x30)

        # 0x30 stream: arm repeat twists via serial, collect ~4s of reports
        if serial is not None:
            serial.write(b"r 1\n")
            await asyncio.sleep(0.3)
            stream = []
            got = asyncio.Event()

            def on_stream(ch, data: bytearray):
                stream.append(bytes(data))
                if len(stream) >= 240:
                    got.set()

            await client.start_notify(inputs[0x30], on_stream)
            try:
                await asyncio.wait_for(got.wait(), timeout=8)
            except asyncio.TimeoutError:
                pass
            serial.write(b"r 0\n")
            check("0x30 stream >=240 reports in 8s (~66Hz)", len(stream) >= 240,
                  f"got {len(stream)}")
            if stream:
                lens = {len(x) for x in stream}
                timers = [x[0] for x in stream]
                mono = all(b >= a for a, b in zip(timers, timers[1:]))
                peak = 0.0
                for rep in stream:
                    for k in range(3):
                        gz = struct.unpack_from("<h", rep, 12 + 12 * k + 10)[0]
                        peak = max(peak, abs(gz) * 0.06103)
                check("0x30 frames 48B, timer monotonic", lens == {48} and mono,
                      f"lens={lens}")
                check("twist visible in gyro stream", 100.0 < peak < 400.0,
                      f"peak |gyro_z|={peak:.0f} dps (expect ~270 for 90deg/500ms)")
        await client.stop_notify(inputs[0x21])
        if serial is not None:
            await client.stop_notify(inputs[0x30])
            serial.close()

    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} checks passed", flush=True)
    if failed:
        print("failed: " + ", ".join(failed), flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
