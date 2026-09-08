# AIGC note: jc_remote.py - PC-side remote button panel for the ringcon board.
# Talks to the firmware's serial CLI ('kb' / 'kb30' commands) so the Switch's
# grip/order screen and in-game menus can be driven without physical buttons.
# Latch on (default): click = toggle, so several buttons can be held at once
# (e.g. SL+SR latched, then click A). Latch off: mouse-hold = button held.
#
# Usage:  python jc_remote.py [COM5]
# Requires: pip install pyserial   (tkinter ships with Python on Windows)
#
# NOTE: this script owns the serial port - close it before flashing or
# watching the log with another tool.
import sys
import threading
import time
import tkinter as tk

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200

# Bit maps. 0x30 = full report, raw layout verified against joycontrol's
# ButtonState (controller_state.py, field-tested on Switch 1):
#   byte4: Y X B A SR SL R ZR (bit0..7)   byte5: Minus Plus RStick LStick
#   Home Capture (bit0..5)
# 0x3F = simple HID (pre-pairing grip screen only - once the host switches us
# to 0x30 it ignores 0x3F reports). Sideways right JC maps its face buttons to
# the d-pad bits of byte 1; that mapping is a guess, not yet screen-verified.
MODES = {
    "0x3F": {
        "A": (1, 0x01), "X": (1, 0x02), "B": (1, 0x04), "Y": (1, 0x08),
        "SL": (1, 0x10), "SR": (1, 0x20),
        "Minus": (2, 0x01), "Plus": (2, 0x02), "RStick": (2, 0x04),
        "Home": (2, 0x10), "Capture": (2, 0x20), "R": (2, 0x40), "ZR": (2, 0x80),
    },
    "0x30": {
        "A": (4, 0x08), "X": (4, 0x02), "B": (4, 0x04), "Y": (4, 0x01),
        "SR": (4, 0x10), "SL": (4, 0x20), "R": (4, 0x40), "ZR": (4, 0x80),
        "Minus": (5, 0x01), "Plus": (5, 0x02), "RStick": (5, 0x04),
        "Home": (5, 0x10), "Capture": (5, 0x20),
    },
}
# state[mode][byte] = accumulated bits for that report byte
state = {"0x3F": {1: 0, 2: 0}, "0x30": {4: 0, 5: 0}}
mode = "0x3F"

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.1
ser.dsrdtr = False  # do not toggle DTR on open -> board stays out of reset
ser.rtscts = False
ser.dtr = False
ser.rts = False
ser.open()
ser.reset_input_buffer()

root = tk.Tk()
root.title(f"Joy-Con (R) Remote -> {PORT}")

latch = tk.BooleanVar(value=True)
mode_var = tk.StringVar(value=mode)


def send():
    if mode == "0x3F":
        cmd = f"kb {state[mode][1]:02x} {state[mode][2]:02x}\n"
    else:
        cmd = f"kb30 {state[mode][4]:02x} {state[mode][5]:02x}\n"
    ser.write(cmd.encode())
    bits = " ".join(f"{state[mode][k]:02X}" for k in sorted(state[mode]))
    status.config(text=f"mode {mode} | bytes: {bits} | sent: {cmd.strip()}")


def set_mode(*_):
    global mode
    for k in state[mode]:  # release everything before switching layouts
        state[mode][k] = 0
    mode = mode_var.get()
    send()


def press(name):
    byte_sel, bit = MODES[mode][name]
    if latch.get():
        state[mode][byte_sel] ^= bit  # toggle
    else:
        state[mode][byte_sel] |= bit
    on = bool(state[mode][byte_sel] & bit)
    btns[name].config(relief=tk.SUNKEN if on else tk.RAISED,
                      bg="#9ecbff" if on else btns[name].default_bg)
    send()


def release(name):
    if latch.get():
        return  # latch mode: release happens on the next click
    byte_sel, bit = MODES[mode][name]
    state[mode][byte_sel] &= ~bit
    btns[name].config(relief=tk.RAISED, bg=btns[name].default_bg)
    send()


def clear_all():
    for k in state[mode]:
        state[mode][k] = 0
    for n in btns:
        btns[n].config(relief=tk.RAISED, bg=btns[n].default_bg)
    send()


top = tk.Frame(root)
top.pack(pady=4)
tk.Label(top, text="Report mode:").pack(side=tk.LEFT, padx=4)
tk.OptionMenu(top, mode_var, *MODES.keys(), command=set_mode).pack(side=tk.LEFT)
tk.Checkbutton(top, text="Latch (click = toggle)", variable=latch).pack(
    side=tk.LEFT, padx=8)

grid = tk.Frame(root)
grid.pack(padx=10, pady=10)
layout = [
    ["", "Y", "", ""],
    ["X", "", "A", ""],
    ["", "B", "", ""],
    ["SL", "", "SR", ""],
    ["Minus", "Home", "Plus", "Capture"],
    ["R", "ZR", "RStick", ""],
]
btns = {}
for r, row in enumerate(layout):
    for c, name in enumerate(row):
        if not name:
            tk.Label(grid, width=9).grid(row=r, column=c)
            continue
        b = tk.Button(grid, text=name, width=9)
        b.default_bg = b.cget("bg")
        b.bind("<ButtonPress-1>", lambda e, n=name: press(n))
        b.bind("<ButtonRelease-1>", lambda e, n=name: release(n))
        b.grid(row=r, column=c, padx=2, pady=2)
        btns[name] = b

status = tk.Label(root, text="", font=("Consolas", 11))
status.pack(pady=4)
tk.Button(root, text="Clear All (release everything)", command=clear_all).pack(pady=2)

raw_row = tk.Frame(root)
raw_row.pack(pady=2)
tk.Label(raw_row, text="Raw cmd:").pack(side=tk.LEFT)
raw_entry = tk.Entry(raw_row, width=24)
raw_entry.pack(side=tk.LEFT, padx=4)


def raw_send(*_):
    cmd = raw_entry.get().strip()
    if cmd:
        ser.write((cmd + "\n").encode())
        status.config(text=f"sent: {cmd}")
        raw_entry.delete(0, tk.END)


raw_entry.bind("<Return>", raw_send)
tk.Button(raw_row, text="Send", command=raw_send).pack(side=tk.LEFT)

# ---- motion panel: one-shot firmware commands (work while in 0x30 stream) ----
motion = tk.LabelFrame(root, text="体感 Motion (游戏内 0x30 流)")
motion.pack(fill=tk.X, padx=10, pady=4)


def send_cmd(cmd):
    ser.write((cmd + "\n").encode())
    status.config(text=f"sent: {cmd}")


rot_row = tk.Frame(motion)
rot_row.pack(pady=2)
tk.Label(rot_row, text="轴向旋转90°:").pack(side=tk.LEFT, padx=2)
for label, cmd in [("X+", "rot 0 1"), ("X-", "rot 0 -1"), ("Y+", "rot 1 1"),
                   ("Y-", "rot 1 -1"), ("Z+", "rot 2 1"), ("Z-", "rot 2 -1")]:
    tk.Button(rot_row, text=label, width=4,
              command=lambda c=cmd: send_cmd(c)).pack(side=tk.LEFT, padx=2)

act_row = tk.Frame(motion)
act_row.pack(pady=2)
tk.Button(act_row, text="重置体感 (面朝上平放)", width=18,
          command=lambda: send_cmd("rst")).pack(side=tk.LEFT, padx=4)
tk.Button(act_row, text="左扭腰90°", width=9,
          command=lambda: send_cmd("tl")).pack(side=tk.LEFT, padx=4)
tk.Button(act_row, text="右扭腰90°", width=9,
          command=lambda: send_cmd("tr")).pack(side=tk.LEFT, padx=4)

auto_var = tk.BooleanVar(value=True)


def toggle_auto():
    send_cmd("r 1" if auto_var.get() else "r 0")


tk.Checkbutton(motion, text="每1.5s自动扭腰", variable=auto_var,
               command=toggle_auto).pack(side=tk.LEFT, padx=8)
tk.Label(motion, text="轴向 X/Y/Z = 手柄本体系，方向含义待游戏内确认").pack(
    side=tk.LEFT, padx=4)

# ---- background serial reader: show the last firmware response ----
rx_line = {"text": ""}


def reader():
    buf = b""
    while True:
        try:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    rx_line["text"] = line.decode(errors="replace").strip()
            else:
                time.sleep(0.02)
        except Exception:
            break  # port closed -> frontend is exiting


threading.Thread(target=reader, daemon=True).start()
rx_label = tk.Label(root, text="rx: -", font=("Consolas", 9), fg="#555")
rx_label.pack(anchor=tk.W, padx=12)


def poll_rx():
    if rx_line["text"]:
        rx_label.config(text="rx: " + rx_line["text"])
    root.after(200, poll_rx)


root.after(200, poll_rx)

tk.Label(root, text="0x30 = normal mode (system settings / in-game) - use this.\n"
                    "0x3F = pre-pairing grip screen only; the host ignores 0x3F\n"
                    "reports once it has switched us to 0x30.\n"
                    "Latch on: click = toggle (hold several at once).\n"
                    "Latch off: mouse-hold = button held.\n"
                    "Raw cmd examples: t 100 360 | y 1 -1 | s 0.07 | kb30 08",
         justify=tk.LEFT).pack(pady=4)

send()
root.mainloop()
