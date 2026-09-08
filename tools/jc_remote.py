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
import tkinter as tk

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200

# Bit maps (dekuNukem simple-HID + full report layouts, sideways right
# Joy-Con: physical face buttons map to the d-pad bits in 0x3F byte 1).
MODES = {
    "0x3F": {
        "A(Down)": (1, 0x01), "X(Right)": (1, 0x02), "B(Left)": (1, 0x04),
        "Y(Up)": (1, 0x08), "SL": (1, 0x10), "SR": (1, 0x20),
        "Minus": (2, 0x01), "Plus": (2, 0x02), "Home": (2, 0x10),
        "Capture": (2, 0x20), "R": (2, 0x40), "ZR": (2, 0x80),
    },
    "0x30": {
        "A": (4, 0x08), "X": (4, 0x02), "B": (4, 0x04), "Y": (4, 0x01),
        "SL": (4, 0x10), "SR": (4, 0x20),
        "Minus": (5, 0x01), "Plus": (5, 0x02), "Home": (5, 0x10),
        "Capture": (5, 0x20), "R": (5, 0x40), "ZR": (5, 0x80),
    },
}
# state[mode][byte] = accumulated bits for that report byte
state = {"0x3F": {1: 0, 2: 0}, "0x30": {4: 0, 5: 0}}
mode = "0x3F"

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.setDTR(False)  # keep the board out of download/reset states
ser.setRTS(False)
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
    ["", "Y(Up)", "", ""],
    ["X(Right)", "", "A(Down)", ""],
    ["", "B(Left)", "", ""],
    ["SL", "", "SR", ""],
    ["Minus", "Home", "Plus", "Capture"],
    ["R", "ZR", "", ""],
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

tk.Label(root, text="Latch on: click = toggle (hold several at once).\n"
                    "Latch off: mouse-hold = button held.\n"
                    "If the screen ignores A(Down), try X(Right)/B(Left)/Y(Up):\n"
                    "the sideways face-button mapping is not verified yet.\n"
                    "Raw cmd examples: t 100 360 | y 1 -1 | s 0.07 | kb 30",
         justify=tk.LEFT).pack(pady=4)

send()
root.mainloop()
