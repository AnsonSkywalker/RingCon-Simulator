# AIGC note: jc_remote.py - PC-side Joy-Con (R) control panel. All buttons are
# abstract (press A = "bp A"), the firmware routes to the right bit for the
# host's current phase (grip-screen simple-HID vs full mode) so the user never
# thinks about report modes. Latch on (default): click = toggle; hold several
# at once (e.g. latch SL+SR then press A). Latch off: mouse-hold = held.
#
# rst = sleep/unlink (real Joy-Con behavior; Switch icon disappears). gr =
# orientation reset (return synthetic controller to face-up flat). Motion
# buttons (X±/Y±/Z±/tl/tr) are one-shot firmware commands.
#
# Usage:  python jc_remote.py [COM5]
# Requires: pip install pyserial   (tkinter ships with Python on Windows)
import sys
import threading
import time
import tkinter as tk

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200

# Right Joy-Con real buttons. Name -> press command.
BUTTONS = ["A", "B", "X", "Y", "SL", "SR", "R", "ZR", "Plus", "RStick", "Home"]

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.1
ser.dsrdtr = False  # don't pulse DTR -> board keeps its connection across opens
ser.rtscts = False
ser.dtr = False
ser.rts = False
ser.open()
ser.reset_input_buffer()

root = tk.Tk()
root.title(f"Joy-Con (R) Remote -> {PORT}")

latch = tk.BooleanVar(value=True)
held = set()  # currently-latched buttons


def send(cmd):
    ser.write((cmd + "\n").encode())
    status.config(text=f"sent: {cmd}")


def button(name):
    if latch.get():
        if name in held:
            held.discard(name)
            send(f"br {name}")
        else:
            held.add(name)
            send(f"bp {name}")
        btns[name].config(relief=tk.SUNKEN if name in held else tk.RAISED,
                          bg=held_bg if name in held else btns[name].default_bg)
    else:
        send(f"bp {name}")


def release(name):
    if latch.get():
        return
    send(f"br {name}")


def clear_all():
    for name in list(held):
        send(f"br {name}")
    held.clear()
    for n in btns:
        btns[n].config(relief=tk.RAISED, bg=btns[n].default_bg)


# ---- button grid (right Joy-Con face) ----
grid = tk.LabelFrame(root, text="按键 (自动适配配对屏/游戏模式)")
grid.pack(padx=10, pady=6)
btns = {}
for name in BUTTONS:
    b = tk.Button(grid, text=name, width=7)
    b.default_bg = b.cget("bg")
    b.config(command=lambda n=name: button(n))
    btns[name] = b
btns["Y"].configure(width=5); btns["X"].configure(width=5)
btns["A"].configure(width=5); btns["B"].configure(width=5)
# diamond: Y top, X left, A right, B bottom
btns["Y"].grid(row=0, column=1, pady=2)
btns["X"].grid(row=1, column=0, pady=2)
btns["A"].grid(row=1, column=2, pady=2)
btns["B"].grid(row=2, column=1, pady=2)
for i, name in enumerate(["SL", "SR", "R", "ZR", "Plus", "RStick", "Home"]):
    btns[name].grid(row=i, column=4, padx=4, pady=2)

held_bg = "#9ecbff"
ctrl = tk.Frame(root)
ctrl.pack(pady=2)
tk.Checkbutton(ctrl, text="Latch (click = toggle)", variable=latch).pack(side=tk.LEFT, padx=8)
tk.Button(ctrl, text="清空", command=clear_all).pack(side=tk.LEFT, padx=4)
tk.Button(ctrl, text="休眠/断连 (rst)", width=16,
          command=lambda: send("rst")).pack(side=tk.LEFT, padx=4)

# ---- motion ----
motion = tk.LabelFrame(root, text="体感")
motion.pack(fill=tk.X, padx=10, pady=4)
rot_row = tk.Frame(motion)
rot_row.pack(pady=2)
tk.Label(rot_row, text="90°:").pack(side=tk.LEFT)
for label, cmd in [("X+", "rot 0 1"), ("X-", "rot 0 -1"), ("Y+", "rot 1 1"),
                   ("Y-", "rot 1 -1"), ("Z+", "rot 2 1"), ("Z-", "rot 2 -1")]:
    tk.Button(rot_row, text=label, width=4,
              command=lambda c=cmd: send(c)).pack(side=tk.LEFT, padx=2)
act_row = tk.Frame(motion)
act_row.pack(pady=2)
tk.Button(act_row, text="重置体感 (面朝上)", width=14,
          command=lambda: send("gr")).pack(side=tk.LEFT, padx=4)
tk.Button(act_row, text="左扭90°", width=8,
          command=lambda: send("tl")).pack(side=tk.LEFT, padx=4)
tk.Button(act_row, text="右扭90°", width=8,
          command=lambda: send("tr")).pack(side=tk.LEFT, padx=4)
auto_var = tk.BooleanVar(value=True)


def toggle_auto():
    send("r 1" if auto_var.get() else "r 0")


tk.Checkbutton(motion, text="每1.5s自动扭腰", variable=auto_var,
               command=toggle_auto).pack(side=tk.LEFT, padx=8)

# raw command
raw_row = tk.Frame(root)
raw_row.pack(pady=2)
tk.Label(raw_row, text="Raw:").pack(side=tk.LEFT)
raw_entry = tk.Entry(raw_row, width=28)
raw_entry.pack(side=tk.LEFT, padx=4)
raw_entry.bind("<Return>", lambda e: (send(raw_entry.get().strip()), raw_entry.delete(0, tk.END)))
tk.Button(raw_row, text="Send", command=lambda: (send(raw_entry.get().strip()),
                                                raw_entry.delete(0, tk.END))).pack(side=tk.LEFT)

# ---- background serial reader ----
link_state = {"text": "未连接"}


def reader():
    buf = b""
    while True:
        try:
            n = ser.in_waiting
            if n:
                buf += ser.read(n)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="replace").strip()
                    if not s:
                        continue
                    if "[st] bt:" in s:
                        # parse connected= and mode= from status
                        import re
                        m = re.search(r"connected=(\d+).*mode=0x([0-9a-f]+)",
                                      s)
                        if m:
                            conn = "已连接" if m.group(1) == "1" else "断连"
                            link_state["text"] = f"{conn} | mode=0x{m.group(2)}"
                    else:
                        rx_line["text"] = s
            else:
                time.sleep(0.02)
        except Exception:
            break


rx_line = {"text": ""}
threading.Thread(target=reader, daemon=True).start()


def poll():
    link.config(text="连接: " + link_state["text"])
    if rx_line["text"]:
        rx_label.config(text="rx: " + rx_line["text"])
    root.after(250, poll)


link = tk.Label(root, text="连接: 未连接", font=("Consolas", 10))
link.pack(anchor=tk.W, padx=12)
rx_label = tk.Label(root, text="rx: -", font=("Consolas", 9), fg="#555")
rx_label.pack(anchor=tk.W, padx=12)

tk.Label(root, text="Latch on: 点击=切换(可多键同按) | Latch off: 鼠标按住=按键按住\n"
                    "休眠后等几秒再按任意键即重新匹配（自动）",
         justify=tk.LEFT).pack(pady=4)

# periodic status poll so the link state stays current
def poll_status():
    try:
        ser.write(b"i\n")
    except Exception:
        pass
    root.after(3000, poll_status)


root.after(3000, poll_status)
root.after(250, poll)
root.mainloop()
