# AIGC note: jc_remote.py - PC-side Joy-Con (R) control panel. All buttons are
# abstract (press A = "bp A"), the firmware routes to the right bit for the
# host's current phase (grip-screen simple-HID vs full mode) so the user never
# thinks about report modes. Latch off (default): mouse-hold = held. Latch on:
# click = toggle; latch several at once (e.g. SL+SR then press A).
#
# rst = sleep/unlink (real Joy-Con behavior; Switch icon disappears). gr =
# orientation reset (return synthetic controller to face-up flat). Motion
# buttons (X±/Y±/Z±/tl/tr) are one-shot firmware commands.
#
# Usage:  python jc_remote.py [COM5] [--selftest]
#   --selftest: build the UI, programmatically exercise every callback, exit.
# Requires: pip install pyserial   (tkinter ships with Python on Windows)
import os
import re
import sys
import threading
import time
import tkinter as tk

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "COM5"
BAUD = 115200

# Right Joy-Con real buttons.
BUTTONS = ["A", "B", "X", "Y", "SL", "SR", "R", "ZR", "Plus", "RStick", "Home"]

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.1
ser.dsrdtr = False  # don't pulse DTR -> board keeps its connection across opens
ser.rtscts = False
ser.dtr = False
ser.rts = False
try:
    ser.open()
except serial.SerialException as e:
    # COM5 被占用（ZCode 后台抓取/另一个前端实例/烧录中）是最常见原因
    print(f"[FAIL] 打开 {PORT} 失败：{e}")
    print("串口可能被占用：关闭 ZCode 的后台抓取任务 / 其他前端实例后重试，")
    print("或让助手先停掉串口任务再启动本前端。")
    raise SystemExit(1)
ser.reset_input_buffer()

# 全量串口日志落盘：窗口上只显示最后一行 rx，冷启动抓包等场景靠文件回看。
# 含 tx+rx 全部行（0x02 应答里有 MAC，公开日志前先脱敏）。
LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "jc_session_log.txt")
try:
    _log_mode = "a" if os.path.getsize(LOG_PATH) < 5 * 1024 * 1024 else "w"
except OSError:
    _log_mode = "w"
session_log = open(LOG_PATH, _log_mode, encoding="utf-8")


def slog(tag, text):
    # 带日期：追加式日志跨天复用同一文件，只有时分秒会和前一天的行混淆
    session_log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {tag} {text}\n")
    session_log.flush()


slog("ui", f"frontend start (pid={os.getpid()}, port={PORT})")

# tx> 行的操作来源标记：panel=面板点击 / selftest=自检 / exit=关窗口收尾。
# 轮询 i 不走 send()、不落日志（3s 一条会把日志刷满，且非用户操作）。
src_tag = {"cur": "panel"}


root = tk.Tk()
root.title(f"Joy-Con (R) Remote -> {PORT}")

latch = tk.BooleanVar(value=False)
auto_var = tk.BooleanVar(value=False)  # 自动扭腰默认关：装环检测等场景不该有自动动作
held = set()          # buttons currently latched down
rx_line = {"text": ""}     # last firmware response line (reader thread)
link_state = {"text": "未连接"}  # parsed from periodic status replies
held_bg = "#9ecbff"

btns = {}
status = None  # assigned during UI construction


def send(cmd):
    slog("tx>", f"[{src_tag['cur']}] {cmd}")
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
        return  # latch mode: release happens on the next click
    send(f"br {name}")


def clear_all():
    for name in list(held):
        send(f"br {name}")
    held.clear()
    for n in btns:
        btns[n].config(relief=tk.RAISED, bg=btns[n].default_bg)


def toggle_auto():
    send("r 1" if auto_var.get() else "r 0")


def raw_send(*_):
    cmd = raw_entry.get().strip()
    if cmd:
        send(cmd)
        raw_entry.delete(0, tk.END)


def wipe_pairing():
    # 实体同步键长按的等价操作：清主机记忆+bond、重启进配对模式。
    # 破坏性操作（需重新配对），确认后才发送；selftest 桩住对话框覆盖本回调。
    import tkinter.messagebox as mb
    if mb.askyesno("清除配对（长按同步键）",
                   "清除板子的主机记忆并重启进配对模式。\n"
                   "之后需要在「更改握法/顺序」重新配对，确定？"):
        send("unpair")


# ---- UI construction (all widgets referenced by callbacks exist below) ----
status = tk.Label(root, text="-", font=("Consolas", 10))
status.pack(anchor=tk.W, padx=12, pady=2)

grid = tk.LabelFrame(root, text="按键 (自动适配配对屏/游戏模式)")
grid.pack(padx=10, pady=6)
for name in BUTTONS:
    b = tk.Button(grid, text=name, width=7)
    b.default_bg = b.cget("bg")
    b.bind("<ButtonPress-1>", lambda e, n=name: button(n))
    b.bind("<ButtonRelease-1>", lambda e, n=name: release(n))
    btns[name] = b
btns["Y"].configure(width=5)
btns["X"].configure(width=5)
btns["A"].configure(width=5)
btns["B"].configure(width=5)
btns["Y"].grid(row=0, column=1, pady=2)
btns["X"].grid(row=1, column=0, pady=2)
btns["A"].grid(row=1, column=2, pady=2)
btns["B"].grid(row=2, column=1, pady=2)
for i, name in enumerate(["SL", "SR", "R", "ZR", "Plus", "RStick", "Home"]):
    btns[name].grid(row=i, column=4, padx=4, pady=2)

ctrl = tk.Frame(root)
ctrl.pack(pady=2)
tk.Checkbutton(ctrl, text="Latch (click = toggle)", variable=latch).pack(side=tk.LEFT, padx=8)
tk.Button(ctrl, text="清空", command=clear_all).pack(side=tk.LEFT, padx=4)
tk.Button(ctrl, text="休眠/断连 (rst)", width=16,
          command=lambda: send("rst")).pack(side=tk.LEFT, padx=4)
tk.Button(ctrl, text="清除配对 (同步键长按)", width=20,
          command=wipe_pairing).pack(side=tk.LEFT, padx=4)

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
tk.Checkbutton(motion, text="每1.5s自动扭腰", variable=auto_var,
               command=toggle_auto).pack(side=tk.LEFT, padx=8)

# 健身环通道：滑杆 = 应变片连续挤压量（+=推压 / -=拉伸）；推压一次 = 游戏
# 开局校准动作（挤压-保持-回弹固件曲线）。拖动中不刷串口，松手才发。
ring = tk.LabelFrame(root, text="健身环")
ring.pack(fill=tk.X, padx=10, pady=4)
ring_row = tk.Frame(ring)
ring_row.pack(pady=2)
tk.Label(ring_row, text="挤压:").pack(side=tk.LEFT)
sq_scale = tk.Scale(ring_row, from_=-100, to=100, orient=tk.HORIZONTAL,
                    length=220, showvalue=True)
sq_scale.set(0)
sq_scale.pack(side=tk.LEFT, padx=4)


def sq_release(_event=None):
    send(f"sq {sq_scale.get()}")


sq_scale.bind("<ButtonRelease-1>", sq_release)
tk.Button(ring_row, text="推压一次", width=10,
          command=lambda: send("sqp")).pack(side=tk.LEFT, padx=4)
tk.Button(ring_row, text="回零", width=6,
          command=lambda: (sq_scale.set(0), sq_release())).pack(side=tk.LEFT)

raw_row = tk.Frame(root)
raw_row.pack(pady=2)
tk.Label(raw_row, text="Raw:").pack(side=tk.LEFT)
raw_entry = tk.Entry(raw_row, width=28)
raw_entry.pack(side=tk.LEFT, padx=4)
raw_entry.bind("<Return>", raw_send)
tk.Button(raw_row, text="Send", command=raw_send).pack(side=tk.LEFT)

link = tk.Label(root, text="连接: -", font=("Consolas", 10))
link.pack(anchor=tk.W, padx=12)
rx_label = tk.Label(root, text="rx: -", font=("Consolas", 9), fg="#555")
rx_label.pack(anchor=tk.W, padx=12)
tk.Label(root, text="Latch on: 点击=切换(可多键同按) | Latch off: 鼠标按住=按键按住\n"
                    "真机语义：休眠/待机=射频静默，点击任意按键唤醒并搜索主机（15 秒搜不到自动回睡）",
         justify=tk.LEFT).pack(pady=4)

# 窗口尺寸只按控件内容定一次：超宽日志行的全量去 jc_session_log.txt 里看，
# 屏上超长行直接裁掉——否则每条 rx 都把窗口撑宽又缩回，来回乱跳。
root.update_idletasks()
root.geometry(f"{root.winfo_reqwidth() + 40}x{root.winfo_reqheight() + 10}")
root.pack_propagate(False)
root.resizable(False, False)


# ---- background serial reader + UI pollers ----
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
                    slog("rx", s)
                    if s.startswith("[st] bt:"):
                        m = re.search(r"connected=(\d+).*mode=0x([0-9a-f]+)", s)
                        w = re.search(r"awake=(\d)", s)
                        if m:
                            if m.group(1) == "1":
                                conn = "已连接"
                            elif w and w.group(1) == "0":
                                conn = "休眠 | 点任意按键唤醒"
                            else:
                                conn = "搜索主机中…"
                            link_state["text"] = f"{conn} | mode=0x{m.group(2)}"
                    else:
                        # truncate: long hex/log lines must not stretch the window
                        rx_line["text"] = s[:90]
            else:
                time.sleep(0.02)
        except Exception:
            break  # port closed -> frontend is exiting


threading.Thread(target=reader, daemon=True).start()


def poll():
    link.config(text="连接: " + link_state["text"])
    if rx_line["text"]:
        rx_label.config(text="rx: " + rx_line["text"])
    root.after(250, poll)


def poll_status():
    try:
        ser.write(b"i\n")
    except Exception:
        pass
    root.after(3000, poll_status)


root.after(250, poll)
root.after(1500, poll_status)

# 开机把固件的自动扭腰同步成面板默认值（关）。r 0 是非按键命令：
# 板子休眠时固件直接忽略，不会违背「只有按键才唤醒」的语义。
src_tag["cur"] = "startup"
send("r 0")
src_tag["cur"] = "panel"


def on_close():
    # 点窗口退出 = 真机放回桌面的语义：先发 rst 让板子休眠（射频静默）再关串口
    slog("ui", "frontend exit (window closed)")
    try:
        src_tag["cur"] = "exit"
        send("rst")
        ser.flush()
        time.sleep(0.3)  # 给固件一拍时间收完这条再断串口
    except Exception:
        pass
    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_close)

if "--selftest" in sys.argv:
    # Exercise every callback path headlessly, then exit. No rst (would kick
    # the board off the Switch mid-session).
    src_tag["cur"] = "selftest"  # 日志里和真人按键区分开
    import time as _t
    root.update()
    for name in BUTTONS:
        button(name)
        root.update()
        release(name)
        root.update()
    latch.set(True)
    button("A")            # latch on: 首次点击=按住
    button("A")            # 第二次点击=松开（toggle）
    latch.set(False)
    button("A")            # latch off: 鼠标按住路径
    release("A")
    for cmd in ["gr", "tl", "tr", "rot 0 1", "rot 2 -1", "sqp"]:
        send(cmd)
        root.update()
    sq_scale.set(80)
    sq_release()  # ring slider callback
    sq_scale.set(-40)
    sq_release()
    sq_scale.set(0)
    sq_release()
    toggle_auto()
    toggle_auto()
    clear_all()
    # 清除配对回调：桩住确认对话框（返回 False → 不发送，不破坏当前配对）
    import tkinter.messagebox as _mb
    _orig_ask = _mb.askyesno
    _mb.askyesno = lambda *a, **k: False
    try:
        wipe_pairing()
        root.update()
    finally:
        _mb.askyesno = _orig_ask
    raw_entry.insert(0, "p")
    raw_send()
    for _ in range(8):
        root.update()
        _t.sleep(0.05)
    assert held == set(), f"buttons still latched: {held}"
    # 退出回调已挂到窗口关闭事件上（selftest 不真触发，避免真发 rst）
    assert root.protocol("WM_DELETE_WINDOW"), "close handler not registered"
    print("SELFTEST OK: all callbacks exercised without error")
    root.destroy()
    sys.exit(0)

root.mainloop()
