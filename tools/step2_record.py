# -*- coding: utf-8 -*-
"""
Phase 0 · 第二步：录制"扭腰 90°"真实动作曲线
用法：python step2_record.py
流程：静止基线 3 秒 → 提示做动作 → 录 6 秒 → 存 CSV + PNG + 控制台输出关键指标
产物：record_时间戳.csv / record_时间戳.png（与脚本同目录，建议先 cd 到一个可写目录再跑，
      或修改下方 OUT_DIR 指定输出目录）
注意：① 必须在上一个脚本进程完全退出后再运行（hidapi 独占打开）。
      ② 报"找不到右手柄"时：多为两次运行之间蓝牙链路断开、手柄休眠——按一下手柄
         任意键唤醒，Windows 蓝牙设置里显示『已连接』后再跑（step1 退出后设备会
         从 HID 层消失，这是实测行为，不是脚本坏了）。
"""
import sys, time, csv, os
from datetime import datetime

OUT_DIR = os.environ.get("PHASE0_OUT", os.getcwd())  # 产物输出目录（默认当前工作目录）

def fail(msg):
    print(f"\n[FAIL] {msg}")
    input("\n按回车退出...")
    sys.exit(1)

try:
    # 同 step1：必须用 PythonicJoyCon 才有 accel_in_g / gyro_in_deg 单位换算
    from pyjoycon import PythonicJoyCon as JoyCon, get_R_id
except Exception as e:
    fail(f"pyjoycon 导入失败：{e}（先跑 step1_selftest.py）")

rid = get_R_id()
if not rid or rid[0] is None:
    fail("找不到右手柄（最常见的坑：上次脚本退出后蓝牙断开，手柄休眠了）。\n"
         "  → 先按一下手柄任意键唤醒，到 Windows 蓝牙设置确认 Joy-Con (R) 显示『已连接』，再重跑本脚本；\n"
         "  → 仍不行再走配对流程：删除设备 → 按住 SYNC 键 3s → 重新添加（step1 排障表第 1 条）。")
print(f"右手柄 id = {rid}，打开中...")
try:
    jc = JoyCon(*rid)
except Exception as e:
    fail(f"打开手柄失败：{e}")
print("已连接。")

# ================= 参数（可调） =================
BASELINE_SEC = 3.0    # 静止基线时长（用于二次零偏校准）
RECORD_SEC   = 6.0    # 动作录制时长
POLL_DT      = 0.004  # 轮询间隔 ≈ 250Hz 轮询，覆盖 HID 实际 ~66Hz 包率（每包 3 帧）
# ================================================

print(f"\n—— 第 1 步：静止零偏校准 {BASELINE_SEC:.0f} 秒 ——")
print("   手柄平放（或插在环上）保持完全不动")
input("   按回车开始 >> ")
t0 = time.time()
base = {0: [], 1: [], 2: []}
while time.time() - t0 < BASELINE_SEC:
    for i, (gx, gy, gz) in enumerate(jc.gyro_in_deg):
        base[i].append(gx if i == 0 else (gy if i == 1 else gz))
    time.sleep(POLL_DT)
# 注意：base[0]=x 轴序列, base[1]=y 轴序列, base[2]=z 轴序列（每帧三元组的分量拆开）
offs = [sum(base[i])/len(base[i]) for i in range(3)]
print(f"   零偏 dps: x={offs[0]:+.3f}  y={offs[1]:+.3f}  z={offs[2]:+.3f}")

print(f"\n—— 第 2 步：录制 {RECORD_SEC:.0f} 秒 ——")
print("   >>> 提示后做一次完整的『扭腰 90°』（像游戏里那样真实地扭！） <<<")
input("   按回车 → 1 秒后开始录 >> ")
time.sleep(1.0)
print("   ●●● 录制中！现在做动作！ ●●●")

rows = []
t0 = time.time()
while time.time() - t0 < RECORD_SEC:
    t = time.time() - t0
    for i in range(3):  # 每包 3 帧，帧间 ~5ms（与 Switch 侧时序一致）
        ax, ay, az = jc.accel_in_g[i]
        gx, gy, gz = jc.gyro_in_deg[i]
        rows.append((t, i, ax, ay, az,
                     gx - offs[0], gy - offs[1], gz - offs[2]))
    time.sleep(POLL_DT)
print("   ○ 录制结束")

# 注意：不要在这里显式关句柄（jc._close() / disconnect_device() 都不行）——
# 后台线程此刻正阻塞在 hid read()，边读边关会在原生层弄脏堆，
# 进程稍后在 matplotlib 等原生代码里无声崩溃（2026-09-06 实测 4/4 复现）。
# 进程退出时由 OS 回收句柄即可。

# ================= 存 CSV =================
ts = datetime.now().strftime("%H%M%S")
csv_path = os.path.join(OUT_DIR, f"record_{ts}.csv")
with open(csv_path, "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "frame_in_packet",
                "accel_x_g", "accel_y_g", "accel_z_g",
                "gyro_x_dps", "gyro_y_dps", "gyro_z_dps"])
    for r in rows:
        w.writerow([f"{r[0]:.4f}", r[1]] + [f"{v:.4f}" for v in r[2:]])
print(f"\nCSV 已存: {csv_path}")

# ================= 分析 + 画图 =================
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
except Exception as e:
    print(f"(matplotlib 不可用，跳过画图: {e})")
    sys.exit(0)

t  = [r[0] for r in rows]
gx = [r[5] for r in rows]
gy = [r[6] for r in rows]
gz = [r[7] for r in rows]

def peak(v): return max(v) - min(v)
axes = {"gyro_x": gx, "gyro_y": gy, "gyro_z": gz}
best = max(axes, key=lambda k: peak(axes[k]))
series = axes[best]

# 对主导轴积分求转角
ang = [0.0]
for i in range(1, len(t)):
    ang.append(ang[-1] + series[i] * (t[i] - t[i-1]))

fig, ax1 = plt.subplots(figsize=(11, 6))
ax1.plot(t, gx, color="tab:green", lw=0.8, label="gyro_x (dps)")
ax1.plot(t, gy, color="tab:orange", lw=0.8, label="gyro_y (dps)")
ax1.plot(t, gz, color="tab:red", lw=0.8, label="gyro_z (dps)")
ax1.set_xlabel("t (s)"); ax1.set_ylabel("gyro (dps)")
ax2 = ax1.twinx()
ax2.plot(t, ang, color="tab:blue", lw=1.8, label=f"累计转角 ∫{best} (deg)")
ax2.set_ylabel("累计转角 (deg)")
ax1.legend(loc="upper left"); ax2.legend(loc="upper right")
plt.title(f"扭腰 90° 真实动作曲线（主导轴: {best}）")
plt.grid(alpha=0.3)
png_path = os.path.join(OUT_DIR, f"record_{ts}.png")
plt.tight_layout(); plt.savefig(png_path, dpi=130)
print(f"PNG 已存: {png_path}")

print("\n—— 关键指标（Phase 1 合成曲线的标定依据）——")
print(f"  主导轴            : {best}")
print(f"  峰值角速度        : {max(series, key=abs):.0f} dps")
print(f"  峰峰值            : {peak(series):.0f} dps")
print(f"  粗积分累计转角    : {ang[-1]:.0f} deg")
print(f"  采样帧数          : {len(rows)}")
print("\n把这一屏输出 + PNG 图发给接手的 agent，即可标定 Phase 1 曲线参数。")
input("按回车退出...")
