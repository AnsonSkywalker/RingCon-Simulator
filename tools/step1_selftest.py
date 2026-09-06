# -*- coding: utf-8 -*-
"""
Phase 0 · 第一步：环境自检（不做任何记录）
用法：python step1_selftest.py
作用：确认 Joy-Con 已配对、能打开、能读到实时 IMU 数据流 + 标度自检。
"""
import sys, time, math

def fail(msg):
    print(f"\n[FAIL] {msg}")
    print("—— 排障表 ——")
    print(" 1) 找不到手柄：Windows 设置→蓝牙和其他设备→先『删除设备』旧配对，")
    print("    再按住手柄侧边的 SYNC 圆形小键 3 秒（跑马灯闪烁），")
    print("    蓝牙里添加『Joy-Con (R)』。成功标志：设备列表显示『已连接』。")
    print(" 2) 打开失败/被占用：关掉 BetterJoy、Steam、其他读手柄的程序后重试。")
    print(" 3) hidapi 相关报错：本脚本已用 pip 安装的 hidapi 包，若仍报 dll 问题，")
    print("    到 github.com/libusb/hidapi/releases 下载 hidapi.dll 放本脚本目录。")
    print(" 4) 读数全 0：手柄休眠了，按一次 Home 键唤醒；或重新配对蓝牙。")
    input("\n按回车退出...")
    sys.exit(1)

print("=" * 62)
print(" Phase 0 自检：真实 Joy-Con → PC 数据流")
print("=" * 62)

# ---- 1. 导入 ----
try:
    # 注意：pyjoycon.__init__ 里 JoyCon 先被 joycon.JoyCon 占位，
    # 随后 PythonicJoyCon（带 gyro_in_deg/accel_in_g 单位换算）才是包装类。
    # 0.2.4 的 __init__ 没有把 PythonicJoyCon 重新绑定到 JoyCon，必须显式导入。
    from pyjoycon import PythonicJoyCon as JoyCon, get_R_id
    import pyjoycon
    print(f"[1/4] pyjoycon {pyjoycon.__version__} 导入 OK（使用 PythonicJoyCon，含单位换算）")
except Exception as e:
    fail(f"pyjoycon 导入失败：{e}\n安装命令：pip install joycon-python pyglm hidapi")

# ---- 2. 找右手柄 ----
rid = get_R_id()
# 注意：某些环境下 get_R_id() 返回 (None, None, None) 而非空元组 → 统一判 None
if not rid or rid[0] is None:
    fail("找不到 Joy-Con (R)。请先完成 Windows 蓝牙配对（排障表第 1 条）。")
print(f"[2/4] 右手柄 id = {rid}")

# ---- 3. 打开 ----
try:
    jc = JoyCon(*rid)
except Exception as e:
    fail(f"打开手柄失败：{e}（排障表第 2 条）")
print("[3/4] 手柄已打开，IMU 已使能（内部发 0x40 使能 + 切 0x30 报告模式）")

# ---- 4. 实时读数 6 秒 ----
N_SEC = 6.0
print(f"[4/4] 实时读数 {N_SEC:.0f} 秒 —— 手柄保持静止平放！")
input("      按回车开始 >> ")

t_end = time.time() + N_SEC
accels, gyros = [], []
last_timer = None
new_packets = 0
while time.time() < t_end:
    # jc._input_report 由后台线程持续刷新；timer 字节 [1] 每 15ms 自增 → 用于统计真实包率
    rep = bytes(getattr(jc, "_JoyCon__dict", {}).get("x", b"")) if False else None
    st_g = jc.gyro_in_deg      # [(x,y,z) × 3帧]，已减出厂零偏、已换算 dps
    st_a = jc.accel_in_g       # [(x,y,z) × 3帧]，已换算 g
    gyros.append(st_g)
    accels.append(st_a)
    time.sleep(0.004)

n = len(gyros)
hz = n / N_SEC
print(f"\n轮询 {n} 次（≈{hz:.0f} 次/秒；每 3 次轮询对应同一 HID 包）")

# 取中间时刻的样本做静止基线检查
mid = gyros[len(gyros)//2]
mida = accels[len(accels)//2]

# 加速度：3 帧取模长平均，应 ≈ 1.0 g
mags = []
for (ax, ay, az) in mida:
    mags.append(math.sqrt(ax*ax + ay*ay + az*az))
mag_avg = sum(mags)/len(mags)

# 角速度：静止应接近 0（出厂零偏已扣除），统计三轴抖动
g_all = [v for frame in gyros for tup in frame for v in tup]
g_jitter = max(g_all) - min(g_all)

print("\n—— 静止基线判定 ——")
print(f"  accel 模长（g）: {mag_avg:.3f}   （正常 0.85~1.15，理想 ≈1.0）")
print(f"  gyro 全程峰峰值（dps）: {g_jitter:.2f}   （静止正常 < ±3）")

ok_a = 0.85 <= mag_avg <= 1.15
ok_g = g_jitter < 6.0
if ok_a and ok_g:
    print("\n✅ 自检通过！标度确认：accel_in_g ≈ 真实 g，gyro_in_deg ≈ 真实 dps")
    print("   → 下一步：python step2_record.py 录制你的『扭腰 90°』")
else:
    if not ok_a:
        print("\n⚠️ accel 模长偏离 1.0：若在 0.5~0.7 或 1.4+，说明标度换算与预期不符——")
        print("   把本页输出发给我，不影响继续，但录制时我会按实测比例修正。")
    if not ok_g:
        print("\n⚠️ gyro 抖动偏大：手柄没放稳或桌面震动；不影响继续，零偏会在录制时再扣除。")

# 注意：不要在这里显式关句柄（jc._close() / disconnect_device() 都不行）——
# 后台线程此刻正阻塞在 hid read()，边读边关会在原生层弄脏堆，
# 进程稍后在 matplotlib 等原生代码里无声崩溃（2026-09-06 实测 4/4 复现）。
# 进程退出时由 OS 回收句柄即可。
input("\n按回车退出...")
