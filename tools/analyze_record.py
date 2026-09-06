# -*- coding: utf-8 -*-
"""
对 step2_record.py 产出的 record_*.csv 离线计算四指标并出图（手柄无需在线）。
用法：
  python analyze_record.py                  # 分析当前目录全部 record_*.csv
  python analyze_record.py a.csv b.csv      # 指定文件
产物：每份 CSV 旁生成 <同名>_analysis.png；控制台打印单文件指标 + 横向对比。
数据约定：CSV 每行 = (t_s, frame_in_packet 0..2, acc xyz [g], gyro xyz [dps，已扣零偏])；
          每次轮询连写 3 行（同一 t），同一 HID 包可能被连续轮询多次 → 按 18 值全等去重；
          帧时间轴重构为 t_packet + i*5ms（IMU 真实帧周期 15ms/3）。
"""
import csv, glob, math, os, sys

FRAME_DT = 0.005

def load_packets(path):
    with open(path, encoding="utf-8") as f:
        r = csv.reader(f)
        next(r)  # header
        raw = [(float(a), int(b), *[float(x) for x in rest]) for a, b, *rest in r]
    n = len(raw) // 3 * 3
    pkts = []  # (t_first_seen, 18 vals)
    for k in range(0, n, 3):
        g = raw[k:k+3]
        vals = tuple(v for row in g for v in row[2:])
        if pkts and vals == pkts[-1][1]:
            continue  # 同一 HID 包被连续轮询多次，只留首次
        pkts.append((g[0][0], vals))
    frames = []   # (t=包首见时刻+i*5ms, ax, ay, az, gx, gy, gz) —— 5ms 帧距假设
    frames_wall = []  # (t=包首见时刻+i*包间隔/3, ...) —— 按实际投递间隔分摊（对照）
    for ki, (t, vals) in enumerate(pkts):
        gap = (pkts[ki+1][0] - t) if ki + 1 < len(pkts) else 3 * FRAME_DT
        for i in range(3):
            ax, ay, az, gx, gy, gz = vals[6*i:6*i+6]
            frames.append((t + i*FRAME_DT, ax, ay, az, gx, gy, gz))
            frames_wall.append((t + i*gap/3, ax, ay, az, gx, gy, gz))
    return pkts, frames, frames_wall

def analyze(path):
    pkts, fr, fr_wall = load_packets(path)
    t = [f[0] for f in fr]
    G = [[f[4+k] for f in fr] for k in range(3)]  # dps，已扣零偏
    A = [[f[1+k] for f in fr] for k in range(3)]  # g
    ax_names = ("x", "y", "z")

    # 主导轴 = 峰峰值最大者
    pps = [max(v) - min(v) for v in G]
    dom = max(range(3), key=lambda k: pps[k])
    v = G[dom]
    peak_abs = max(abs(x) for x in v)
    ipk = max(range(len(v)), key=lambda i: abs(v[i]))

    # 动作窗口：|ω| > 10% 峰值
    thr = 0.10 * peak_abs
    idx = [i for i, x in enumerate(v) if abs(x) > thr]
    i0, i1 = idx[0], idx[-1]

    # 转角：主导轴梯形积分（重构时间轴，帧距 5ms）
    ang = [0.0]
    for i in range(1, len(t)):
        ang.append(ang[-1] + 0.5 * (v[i] + v[i-1]) * (t[i] - t[i-1]))
    net_win = ang[i1] - ang[i0]

    # 对照积分：帧时间按实际投递间隔分摊（检验 5ms 假设；两版差太大说明丢包/降速）
    tw = [f[0] for f in fr_wall]
    ang_w = [0.0]
    for i in range(1, len(tw)):
        ang_w.append(ang_w[-1] + 0.5 * (v[i] + v[i-1]) * (tw[i] - tw[i-1]))
    net_win_w = ang_w[i1] - ang_w[i0]

    # 静止段噪声（窗外主导轴）
    out = [v[i] for i in range(len(v)) if i < i0 or i > i1]
    m = sum(out) / len(out)
    noise = math.sqrt(sum((x - m) ** 2 for x in out) / len(out))

    span = t[-1] - t[0]
    print(f"\n== {os.path.basename(path)} ==")
    print(f"  HID 包 {len(pkts)} 个 / {span:.2f}s（≈{len(pkts)/span:.0f} Hz），帧 {len(fr)} 个")
    print(f"  gyro 峰峰值 dps: x={pps[0]:7.1f}  y={pps[1]:7.1f}  z={pps[2]:7.1f}")
    print(f"  gyro 峰值   dps: x={max(G[0], key=abs):+7.1f}  y={max(G[1], key=abs):+7.1f}  z={max(G[2], key=abs):+7.1f}")
    print(f"  accel 峰峰值 g : x={max(A[0])-min(A[0]):5.2f}  y={max(A[1])-min(A[1]):5.2f}  z={max(A[2])-min(A[2]):5.2f}")
    print(f"  —— 主导轴 gyro_{ax_names[dom]} ——")
    print(f"  峰值角速度   : {v[ipk]:+.0f} dps（t={t[ipk]:.2f}s）")
    print(f"  峰峰值       : {pps[dom]:.0f} dps")
    print(f"  动作窗口     : {t[i1]-t[i0]:.2f} s（t={t[i0]:.2f}→{t[i1]:.2f}，起点后 {t[ipk]-t[i0]:.2f}s 达峰）")
    print(f"  窗口内净转角 : {net_win:+.1f} deg   （全程积分 {ang[-1]:+.1f} deg）")
    print(f"  墙钟对照积分 : {net_win_w:+.1f} deg（窗口内，与上行差大=时间轴不可信）")
    print(f"  静止段噪声   : {noise:.2f} dps (1σ)")

    plot(path, t, G, ang, dom, ax_names, i0, i1, net_win)
    return dict(file=os.path.basename(path), dom=ax_names[dom],
                peak=abs(v[ipk]), pp=pps[dom], win=t[i1]-t[i0],
                rise=t[ipk]-t[i0], net=net_win, net_w=net_win_w,
                noise=noise, hz=len(pkts)/span)

def plot(path, t, G, ang, dom, ax_names, i0, i1, net_win):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax1 = plt.subplots(figsize=(11, 6))
    ax1.plot(t, G[0], color="tab:green", lw=0.8, label="gyro_x (dps)")
    ax1.plot(t, G[1], color="tab:orange", lw=0.8, label="gyro_y (dps)")
    ax1.plot(t, G[2], color="tab:red", lw=0.8, label="gyro_z (dps)")
    ax1.set_xlabel("t (s)"); ax1.set_ylabel("gyro (dps)")
    ax2 = ax1.twinx()
    ax2.plot(t, ang, color="tab:blue", lw=1.8, label=f"angle of gyro_{ax_names[dom]} (deg)")
    ax2.set_ylabel("累计转角 (deg)")
    ax1.axvspan(t[i0], t[i1], color="gray", alpha=0.10)
    ax1.legend(loc="upper left"); ax2.legend(loc="upper right")
    plt.title(f"{os.path.basename(path)}  主导轴 gyro_{ax_names[dom]}  窗口内净转角 {net_win:+.0f}°")
    plt.grid(alpha=0.3)
    png = os.path.splitext(path)[0] + "_analysis.png"
    plt.tight_layout(); plt.savefig(png, dpi=130)
    plt.close(fig)
    print(f"  图: {png}")

if __name__ == "__main__":
    files = sys.argv[1:] or sorted(glob.glob("record_*.csv"))
    if not files:
        sys.exit("当前目录没有 record_*.csv，或用参数指定 CSV 路径")
    res = [analyze(f) for f in files]
    print("\n—— 横向对比 ——")
    print(f"{'文件':<22}{'主导轴':<8}{'|峰值|dps':>10}{'峰峰值':>8}{'窗口s':>7}{'达峰s':>7}{'净转角':>9}{'墙钟积分':>9}{'噪声1σ':>8}{'包Hz':>6}")
    for r in res:
        print(f"{r['file']:<22}{r['dom']:<8}{r['peak']:>10.0f}{r['pp']:>8.0f}"
              f"{r['win']:>7.2f}{r['rise']:>7.2f}{r['net']:>+9.1f}{r['net_w']:>+9.1f}{r['noise']:>8.2f}{r['hz']:>6.0f}")
