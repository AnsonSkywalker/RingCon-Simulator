# -*- coding: utf-8 -*-
"""
Ring-Con 真值采集（ground-truth probe）
=======================================
用真实右 Joy-Con（插在健身环里）通过蓝牙连 PC，采集两类真值，用于校准模拟器固件：

  1) 环检测启用序列每条子命令的真机应答字节（0x22 / 0x21 / 0x59 / 0x5C / 0x5A）
     —— 0x59 应答含外设 ID 0x20 = Ring-Con（refs/ringcon/ 两个参考实现均已验证）
  2) 0x30 报告里 strain（压力）的真实字节偏移、静息值、推压/拉伸极值

启用序列逐字节复刻 tomayac/joy-con-webhid 的 src/connectRingCon.ts
（refs/ringcon/connectRingCon.ts 已存档）。

用法： python tools\ring_probe.py   （用装有 hidapi 的 Python 运行；
       依赖见 HANDOFF.md §〇.2 的 Phase 0 环境）
前置： 真实右 JC 已与 PC 蓝牙配对并显示「已连接」；JC 插入健身环导轨到底；
       不要运行 pyjoycon / BetterJoy / Steam 等占用 HID 的程序。
输出： 全部日志写入 tools/ring_probe_log.txt（发回给助手用于调固件常量）。

注意：本脚本用原生 hidapi 同步读（带超时），没有 pyjoycon 的后台 daemon，
不存在「边读边关句柄弄脏堆」的问题；结束时不显式 close，由 OS 回收。
"""
import sys
import time

try:
    import hid
except ImportError:
    print("[FAIL] 缺 hidapi 包：pip install hidapi")
    sys.exit(1)

VID, PID = 0x057E, 0x2007  # Joy-Con (R) over Bluetooth

LOG = None


def log(s=""):
    print(s)
    if LOG:
        LOG.write(s + "\n")
        LOG.flush()


def fail(msg):
    log(f"\n[FAIL] {msg}")
    log("—— 排障表 ——")
    log(" 1) 找不到手柄：Windows 蓝牙先『删除设备』旧配对，按住 JC 侧边 SYNC 圆键")
    log("    3 秒，蓝牙添加『Joy-Con (R)』，显示『已连接』后再跑本脚本。")
    log(" 2) 手柄休眠：按一次任意键唤醒（每次跑脚本前都先按一下）。")
    log(" 3) 打开失败/被占用：关掉 BetterJoy、Steam、pyjoycon 等读手柄的程序。")
    log(" 4) 0x59 报 0xFE/无环：确认 JC 已插入健身环导轨到底（卡扣入位）。")
    try:
        input("\n按回车退出...")
    except EOFError:
        pass
    sys.exit(1)


# ---- 与 webhid connectRingCon.ts 逐字节一致的启用序列 ----
MCU_CFG_21 = ([0x21, 0x21, 0x01, 0x01] + [0x00] * 34 + [0xF3])  # 38B + CRC8
FMT_CFG_5C = [0x5C, 0x06, 0x03, 0x25, 0x06, 0x00, 0x00, 0x00, 0x00,
              0x1C, 0x16, 0xED, 0x34, 0x36, 0x00, 0x00, 0x00, 0x0A,
              0x64, 0x0B, 0xE6, 0xA9, 0x22, 0x00, 0x00, 0x04, 0x00,
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0xA8, 0xE1,
              0x34, 0x36]


class Probe:
    def __init__(self):
        devs = hid.enumerate(VID, PID)
        if not devs:
            fail("找不到 Joy-Con (R)（VID 057E / PID 2007）。先完成蓝牙配对。")
        # 设备路径可能内嵌序列号，日志只记 vid/pid（脱敏惯例）
        log(f"[dev] 找到 {len(devs)} 个 Joy-Con (R) HID 实例，取第一个")
        self.dev = hid.device()
        try:
            self.dev.open_path(devs[0]["path"])
        except Exception as e:
            fail(f"打开手柄失败：{e}")
        self.dev.set_nonblocking(0)
        self.counter = 0

    def subcmd(self, args):
        # 输出报告：[0x01][counter][rumble*8][subcmd args...]（dekuNukem 布局）
        pkt = bytes([0x01, self.counter & 0xF]) + b"\x00" * 8 + bytes(args)
        self.counter += 1
        self.dev.write(pkt)

    def read_reply(self, echo, timeout_s=5.0):
        """等 ack 应答（0x21）：含报告 ID 索引下 buf[14]=subcmd 回显。"""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            r = self.dev.read(384, timeout_ms=100)
            if r and r[0] == 0x21 and len(r) > 14 and r[14] == echo:
                return bytes(r)
        return None

    def drain(self, sec=0.3):
        t0 = time.time()
        while time.time() - t0 < sec:
            self.dev.read(384, timeout_ms=20)

    def step(self, name, args, echo, check=None, tries=3):
        for i in range(tries):
            self.drain(0.1)
            self.subcmd(args)
            r = self.read_reply(echo)
            if r is not None:
                log(f"[OK] {name} 应答（{len(r)}B）: {r.hex(' ')}")
                if check and not check(r):
                    return ("checked-fail", r)
                time.sleep(0.05)
                return ("ok", r)
        return ("timeout", None)


def collect(dev, seconds):
    frames = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        r = dev.read(384, timeout_ms=50)
        if r and r[0] == 0x30 and len(r) >= 50:
            frames.append(bytes(r))
    return frames


def signed16(v):
    return v - 0x10000 if v >= 0x8000 else v


def stats(frames, off=39):
    vals = [signed16((f[off + 1] << 8) | f[off]) for f in frames]
    m = sum(vals) / len(vals)
    std = (sum((v - m) ** 2 for v in vals) / len(vals)) ** 0.5
    return m, std, min(vals), max(vals)


def marker_stats(frames):
    """真机 extdev 格式标记：第 3 IMU 帧 acc-X=0x0000（r[37..38]）、
    acc-Z=0x2000（r[41..42]），陀螺保持真实。两标记全中 → strain 槽位
    r[39..40] 可信。"""
    n = len(frames)
    x_ok = sum(1 for f in frames if ((f[38] << 8) | f[37]) == 0x0000)
    z_ok = sum(1 for f in frames if ((f[42] << 8) | f[41]) == 0x2000)
    return x_ok, z_ok, n


def analyze(rest, press, pull):
    """按真机标记定位 strain 槽位（r[39..40]）并输出三段统计。
    旧版『推压变化最大』扫偏移会误锁到第 3 帧 gyro 接缝——r[44..45] 恰好
    跨在 gyro-x 与 gyro-y 两个字段上，压环时手抖让两轴同动产生假峰
    （2026-09-09 日志教训）。扫描表降级为交叉参考，不作定位依据。"""
    for name, seg in (("静息", rest), ("推压", press), ("拉伸", pull)):
        if not seg:
            continue
        x_ok, z_ok, n = marker_stats(seg)
        note = "" if x_ok == n and z_ok == n else "  ⚠ 命中不全，看交叉参考表"
        log(f"[mark] {name} 段标记命中: acc-X=0x0000 {x_ok}/{n}, "
            f"acc-Z=0x2000 {z_ok}/{n}{note}")
    m_rest, s_rest, lo_r, hi_r = stats(rest)
    m_press, s_press, lo_p, hi_p = stats(press)
    m_pull = s_pull = lo_l = hi_l = None
    if pull:
        m_pull, s_pull, lo_l, hi_l = stats(pull)
    log("[scan] 各偏移 推压-静息 均值差（跨字段读数会产生假峰，仅参考）:")
    cells = []
    for off in range(36, 47):
        d = (sum(signed16((f[off + 1] << 8) | f[off]) for f in press) / len(press)
             - sum(signed16((f[off + 1] << 8) | f[off]) for f in rest) / len(rest))
        cells.append(f"{off}:{d:+.0f}")
    log("       " + "  ".join(cells))
    return (m_rest, s_rest, lo_r, hi_r,
            m_press, s_press, lo_p, hi_p,
            m_pull, s_pull, lo_l, hi_l)


def main():
    global LOG
    import os
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "ring_probe_log.txt")
    LOG = open(log_path, "w", encoding="utf-8")
    log("=" * 62)
    log(" Ring-Con 真值采集（真实右 JC + 健身环 → PC 蓝牙）")
    log("=" * 62)
    log("前置检查：① JC 在环里插到底 ② 蓝牙显示『已连接』")
    log("          ③ 刚按过任意键（不在休眠）④ 没开 BetterJoy/Steam/pyjoycon")
    input("按回车开始 >> ")

    p = Probe()
    log("\n—— 第 1 步：环检测启用序列（复刻 webhid connectRingCon.ts）——")

    st, r = p.step("0x03 输入模式→0x30", [0x03, 0x30], 0x03)
    if st == "timeout":
        fail("0x03 无应答。手柄休眠或被占用（排障表 2/3）。")
    st, r = p.step("0x40 六轴使能", [0x40, 0x01], 0x40)
    if st == "timeout":
        fail("0x40 无应答。")
    st, r = p.step("0x22 MCU resume", [0x22, 0x01], 0x22)
    if st == "timeout":
        fail("0x22 无应答（MCU 未启动）。")
    st, r = p.step("0x21 MCU 配置(external ready)", MCU_CFG_21, 0x21)
    if st == "timeout":
        fail("0x21 无应答（MCU 配置被拒）。")
    st, r = p.step("0x59 GetExtDevInfo（环检测点）", [0x59], 0x59,
                   check=lambda rr: len(rr) > 16 and rr[16] == 0x20)
    if st == "timeout":
        fail("0x59 无应答。")
    if st == "checked-fail":
        v15 = r[15] if len(r) > 15 else -1
        v16 = r[16] if len(r) > 16 else -1
        if v15 == 0xFE:
            fail(f"真机 JC 报告未检测到环（应答 data[0]=0xFE）：环没插好（排障表 4）。")
        fail(f"0x59 应答异常：data[0]=0x{v15:02X} data[1]=0x{v16:02X}"
             f"（期望 0x00/0x20）。把日志发回给助手。")
    log("    → ✅ 真机确认环在线：应答 data[1]=0x20 = Ring-Con 外设 ID")
    st, r = p.step("0x5C ExtDev 格式配置", FMT_CFG_5C, 0x5C)
    if st == "timeout":
        fail("0x5C 无应答。")
    st, r = p.step("0x5A 轮询使能", [0x5A, 0x04, 0x01, 0x01, 0x02], 0x5A)
    if st == "timeout":
        fail("0x5A 无应答。")

    log("\n—— 第 2 步：strain 真值标定（三段各 3 秒）——")
    log("把 JC 连环平放桌面，保持静止")
    input("按回车采集【静息】>> ")
    rest = collect(p.dev, 3.0)
    if len(rest) < 30:
        fail(f"0x30 流太稀（{len(rest)} 包）：手柄休眠了？按任意键重跑。")
    log(f"    静息 {len(rest)} 包")
    log("现在【向内用力推压环并保持不动】")
    input("按回车采集【推压】>> ")
    press = collect(p.dev, 3.0)
    log(f"    推压 {len(press)} 包")
    log("现在【向外拉伸环并保持不动】")
    input("按回车采集【拉伸】>> ")
    pull = collect(p.dev, 3.0)
    log(f"    拉伸 {len(pull)} 包")

    (m_rest, s_rest, lo_r, hi_r,
     m_press, s_press, lo_p, hi_p,
     m_pull, s_pull, lo_l, hi_l) = analyze(rest, press, pull)
    log("\n—— 标定结果（strain 槽位 r[39..40]；0xA1 帧 40-41 = 固件 buf[40..41]）——")
    log(f"静息: {m_rest:.0f} ± {s_rest:.0f}   （min {lo_r} / max {hi_r}）")
    log(f"推压: {m_press:.0f} ± {s_press:.0f}   （Δ{m_press - m_rest:+.0f}，"
        f"min {lo_p} / max {hi_p}）")
    if m_pull is not None:
        log(f"拉伸: {m_pull:.0f} ± {s_pull:.0f}   （Δ{m_pull - m_rest:+.0f}，"
            f"min {lo_l} / max {hi_l}）")
    swing_p = max(abs(m_press - m_rest), abs(hi_p - lo_r))
    if swing_p < 300:
        log("⚠ 推压段变化 <300：应变片可能没受力——检查 JC 是否插到底卡扣入位、"
            "是否捏在环腿上用力；建议重跑一次再看")
    if m_press > m_rest and (m_pull is None or m_pull < m_rest):
        log("方向自检: 推压>静息>拉伸 ✓（与社区实测一致）")
    elif swing_p >= 300:
        log("方向自检: ✗ 与『推压增大/拉伸减小』不符——以本环实测为准：固件常量"
            "按实测填，游戏内推/拉若反向，调 motion.h Config 或用 sq 滑杆纠正")
    log("\n全帧 hex（第一包）:")
    log(f"  静息: {rest[0].hex(' ')}")
    log(f"  推压: {press[0].hex(' ')}")
    if pull:
        log(f"  拉伸: {pull[0].hex(' ')}")
    log("\n—— 固件常量建议（src/motion.h RingStrain::Config）——")
    log(f"  rest_raw  = {round(m_rest)}")
    log(f"  press_raw = {round(m_press)}")
    if m_pull is not None:
        log(f"  pull_raw  = {round(m_pull)}")
    log(f"\n请把 {log_path} 发回给助手。")
    # 不显式 close 句柄（项目惯例：进程退出由 OS 回收）
    try:
        input("\n按回车退出...")
    except EOFError:
        pass


if __name__ == "__main__":
    main()
