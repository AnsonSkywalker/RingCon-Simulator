# -*- coding: utf-8 -*-
"""
Ring-Con MCU 真值采集 v5（真实右 JC + 环 → PC 蓝牙）
====================================================
v5（2026-09-10 复测#5 之后）：身份回退+数据镜像的新固件下，NS2 不再断链，改为
**5 秒内连发 41 次 mode-0 然后放弃、保持连接继续跑**——游戏在 mode-0 之后在等
一个我们没给的东西。头号嫌疑：真机 mode-0（MCU 复位）后会自发上报异步 0x31
状态帧；而 v4 的 reply() 把非 ack 报文**静默丢弃**，collect31 的窗口又从没
覆盖 mode-0 后那一刻——「0x31 已排除」的结论在这个窗口是盲的。旁证：真机
mode-0 ack 耗时 322-548ms（v4 日志），像在 ack 之前先做复位并上报。本版给
0 / mode-3 / ext-ready / 0x59 的关键窗口加盯流（step_watch）：
ack 之外的一切输入照单全收，0x31 全帧转储。

v4（2026-09-10）：NS2 冷启动复测 #3 证明固件对 mode-0 的 ack 已与 v3 真值逐字节
一致、0x22 ack 也一致，但游戏仍不发 mode-3（约 13-16s 主动断链，3 次后放弃）。
协议应答面已对齐 → 剩余嫌疑转向「游戏读到的其他数据」。本版做两件事：

  1) 逐条复刻 NS2 游戏 2026-09-10 11:37 冷启动捕获的完整 34 步序列
     （字节照抄会话日志，含 0x22×4 / 0x6020×4 / 尾部 0x30[01]×5 / 0x48×6），
     采回真机对每一command的 ack 与往返耗时；
  2) 重点核对游戏读的 4 个我们固件填 FF 的 SPI 区域
     （0x6000 序列号 / 0x6080 / 0x6098 / 0x8010）真机上到底存了什么——
     若非 FF，固件照抄后冷启动复测。0x6020/0x603D/0x6050 已镜像 joycontrol
     值，一并采真值参考。

随后接 v3 的后段剧本（mode-3 / ext-ready / 0x59 / 0x40 / 0x5C / 0x5A）与
0x31 采样，复核「游戏精确序列下真机仍 0 帧 0x31」。

日志追加写 tools/ring_probe31_log.txt，头部带日期（防跨天混淆）。
用法： python tools\\ring_probe31.py   （Phase 0 环境，hidapi）
前置： 真实右 JC 蓝牙连 PC（脚本会等待）；环插到底（回车确认——剧本前真机流里
       没有应变签名，无法自动检测）；别开 BetterJoy/Steam/pyjoycon。
"""
import os
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


def now():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def crc8_poly07(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def wait_enter(msg):
    try:
        input(f"\n[等待] {msg}\n回车继续 > ")
    except EOFError:
        pass


# ---------------------------------------------------------------- 设备封装
class Probe:
    def __init__(self):
        self.dev = None
        self.counter = 0

    def open(self):
        devs = hid.enumerate(VID, PID)
        if not devs:
            return False
        self.dev = hid.device()
        self.dev.open_path(devs[0]["path"])
        self.dev.set_nonblocking(0)
        return True

    def out(self, report_id, payload):
        # BT 输出帧：[report_id][counter][rumble*8][payload...]
        pkt = bytes([report_id, self.counter & 0xF]) + b"\x00" * 8 + bytes(payload)
        self.counter += 1
        self.dev.write(pkt)

    def subcmd(self, args):
        self.out(0x01, args)

    def read(self, timeout_ms=100):
        return self.dev.read(384, timeout_ms=timeout_ms)

    def drain(self, sec=0.3):
        t0 = time.time()
        while time.time() - t0 < sec:
            self.read(20)

    def reply(self, echo, timeout_s=4.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            r = self.read(60)
            if r and r[0] == 0x21 and len(r) > 14 and r[14] == echo:
                return bytes(r), (time.time() - t0) * 1000
        return None, (time.time() - t0) * 1000

    def step(self, name, args, echo, tries=3, show=None):
        """发一条子命令并等 ack。show(r) 可选：追加应答解读行。
        返回 (ack bytes | None, 往返ms)。"""
        for _ in range(tries):
            self.drain(0.05)
            t0 = time.time()
            self.subcmd(args)
            r, ms = self.reply(echo)
            total = (time.time() - t0) * 1000
            if r is not None:
                log(f"[OK] {name} 应答 {ms:.0f}ms: {r.hex(' ')}")
                if show:
                    show(r)
                return r, total
        log(f"[!!] {name} 无应答（{total:.0f}ms）")
        return None, total

    def step_watch(self, name, args, echo, watch_s=2.5):
        """发命令后盯原始输入流 watch_s 秒：抓 ack + 0x31/异常报文全帧。
        v4 的 reply() 会丢弃非 ack 报文——mode-0 后若真机自发 0x31 状态帧，
        v4 永远看不见（#5 的 41 连发 mode-0 正指向这个盲区）。"""
        self.drain(0.05)
        t0 = time.time()
        self.subcmd(args)
        ack, ack_ms, f31, other = None, None, [], 0
        while time.time() - t0 < watch_s:
            r = self.read(30)
            if not r:
                continue
            dt = (time.time() - t0) * 1000
            if r[0] == 0x21 and len(r) > 14 and r[14] == echo and ack is None:
                ack = bytes(r)
                ack_ms = dt
                log(f"[OK] {name} ack @{ack_ms:.0f}ms: {ack.hex(' ')}")
                if echo == 0x21:
                    log(f"    ack 体[0..7]: {ack[15:23].hex(' ')}")
            elif r[0] == 0x31:
                f31.append((dt, bytes(r)))
            else:
                other += 1
        log(f"    盯流 {watch_s:.1f}s: 0x31 {len(f31)} 帧 / 其他(多为0x30) {other} 包")
        for dt, f in f31:
            log(f"    [0x31 @{dt:.0f}ms] {f.hex(' ')}")
        if not f31:
            log("    （无 0x31——此窗口真机也不发异步状态帧）")
        return ack

    def collect31(self, sec, label):
        t0 = time.time()
        frames, other = [], 0
        while time.time() - t0 < sec:
            r = self.read(30)
            if r and r[0] == 0x31:
                frames.append(bytes(r))
            elif r:
                other += 1
        log(f"\n—— {label}: {len(frames)} 帧 0x31（其他 {other} 包）——")
        seen = set()
        for i, f in enumerate(frames):
            key = (len(f), f[49] if len(f) > 49 else -1,
                   f[56] if len(f) > 56 else -1)
            tag = f"len={len(f)} b49=0x{f[49]:02X} b52={f[52]:02X}{f[53]:02X} " \
                  f"b54={f[54]:02X}{f[55]:02X} b56=0x{f[56]:02X}" if len(f) > 56 \
                  else f"len={len(f)}"
            if i < 5 or key not in seen:
                seen.add(key)
                log(f"  [{i}] {tag}")
                log(f"      {f.hex(' ')}")
        if not frames:
            log("  （无 0x31 帧——与 v3 结论一致：本剧本真机不发 0x31）")
        return frames


# ---------------------------------------------------------------- 预检
def strain_hits(dev_frames):
    """0x30 第3帧acc块出现 [00 00][strain][00 20] 视为环数据在流里。"""
    hits, vals = 0, []
    for f in dev_frames:
        if len(f) < 50:
            continue
        b = f[37:43]
        if b[0] == 0x00 and b[1] == 0x00 and b[4] == 0x00 and b[5] == 0x20:
            v = b[2] | (b[3] << 8)
            if 500 <= v <= 6000:
                hits += 1
                vals.append(v)
    return hits, vals


def sample30(p, sec=0.8):
    """采一段 0x30 流：返回 (帧数, 第3帧acc块样例, 应变命中数, strain值表)。"""
    frames = []
    t0 = time.time()
    while time.time() - t0 < sec:
        r = p.read(20)
        if r:
            frames.append(bytes(r))
    blocks = [f[37:43].hex(" ") for f in frames if len(f) >= 50][:3]
    hits, vals = strain_hits(frames)
    return len(frames), blocks, hits, vals


def preflight(p):
    """等 JC 蓝牙连上 PC 并起流。环插没插只能靠人确认——真机的应变数据要等
    MCU 外设模式激活后才 mux 进 0x30 流（v3 日志实证：剧本前第3帧=裸加速度），
    剧本前没有自动检测环在位的手段。"""
    while True:
        if hid.enumerate(VID, PID):
            break
        wait_enter("找不到 Joy-Con (R)。请检查：① JC 已开机并唤醒（按一下任意键）"
                   "② Windows 蓝牙列表里显示「已连接」③ 没开 BetterJoy/Steam/pyjoycon。"
                   "好了之后回车重试。")
    while True:
        if p.open():
            break
        wait_enter("JC 已枚举但打开 HID 句柄失败（可能被别的程序占用）。"
                   "关掉占用后回车重试。")
    log(f"[预检] JC 已连接（{now()}）")
    # 起流：与剧本里游戏发的是同两条命令，幂等
    p.subcmd([0x03, 0x30])
    p.subcmd([0x40, 0x01])
    while True:
        n, blocks, _, _ = sample30(p)
        if n:
            break
        wait_enter("JC 连着但收不到 0x30 流（可能休眠了）。按一下 JC 按键唤醒，"
                   "回车重试。")
    log(f"[预检] 0x30 流正常（0.8s {n} 帧）")
    log(f"[预检] 剧本前第3帧acc块样例: {blocks}")
    log("    （若为裸加速度值 → 证实真机 strain 注入是 MCU 门控的，记下这份『前』照）")
    wait_enter("确认 Ring-Con 已插到 Joy-Con (R) 上（插到底）。环在位与否由剧本后"
               "的流采样复核，回车开始。")


# ---------------------------------------------------------------- 剧本数据
def mcu_frame(b2, b3):
    """0x21 MCU 帧：payload=[0x21,0x21,b2,b3,0*34,crc8(b2,b3+34零)]（v3 布局）。"""
    scope = bytes([b2, b3]) + bytes(34)
    return [0x21, 0x21, b2, b3] + [0] * 34 + [crc8_poly07(scope)]


def spi_read(addr, ln):
    """NS2 游戏实际发的 0x10 读（照抄日志字节）：[0x10, addr*4 LE, len, 0*33]。"""
    return [0x10, addr & 0xFF, (addr >> 8) & 0xFF, 0, 0, ln] + [0] * 33


def sub(name, payload):
    return (name, payload)


# 2026-09-10 11:37 冷启动 conn1 的 34 步，字节照抄会话日志
GAME_SEQ = (
    sub("0x02 设备信息", [0x02] + [0] * 38),
    sub("0x08 shipment", [0x08] + [0] * 38),
    sub("0x10 读 0x6000 序列号(16)", spi_read(0x6000, 0x10)),
    sub("0x10 读 0x6050 颜色(13)", spi_read(0x6050, 0x0D)),
    sub("0x03 输入模式 0x30", [0x03, 0x30] + [0] * 37),
    sub("0x04", [0x04] + [0] * 38),
    sub("0x10 读 0x6080(24)", spi_read(0x6080, 0x18)),
    sub("0x10 读 0x6098(18)", spi_read(0x6098, 0x12)),
    sub("0x10 读 0x8010(24)", spi_read(0x8010, 0x18)),
    sub("0x10 读 0x603D 摇杆L(25)", spi_read(0x603D, 0x19)),
    sub("0x10 读 0x6020 IMU(24) #1", spi_read(0x6020, 0x18)),
    sub("0x10 读 0x6020 IMU(24) #2", spi_read(0x6020, 0x18)),
    sub("0x10 读 0x6020 IMU(24) #3", spi_read(0x6020, 0x18)),
    sub("0x10 读 0x6020 IMU(24) #4", spi_read(0x6020, 0x18)),
    sub("0x40 六轴 on #1", [0x40, 0x01] + [0] * 37),
    sub("0x40 六轴 on #2", [0x40, 0x01] + [0] * 37),
    sub("0x30 灯 [00]", [0x30, 0x00] + [0] * 37),
    sub("0x48 震动 on #1", [0x48, 0x01] + [0] * 37),
    sub("0x48 震动 on #2", [0x48, 0x01] + [0] * 37),
    sub("0x22 resume #1", [0x22, 0x01] + [0] * 37),
    sub("0x22 resume #2", [0x22, 0x01] + [0] * 37),
    sub("0x22 resume #3", [0x22, 0x01] + [0] * 37),
    sub("0x22 resume #4", [0x22, 0x01] + [0] * 37),
    sub("0x30 灯 [01] #1", [0x30, 0x01] + [0] * 37),
    sub("0x30 灯 [01] #2", [0x30, 0x01] + [0] * 37),
    sub("0x30 灯 [01] #3", [0x30, 0x01] + [0] * 37),
    sub("0x30 灯 [01] #4", [0x30, 0x01] + [0] * 37),
    sub("0x30 灯 [01] #5", [0x30, 0x01] + [0] * 37),
    sub("0x21 mode-0 复位 #1", mcu_frame(0x00, 0x00)),
    sub("0x21 mode-0 复位 #2", mcu_frame(0x00, 0x00)),
    sub("0x48 震动 off #1", [0x48, 0x00] + [0] * 37),
    sub("0x48 震动 off #2", [0x48, 0x00] + [0] * 37),
    sub("0x48 震动 off #3", [0x48, 0x00] + [0] * 37),
    sub("0x48 震动 off #4", [0x48, 0x00] + [0] * 37),
)

# 我们固件对这几个区域的现值（疑点=FF 填充；校准区=joycontrol 镜像）
FW_SPI = {
    0x6000: "FF"*16 + "（FF 填充）",
    0x6050: "FF 45 54 25 26 26 FF 45 54 FF 45 54 00（joycontrol 镜像）",
    0x6080: "FF"*24 + "（FF 填充）",
    0x6098: "FF"*18 + "（FF 填充）",
    0x8010: "FF"*24 + "（FF 填充）",
    0x603D: "00 07 70 00 08 80 00 07 70 00 08 80 00 …（joycontrol 镜像）",
    0x6020: "76 00 A6 FE EA 02 00 40 00 40 00 40 0E …（joycontrol 镜像）",
}
SPI_NAME = {0x6000: "序列号", 0x6050: "颜色", 0x6080: "?", 0x6098: "?",
            0x8010: "?", 0x603D: "摇杆L校准", 0x6020: "IMU校准"}


def show_spi(addr):
    def _show(r):
        # ack: [13]=0x90 [14]=0x10 [15..19]=addr+len 回显, 数据从 [20] 起
        data = bytes(r[20:20 + 40])
        log(f"    → 真机 0x{addr:04X} 数据({SPI_NAME.get(addr, '?')}): {data.hex(' ')}")
    return _show


def show_mcu_body(tag):
    def _show(r):
        log(f"    → ack 体[0..7]: {r[15:23].hex(' ')}（{tag}）")
    return _show


# ---------------------------------------------------------------- 主流程
def main():
    global LOG
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "ring_probe31_log.txt")
    LOG = open(log_path, "a", encoding="utf-8")  # 追加：跨天不丢，头部有日期
    log("=" * 62)
    log(f" Ring-Con 真值采集 v4 —— {now()}")
    log("=" * 62)

    p = Probe()
    preflight(p)
    wait_enter("JC 与环就绪。接下来将向真机复刻 NS2 冷启动完整 34 步剧本"
               "（只读配置+复位 MCU，与游戏行为一致，无破坏性）。")

    log(f"\n—— A. NS2 冷启动剧本逐条复刻（{now()}）——")
    for name, payload in GAME_SEQ:
        echo = payload[0]
        if payload[0] == 0x10:
            addr = payload[1] | (payload[2] << 8)
            p.step(name, payload, echo, show=show_spi(addr))
        elif payload[0] == 0x21:
            p.step_watch(name, payload, echo)  # mode-0：盯异步 0x31（v5 核心）
        elif payload[0] == 0x22:
            p.step_watch(name, payload, echo)  # resume：第一个 0x22 窗口也要盯
        else:
            p.step(name, payload, echo)

    log(f"\n—— B. 后段剧本（游戏 mode-0 放行后才发；采真机 ack，{now()}）——")
    p.step_watch("0x21 设模式3 (00 03)", mcu_frame(0x00, 0x03), 0x21)
    p.step_watch("0x21 external ready (01 01)", mcu_frame(0x01, 0x01), 0x21)
    p.step_watch("0x59 GetExtDevInfo", [0x59], 0x59)
    p.step("0x40 Ringcon IMU (0x03)", [0x40, 0x03], 0x40)
    p.step("0x40 IMU (0x01)", [0x40, 0x01], 0x40)
    FMT_CFG_5C = [0x5C, 0x06, 0x03, 0x25, 0x06, 0x00, 0x00, 0x00, 0x00,
                  0x1C, 0x16, 0xED, 0x34, 0x36, 0x00, 0x00, 0x00, 0x0A,
                  0x64, 0x0B, 0xE6, 0xA9, 0x22, 0x00, 0x00, 0x04, 0x00,
                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0xA8, 0xE1,
                  0x34, 0x36]
    p.step("0x5C 格式配置", FMT_CFG_5C, 0x5C)
    p.step("0x5A 轮询使能", [0x5A, 0x04, 0x01, 0x01, 0x02], 0x5A)

    log(f"\n—— C. 0x11 请求 MCU 数据 ×2（复核精确序列下仍无 0x31，{now()}）——")
    p.out(0x11, [0x01])
    p.collect31(4, "0x11 后")
    p.out(0x11, [0x01])
    p.collect31(4, "0x11 后 #2")

    log(f"\n—— D. 流采样与 strain 门控对照（{now()}）——")
    n, blocks, hits, vals = sample30(p, 5)
    med = sorted(vals)[len(vals) // 2] if vals else None
    log(f"0x30 流 {n} 包（5s） / 第3帧acc块样例: {blocks}")
    if hits:
        log(f"应变签名 {hits} 帧，strain≈{med} → 剧本后 strain 已被真机 mux 进流")
        log("    对照：剧本前=裸加速度（预检样例）→ 证实 strain 注入是 MCU 门控的；"
            "我们固件却从上电就注入——这就是新的头号嫌疑，固件应改为门控注入")
    else:
        log("剧本后 5s 内 0 帧应变签名——门控假设证伪或环没插好，把日志发回")

    log(f"\n—— E. SPI 真值汇总（重点：FF 填充区）——")
    log("对照表：上面 A 段各 0x10 读的「真机数据」 vs 我们固件现值：")
    for addr, fw in FW_SPI.items():
        log(f"  0x{addr:04X} {SPI_NAME.get(addr, '?'):　<6} 固件现值: {fw}")
    log("若真机在 0x6000/0x6080/0x6098/0x8010 返回非 FF 数据 → 固件照抄后冷启动复测。")

    log(f"\n日志在 {log_path}（本次 run 头部时间 {now()}）")
    try:
        input("\n按回车退出...")
    except EOFError:
        pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log(f"\n[中断] 用户中止（{now()}）")
        sys.exit(1)
