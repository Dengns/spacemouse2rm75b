"""UDP 实时反馈频率测试

用途：
  实测 RM75B 通过 UDP 主动推送状态的频率。
  配置文件里 UDP_CYCLE_MS=5 表示期望 200Hz，本工具用来验证机械臂端
  是否真按这个周期推送，以及网络/Python 这边能否稳定收到。

用法：
  python3 udp_rate_test.py
  机械臂上电、网络连通后启动即可，无需移动机械臂。

输出列：
  1s rate(Hz)       最近 1 秒收到的 UDP 帧数
  5s rate(Hz)       最近 5 秒平均频率
  峰值/谷值(Hz)     整次运行的 1s rate 最大/最小值
  最小/最大间隔(ms) 相邻帧的最小/最大时间差（抖动指标）
  中位间隔(ms)      最近帧的典型周期
  总帧               累计帧数

判断标准：
  期望周期 cfg.UDP_CYCLE_MS = 5ms → 期望频率 200Hz
  - 频率达到期望 90% 以上、抖动 < 2ms：网络与机械臂状态健康
  - 频率明显偏低 / 最大间隔忽大忽小：检查网线、Wi-Fi、目标 IP/端口

Ctrl+C 退出。
"""

import argparse
import signal
import sys
import threading
import time
from collections import deque

from Robotic_Arm.rm_robot_interface import rm_realtime_arm_state_callback_ptr

import config_admittance as cfg
from rm75b import RM75BInterface


class MeasuringRM75B(RM75BInterface):
    """复用 RM75BInterface，但替换 UDP 回调为带时间戳计数的版本。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._frame_times = deque(maxlen=50000)
        self._frame_count = 0
        self._measure_lock = threading.Lock()

    def register_udp_feedback_callback(self):
        def _on_udp_state(data):
            now = time.monotonic()
            # 不解析全部 dict，避免拖慢回调；只确认数据到达
            with self._measure_lock:
                self._frame_times.append(now)
                self._frame_count += 1

        self._udp_callback = rm_realtime_arm_state_callback_ptr(_on_udp_state)
        self.arm.rm_realtime_arm_state_call_back(self._udp_callback)
        print("UDP measuring callback registered.")

    def snapshot(self):
        with self._measure_lock:
            return list(self._frame_times), self._frame_count


def fmt(value, spec, fallback="--"):
    return format(value, spec) if value is not None else fallback


def main():
    parser = argparse.ArgumentParser(description="RM75B UDP feedback rate test")
    parser.add_argument("--ip", default=cfg.ROBOT_IP)
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT)
    parser.add_argument("--cycle-ms", type=int, default=cfg.UDP_CYCLE_MS,
                        help="UDP 广播周期，5ms 的倍数")
    parser.add_argument("--no-force", action="store_true",
                        help="关闭力传感器数据上报（验证力是否拖慢 UDP）")
    args = parser.parse_args()

    force_coord = -1 if args.no_force else cfg.UDP_FORCE_COORDINATE

    arm = MeasuringRM75B(args.ip, args.port, enable_gripper=False)
    arm.enable_udp_realtime_feedback(
        cfg.UDP_TARGET_IP,
        cfg.UDP_TARGET_PORT,
        args.cycle_ms,
        force_coord,
    )
    arm.register_udp_feedback_callback()

    expected_hz = 1000.0 / args.cycle_ms
    expected_period_ms = args.cycle_ms
    force_label = "关闭" if args.no_force else f"开启(coord={force_coord})"
    print(f"\n期望频率: {expected_hz:.1f} Hz (周期 {expected_period_ms} ms) | 力上报: {force_label}")
    print("等待 UDP 帧... Ctrl+C 退出\n")

    def _quit(sig, frame):
        try:
            arm.close()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _quit)

    header = (
        f"{'1s(Hz)':>8} {'5s(Hz)':>8} {'峰值':>6} {'谷值':>6} "
        f"{'最小间隔ms':>12} {'最大间隔ms':>12} {'中位ms':>10} {'总帧':>10}"
    )
    print(header)
    print("-" * len(header.replace(" ", "")) * 2)

    peak_rate = 0
    valley_rate = None
    global_min_ms = None
    global_max_ms = None

    while True:
        time.sleep(0.5)
        times, total = arm.snapshot()
        now = time.monotonic()

        rate_1s = sum(1 for t in times if now - t <= 1.0)
        rate_5s = sum(1 for t in times if now - t <= 5.0) / 5.0
        peak_rate = max(peak_rate, rate_1s)
        # 谷值只在有数据时更新，避免启动期把 0 当谷值
        if rate_1s > 0:
            valley_rate = rate_1s if valley_rate is None else min(valley_rate, rate_1s)

        sample = times[-500:] if len(times) >= 2 else times
        intervals_ms = [
            (sample[i + 1] - sample[i]) * 1000.0
            for i in range(len(sample) - 1)
        ]
        if intervals_ms:
            cur_min = min(intervals_ms)
            cur_max = max(intervals_ms)
            cur_med = sorted(intervals_ms)[len(intervals_ms) // 2]
            global_min_ms = cur_min if global_min_ms is None else min(global_min_ms, cur_min)
            global_max_ms = cur_max if global_max_ms is None else max(global_max_ms, cur_max)
        else:
            cur_med = None

        if rate_1s == 0:
            sys.stdout.write(
                f"\r{'idle':>8} {'--':>8} {peak_rate:6d} "
                f"{fmt(valley_rate, '6d'):>6} "
                f"{fmt(global_min_ms, '12.2f'):>12} "
                f"{fmt(global_max_ms, '12.2f'):>12} "
                f"{'--':>10} {total:10d}     "
            )
        else:
            sys.stdout.write(
                f"\r{rate_1s:8d} {rate_5s:8.1f} {peak_rate:6d} "
                f"{fmt(valley_rate, '6d'):>6} "
                f"{fmt(global_min_ms, '12.2f'):>12} "
                f"{fmt(global_max_ms, '12.2f'):>12} "
                f"{fmt(cur_med, '10.2f'):>10} {total:10d}     "
            )
        sys.stdout.flush()


if __name__ == "__main__":
    main()
