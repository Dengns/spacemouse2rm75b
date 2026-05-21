"""SpaceMouse 物理上报频率测试

用途：
  实测 SpaceMouse 设备硬件层面的 motion event 上报率。
  spnav_poll_event 是事件驱动的——设备没移动就不发事件。
  只有持续晃动 SpaceMouse 时，下面打印的数字才反映真实硬件上限。

用法：
  python3 spacemouse_rate_test.py
  然后持续地、用力地晃动 SpaceMouse（前后左右上下 + 旋转，越剧烈越好）。

输出列：
  1s rate      最近 1 秒事件数  (Hz)
  5s rate      最近 5 秒平均事件数 (Hz)
  峰值Hz       本次运行观测到的 1s rate 峰值
  最小间隔ms   最近 200 个事件中相邻事件最小时间差（接近硬件周期）
  总事件       从启动到现在累计的 motion event 数

判断标准：
  典型 3DConnexion SpaceMouse 硬件采样率：60 ~ 100 Hz
  → 最小间隔约 10 ~ 16 ms
  如果峰值 < 50Hz 且最小间隔 > 20ms，多半是设备/驱动有问题。

Ctrl+C 退出。
"""

import ctypes
import signal
import sys
import time
from collections import deque

from spacemouse_input import (
    SpaceMouseReader,
    SpnavEvent,
    SPNAV_EVENT_MOTION,
    SPNAV_EVENT_BUTTON,
)


class MeasuringSpaceMouseReader(SpaceMouseReader):
    """子类化：在 _poll_loop 里给每个 motion event 打时间戳。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._event_times = deque(maxlen=20000)  # ring buffer 防内存涨
        self._event_count = 0

    def _poll_loop(self):
        ev = SpnavEvent()
        while not self._stop_event.is_set():
            ret = self._lib.spnav_poll_event(ctypes.byref(ev))
            if ret == 0:
                time.sleep(0.001)
                continue

            now = time.monotonic()

            if ev.type == SPNAV_EVENT_MOTION:
                m = ev.motion
                raw_axes = [m.x, m.y, m.z, m.rx, m.ry, m.rz]
                # 跳过原版的 raise，避免测试时被打断
                axes = [self._apply_deadzone(v) for v in raw_axes]
                with self._lock:
                    self._axes = axes
                    self._event_times.append(now)
                    self._event_count += 1

            elif ev.type == SPNAV_EVENT_BUTTON:
                b = ev.button
                pressed = bool(b.press)
                with self._lock:
                    self._buttons[b.bnum] = pressed
                    self._button_events.append((b.bnum, pressed))

    def snapshot(self):
        with self._lock:
            return list(self._event_times), self._event_count


def fmt(value, spec, fallback="--"):
    return format(value, spec) if value is not None else fallback


def main():
    reader = MeasuringSpaceMouseReader()
    reader.open()
    reader.start()

    def _quit(sig, frame):
        reader.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _quit)

    print("\n请持续晃动 SpaceMouse 进行测量... Ctrl+C 退出\n")
    print(f"{'1s rate(Hz)':>12} {'5s rate(Hz)':>12} {'峰值(Hz)':>10} "
          f"{'最小间隔(ms)':>14} {'中位间隔(ms)':>14} {'总事件':>10}")
    print("-" * 80)

    max_rate = 0
    overall_min_interval_ms = None

    while True:
        time.sleep(0.5)
        times, total = reader.snapshot()
        now = time.monotonic()

        # 滑动窗口统计
        recent_1s = [t for t in times if now - t <= 1.0]
        recent_5s = [t for t in times if now - t <= 5.0]
        rate_1s = len(recent_1s)
        rate_5s = len(recent_5s) / 5.0
        max_rate = max(max_rate, rate_1s)

        # 用最近 200 个事件估计硬件周期
        sample = times[-200:] if len(times) >= 2 else times
        intervals_ms = [
            (sample[i + 1] - sample[i]) * 1000.0
            for i in range(len(sample) - 1)
        ]
        if intervals_ms:
            cur_min = min(intervals_ms)
            cur_median = sorted(intervals_ms)[len(intervals_ms) // 2]
            if overall_min_interval_ms is None or cur_min < overall_min_interval_ms:
                overall_min_interval_ms = cur_min
        else:
            cur_median = None

        # 闲置提示
        if rate_1s == 0:
            sys.stdout.write(
                f"\r{'idle':>12} {'--':>12} {max_rate:10d} "
                f"{fmt(overall_min_interval_ms, '14.2f'):>14} "
                f"{'--':>14} {total:10d}     "
            )
        else:
            sys.stdout.write(
                f"\r{rate_1s:12d} {rate_5s:12.1f} {max_rate:10d} "
                f"{fmt(overall_min_interval_ms, '14.2f'):>14} "
                f"{fmt(cur_median, '14.2f'):>14} {total:10d}     "
            )
        sys.stdout.flush()


if __name__ == "__main__":
    main()
