"""
Motion recorder: logs SpaceMouse input, commanded target pose, and actual arm end-effector pose.

CSV columns:
  t            - timestamp (s from start)
  inp_mag      - SpaceMouse input magnitude (max absolute raw axis value)
  tgt_x/y/z   - commanded target position (m)
  tgt_rx/ry/rz - commanded target orientation (rad)
  act_x/y/z   - actual arm end-effector position (m)
  act_rx/ry/rz - actual arm end-effector orientation (rad)

Usage:
  python record_motion.py [--ip 192.168.5.105] [--out motion.csv]
  Press Ctrl+C to stop and save.
"""

import argparse
import csv
import signal
import sys
import time

import numpy as np

import config as cfg
from spacemouse_input import SpaceMouseReader
from rm75b import RM75BInterface

import threading


class ArmPoller(threading.Thread):
    def __init__(self, arm_interface, hz=200):
        super().__init__(daemon=True)
        self._arm = arm_interface
        self._interval = 1.0 / hz
        self._lock = threading.Lock()
        self._pose = None
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            ret, state = self._arm.arm.rm_get_current_arm_state()
            if ret == 0:
                with self._lock:
                    self._pose = np.array(state["pose"], dtype=float)
            time.sleep(self._interval)

    def get_pose(self):
        with self._lock:
            return self._pose.copy() if self._pose is not None else None

    def stop(self):
        self._stop.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",   default=cfg.ROBOT_IP)
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT)
    parser.add_argument("--out",  default="motion.csv")
    args = parser.parse_args()

    mouse = SpaceMouseReader()
    mouse.open()
    mouse.start()

    arm = RM75BInterface(args.ip, args.port, enable_gripper=False)

    ret, state = arm.arm.rm_get_current_arm_state()
    if ret != 0:
        raise RuntimeError("Failed to read initial pose")
    target_pose = np.array(state["pose"], dtype=float)
    print(f"Start pose: {np.round(target_pose, 4).tolist()}")

    poller = ArmPoller(arm, hz=100)
    poller.start()
    time.sleep(0.2)

    records = []
    t_start = time.perf_counter()
    smoothed = np.zeros(6)
    consecutive_fails = 0

    def _sigint(*_):
        print(f"\nStopping. Saving {len(records)} frames to {args.out} ...")
        with open(args.out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "t",
                "inp_mag",
                "tgt_x", "tgt_y", "tgt_z",
                "tgt_rx", "tgt_ry", "tgt_rz",
                "act_x", "act_y", "act_z",
                "act_rx", "act_ry", "act_rz",
            ])
            writer.writerows(records)
        print("Saved.")
        poller.stop()
        mouse.stop()
        arm.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    control_hz = 50
    dt = 1.0 / control_hz
    print(f"Recording at {control_hz} Hz ... Press Ctrl+C to stop.\n")

    while True:
        t0 = time.monotonic()
        now = time.perf_counter() - t_start

        raw = mouse.get_axes()
        inp_mag = max(abs(v) for v in raw)

        # if inp_mag < cfg.DEADZONE:
        #     arm.arm.rm_set_arm_stop()
        #     print(f"\r  t={now:6.2f}s  input={inp_mag:4.0f}  [STOPPED - no input]          ",
        #           end="", flush=True)
        #     elapsed = time.monotonic() - t0
        #     sleep_t = dt - elapsed
        #     if sleep_t > 0:
        #         time.sleep(sleep_t)
        #     continue

        mapped = np.array([raw[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)], dtype=float)
        smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * smoothed
        delta = np.zeros(6)
        delta[:3] = smoothed[:3] * cfg.TRANSLATION_SCALE
        delta[3:] = smoothed[3:] * cfg.ROTATION_SCALE
        delta[:3] = np.clip(delta[:3], -cfg.MAX_TRANSLATION_PER_CYCLE, cfg.MAX_TRANSLATION_PER_CYCLE)
        delta[3:] = np.clip(delta[3:], -cfg.MAX_ROTATION_PER_CYCLE, cfg.MAX_ROTATION_PER_CYCLE)
        delta *= np.array(cfg.AXIS_ENABLE, dtype=float)

        target_pose += delta
        for i in range(3):
            target_pose[i] = np.clip(target_pose[i], cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i])

        # ret = arm.arm.rm_movep_canfd(target_pose.tolist(), follow=True)
        ret = arm.arm.rm_movep_follow(target_pose.tolist())
        if ret != 0:
            consecutive_fails += 1
            if consecutive_fails >= 10:
                print("[ERROR] 10 consecutive send failures - emergency stop")
                arm.arm.rm_set_arm_stop()
                break
        else:
            consecutive_fails = 0

        actual = poller.get_pose()
        if actual is not None:
            records.append([
                round(now, 5),
                inp_mag,
                *[round(v, 6) for v in target_pose],
                *[round(v, 6) for v in actual],
            ])

        if actual is not None:
            err_mm = np.linalg.norm(target_pose[:3] - actual[:3]) * 1000
            print(f"\r  t={now:6.2f}s  input={inp_mag:4.0f}  error={err_mm:5.2f}mm  frames={len(records):5d}",
                  end="", flush=True)

        elapsed = time.monotonic() - t0
        sleep_t = dt - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)


if __name__ == "__main__":
    main()
