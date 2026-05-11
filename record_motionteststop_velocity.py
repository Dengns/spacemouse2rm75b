"""
Velocity-mode motion recorder for RM75B.

What it does:
- Reads SpaceMouse axes
- Streams Cartesian velocity via rm_movev_canfd
- Logs command velocity + target-integrated pose + actual pose to CSV
- Auto-generates a PNG summary figure after saving

Usage:
  python3 record_motionteststop_velocity.py --ip 192.168.5.105
  python3 record_motionteststop_velocity.py --out outputs/velocity_run.csv --hz 100
  python3 record_motionteststop_velocity.py --feedback udp --udp-target-ip 192.168.1.104 --out outputs/velocity_udp.csv
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

import config as cfg
from rm75b import RM75BInterface
from spacemouse_input import SpaceMouseReader
from udp_feedback import enable_udp_feedback, read_udp_feedback, wait_for_udp_pose


class ArmPoller(threading.Thread):
    def __init__(self, arm_interface, hz=200):
        super().__init__(daemon=True)
        self._arm = arm_interface
        self._interval = 1.0 / hz
        self._lock = threading.Lock()
        self._pose = None
        self._vel = None
        self._ts = None
        self._last_pose = None
        self._last_ts = None
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            ret, state = self._arm.arm.rm_get_current_arm_state()
            if ret == 0:
                now = time.perf_counter()
                pose = np.array(state["pose"], dtype=float)
                vel = np.zeros(6)
                if self._last_pose is not None and self._last_ts is not None:
                    dt = now - self._last_ts
                    if dt > 1e-6:
                        vel = (pose - self._last_pose) / dt
                with self._lock:
                    self._pose = pose
                    self._vel = vel
                    self._ts = now
                self._last_pose = pose
                self._last_ts = now
            time.sleep(self._interval)

    def get_state(self):
        with self._lock:
            pose = self._pose.copy() if self._pose is not None else None
            vel = self._vel.copy() if self._vel is not None else None
            ts = self._ts
            return pose, vel, ts

    def stop(self):
        self._stop.set()


def _ensure_output_path(path):
    out_dir = os.path.dirname(path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)


def _build_velocity(raw_axes, smoothed):
    mapped = np.array([raw_axes[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)], dtype=float)
    smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * smoothed

    vel = np.zeros(6)
    vel[:3] = smoothed[:3] * cfg.TRANSLATION_VEL_SCALE
    vel[3:] = smoothed[3:] * cfg.ROTATION_VEL_SCALE

    vel[:3] = np.clip(vel[:3], -cfg.MAX_LINEAR_VEL, cfg.MAX_LINEAR_VEL)
    vel[3:] = np.clip(vel[3:], -cfg.MAX_ANGULAR_VEL, cfg.MAX_ANGULAR_VEL)

    vel *= np.array(cfg.AXIS_ENABLE, dtype=float)
    return vel, smoothed


def _clamp_workspace(target_pose):
    for i in range(3):
        target_pose[i] = np.clip(target_pose[i], cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i])


def _save_csv(path, rows):
    _ensure_output_path(path)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t",
            "inp_mag",
            "feedback_age_ms",
            "cmd_vx", "cmd_vy", "cmd_vz",
            "cmd_wx", "cmd_wy", "cmd_wz",
            "act_vx", "act_vy", "act_vz",
            "act_wx", "act_wy", "act_wz",
            "tgt_x", "tgt_y", "tgt_z",
            "tgt_rx", "tgt_ry", "tgt_rz",
            "act_x", "act_y", "act_z",
            "act_rx", "act_ry", "act_rz",
        ])
        writer.writerows(rows)


def _auto_plot(csv_path):
    cmd = [sys.executable, "plot_motion.py", csv_path, "--no-show"]
    try:
        subprocess.run(cmd, check=True)
    except Exception as e:
        print(f"[WARN] Failed to auto-plot: {e}")


def _udp_cfg_from_args(args):
    return SimpleNamespace(
        UDP_TARGET_IP=args.udp_target_ip,
        UDP_TARGET_PORT=args.udp_target_port,
        UDP_CYCLE_MS=args.udp_cycle_ms,
        UDP_FORCE_COORDINATE=args.udp_force_coordinate,
    )


def _estimate_velocity(pose, pose_ts, last_pose, last_pose_ts):
    vel = np.zeros(6)
    if pose is not None and last_pose is not None and last_pose_ts is not None:
        dt = pose_ts - last_pose_ts
        if dt > 1e-6:
            vel = (pose - last_pose) / dt
    return vel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=cfg.ROBOT_IP)
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT)
    parser.add_argument("--hz", type=int, default=100, help="Control loop rate in Hz")
    parser.add_argument(
        "--feedback",
        choices=("tcp", "udp"),
        default="tcp",
        help="Actual pose feedback source for error logging",
    )
    parser.add_argument("--udp-target-ip", default=cfg.UDP_TARGET_IP, help="Local PC IP receiving robot UDP")
    parser.add_argument("--udp-target-port", type=int, default=cfg.UDP_TARGET_PORT)
    parser.add_argument("--udp-cycle-ms", type=int, default=cfg.UDP_CYCLE_MS)
    parser.add_argument("--udp-force-coordinate", type=int, default=cfg.UDP_FORCE_COORDINATE)
    parser.add_argument("--udp-timeout", type=float, default=cfg.UDP_TIMEOUT_S)
    parser.add_argument("--udp-wait", type=float, default=1.5, help="Seconds to wait for first UDP pose")
    parser.add_argument(
        "--out",
        default=os.path.join(cfg.OUTPUT_DIR, "motion_velocity.csv"),
        help="Output CSV path",
    )
    args = parser.parse_args()

    mouse = SpaceMouseReader()
    poller = None
    arm = None
    records = []

    try:
        mouse.open()
        mouse.start()

        arm = RM75BInterface(args.ip, args.port, enable_gripper=False)

        if not hasattr(arm.arm, "rm_set_movev_canfd_init") or not hasattr(arm.arm, "rm_movev_canfd"):
            raise RuntimeError("SDK does not expose rm_set_movev_canfd_init/rm_movev_canfd")

        if args.feedback == "udp":
            enable_udp_feedback(arm, _udp_cfg_from_args(args))

        dt = 1.0 / float(args.hz)
        sdk_dt_ms = int(round(dt * 1000))
        init_ret = arm.arm.rm_set_movev_canfd_init(
            int(cfg.MOVEV_AVOID_SINGULARITY),
            int(cfg.MOVEV_FRAME_TYPE),
            sdk_dt_ms,
        )
        if init_ret != 0:
            raise RuntimeError(f"rm_set_movev_canfd_init failed (ret={init_ret})")

        if args.feedback == "udp":
            initial_pose = wait_for_udp_pose(
                arm,
                args.udp_timeout,
                wait_s=args.udp_wait,
            )
            if initial_pose is None:
                raise RuntimeError("Failed to read initial pose from UDP feedback")
            target_pose = initial_pose.copy()
            print(f"Start pose from UDP: {np.round(target_pose, 4).tolist()}")
        else:
            ret, state = arm.arm.rm_get_current_arm_state()
            if ret != 0:
                raise RuntimeError("Failed to read initial pose")
            target_pose = np.array(state["pose"], dtype=float)
            print(f"Start pose from TCP: {np.round(target_pose, 4).tolist()}")

        print(
            f"Velocity mode init: dt={dt:.4f}s, frame_type={cfg.MOVEV_FRAME_TYPE}, "
            f"avoid_singularity={cfg.MOVEV_AVOID_SINGULARITY}, feedback={args.feedback.upper()}"
        )

        if args.feedback == "tcp":
            poller = ArmPoller(arm, hz=cfg.ARM_STATE_POLL_HZ)
            poller.start()
            time.sleep(0.2)

        t_start = time.perf_counter()
        smoothed = np.zeros(6)
        last_udp_pose = None
        last_udp_pose_ts = None
        consecutive_fails = 0
        print(f"Recording velocity mode at {args.hz} Hz ... Press Ctrl+C to stop.\n")

        stopping = {"flag": False}

        def _stop_handler(*_):
            stopping["flag"] = True

        signal.signal(signal.SIGINT, _stop_handler)

        while not stopping["flag"]:
            loop_t0 = time.monotonic()
            now = time.perf_counter() - t_start

            raw = mouse.get_axes()
            inp_mag = max(abs(v) for v in raw)

            if inp_mag < cfg.DEADZONE:
                vel_cmd = np.zeros(6)
            else:
                vel_cmd, smoothed = _build_velocity(raw, smoothed)

            target_pose += vel_cmd * dt
            _clamp_workspace(target_pose)

            ret = arm.arm.rm_movev_canfd(
                vel_cmd.tolist(),
                True,  # follow
                int(cfg.MOVEV_TRAJECTORY_MODE),
                int(cfg.MOVEV_RADIO),
            )
            if ret != 0:
                consecutive_fails += 1
                if consecutive_fails >= 10:
                    print("[ERROR] 10 consecutive send failures - emergency stop")
                    arm.arm.rm_set_arm_stop()
                    break
            else:
                consecutive_fails = 0

            feedback_age = None
            if args.feedback == "udp":
                _, actual, feedback_age = read_udp_feedback(arm, args.udp_timeout)
                actual_ts = time.perf_counter()
                actual_vel = _estimate_velocity(actual, actual_ts, last_udp_pose, last_udp_pose_ts)
                if actual is not None:
                    last_udp_pose = actual.copy()
                    last_udp_pose_ts = actual_ts
            else:
                actual, actual_vel, _ = poller.get_state()

            if actual is not None and actual_vel is not None:
                records.append([
                    round(now, 5),
                    inp_mag,
                    "" if feedback_age is None else round(feedback_age * 1000, 3),
                    *[round(v, 6) for v in vel_cmd],
                    *[round(v, 6) for v in actual_vel],
                    *[round(v, 6) for v in target_pose],
                    *[round(v, 6) for v in actual],
                ])
                err_mm = np.linalg.norm(target_pose[:3] - actual[:3]) * 1000
                speed_mm = np.linalg.norm(vel_cmd[:3]) * 1000
                act_speed_mm = np.linalg.norm(actual_vel[:3]) * 1000
                age_str = ""
                if feedback_age is not None:
                    age_str = f"  udp_age={feedback_age * 1000:5.1f}ms"
                print(
                    f"\r  t={now:6.2f}s  input={inp_mag:4.0f}  v_cmd={speed_mm:6.2f}mm/s  v_act={act_speed_mm:6.2f}mm/s  "
                    f"error={err_mm:5.2f}mm{age_str}  frames={len(records):5d}",
                    end="",
                    flush=True,
                )

            elapsed = time.monotonic() - loop_t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

        # Stop motion cleanly by sending zero velocity once before shutdown.
        if arm is not None:
            try:
                arm.arm.rm_movev_canfd([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False,
                                       int(cfg.MOVEV_TRAJECTORY_MODE), int(cfg.MOVEV_RADIO))
            except Exception:
                pass

    finally:
        print(f"\nStopping. Saving {len(records)} frames to {args.out} ...")
        _save_csv(args.out, records)
        print("Saved CSV.")
        _auto_plot(args.out)

        if poller is not None:
            poller.stop()
        if mouse is not None:
            try:
                mouse.stop()
            except Exception:
                pass
        if arm is not None:
            arm.close()


if __name__ == "__main__":
    main()
