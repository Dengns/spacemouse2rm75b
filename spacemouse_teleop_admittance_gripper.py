"""SpaceMouse teleoperation for RM75B — admittance(力位混合导纳) + gripper 合并版.

合并自:
  - spacemouse_teleop_admittance.py  (UDP 实时反馈 + 导纳外环 MBK + rm_force_position_move 力位混合)
  - spacemouse_teleop_switch.py (LJL)  (夹爪 incremental/binary/Switching/disabled + 启动位置读取同步)

运动/导纳逻辑完整保留: UDP 高频六维力 -> admittance MBK 柔顺偏移 -> rm_force_position_move 下发。
夹爪控制采用 LJL 版: 启动时读真实位置同步基准(避免假设全开导致的错位), 支持 disabled 模式与退出开关。

关键不变量(合并时务必保持):
  * UDP 高频六维力是导纳的输入, 不能删;
  * 运动指令用 rm_force_position_move(力位混合), 不能退回 rm_movep_canfd(纯位置);
  * setup 必须保留 rm_clear_force_data + rm_start_force_position_move + wait_for_udp_pose。
"""

import sys
import argparse
import signal
import time
from collections import deque

import numpy as np

import config_admittance_gripper as cfg
from spacemouse_input import SpaceMouseReader
from rm75b import RM75BInterface
from class_switch import USBRelayController
from udp_feedback import enable_udp_feedback, read_udp_feedback, wait_for_udp_pose
import Robotic_Arm.rm_robot_interface as rm


class SpaceMouseTeleop(USBRelayController):
    """Admittance teleop (force-position) + gripper control."""

    def __init__(self, ip: str, port: int):
        super().__init__(
            serial_port=cfg.SERIAL_PORT,
            baud_rate=cfg.SERIAL_BAUD_RATE,
            timeout=cfg.SERIAL_TIMEOUT,
        )

        self.ip = ip
        self.port = port

        self.mouse = None
        self.arm = None

        self.target_pose = None          # [x, y, z, rx, ry, rz] (m / rad)
        self._smoothed = np.zeros(6)     # EMA state
        self._consecutive_fails = 0
        self._force_payload_warned = False
        self._gripper_pos = None         # 当前夹爪位置 (0~1000), None=未同步
        self._relay_ready = False

        self.control_mode = [3, 3, 3, 3, 3, 3]
        self.desired_force = [0, 0, 5, 0, 0, 0]

        # 柔顺控制
        self._adm_offset = np.zeros(6)   # 因为柔顺控制额外产生的方向位移
        self._adm_vel = np.zeros(6)      # admittance velocity

        # UDP 实际频率测量
        self._udp_last_ts = 0.0          # arm._latest_udp_time 上次值
        self._udp_frame_times = deque(maxlen=1000)  # 收到新帧的本地时间戳（1s 滑窗）
        self._udp_frame_total = 0        # 累计新帧数

    # ------ setup / teardown ------

    def setup(self):
        # 1. SpaceMouse
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()

        # 2. Robot arm + gripper (Switching/disabled 模式不启用夹爪 Modbus)
        self.arm = RM75BInterface(
            self.ip, self.port,
            enable_gripper=(cfg.GRIPPER_MODE not in ("Switching", "disabled")),
        )

        # 启动时读取真实夹爪位置, 同步基准(避免原仓假设全开导致的错位)
        if cfg.GRIPPER_MODE not in ("Switching", "disabled"):
            pos = self.arm.get_gripper_position()
            if pos is not None:
                self._gripper_pos = float(pos)
                print(f"Gripper start pos: {int(round(self._gripper_pos))}")

        if cfg.UDP_FEEDBACK_ENABLE:
            enable_udp_feedback(self.arm, cfg)

        if cfg.GRIPPER_MODE == "Switching":
            try:
                self.connect()
                self._relay_ready = True
            except Exception as e:
                self._relay_ready = False
                print(f"[WARN] Relay connect failed: {e}")
        # 将六维力数据清零，标定当前状态下的零位
        ret_clear = self.arm.arm.rm_clear_force_data()
        if isinstance(ret_clear, int) and ret_clear != 0:
            raise RuntimeError(f"rm_clear_force_data failed (ret={ret_clear})")
        # 开启透传力位混合控制补偿模式
        ret_start = self.arm.arm.rm_start_force_position_move()
        if isinstance(ret_start, int) and ret_start != 0:
            raise RuntimeError(f"rm_start_force_position_move failed (ret={ret_start})")
        # 3. Read current pose as starting point
        if cfg.UDP_FEEDBACK_ENABLE:
            current_pose = wait_for_udp_pose(self.arm, cfg.UDP_TIMEOUT_S)
            self.target_pose = current_pose.copy() if current_pose is not None else None
            if self.target_pose is None:
                raise RuntimeError("Failed to read initial pose from UDP feedback")
            print(f"Start pose from UDP: {np.round(self.target_pose, 4).tolist()}")
        else:
            ret, state = self.arm.arm.rm_get_current_arm_state()
            if ret != 0:
                raise RuntimeError(f"Failed to read arm state (ret={ret})")
            # state["pose"] = [x, y, z, rx, ry, rz] (m / rad)
            self.target_pose = np.array(state["pose"], dtype=float)
            print(f"Start pose: {np.round(self.target_pose, 4).tolist()}")

    def teardown(self, slow_stop: bool = True):
        """Clean shutdown: stop arm, open gripper (optional), disconnect."""
        if self.arm is not None:
            if slow_stop:
                self.arm.arm.rm_set_arm_slow_stop()
            if cfg.GRIPPER_OPEN_ON_EXIT and cfg.GRIPPER_MODE not in ("Switching", "disabled"):
                self.arm.set_gripper_position(cfg.GRIPPER_OPEN_POS)
                time.sleep(0.3)
            self.arm.close()
        if cfg.GRIPPER_MODE == "Switching":
            try:
                self.disconnect()
            except Exception as e:
                print(f"[WARN] Relay disconnect failed: {e}")
        if self.mouse is not None:
            self.mouse.stop()
        print("Shutdown complete.")

    # ------ control helpers ------

    def _compute_delta(self, raw_axes):
        """Apply axis mapping, sign flip, EMA smoothing, scaling, and clamping."""
        mapped = np.array([raw_axes[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)],
                          dtype=float)

        # EMA smoothing
        self._smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * self._smoothed

        # Scale to physical units
        delta = np.zeros(6)
        delta[:3] = self._smoothed[:3] * cfg.TRANSLATION_SCALE
        delta[3:] = self._smoothed[3:] * cfg.ROTATION_SCALE

        # Per-axis hard clamp
        delta[:3] = np.clip(delta[:3], -cfg.MAX_TRANSLATION_PER_CYCLE, cfg.MAX_TRANSLATION_PER_CYCLE)
        delta[3:] = np.clip(delta[3:], -cfg.MAX_ROTATION_PER_CYCLE, cfg.MAX_ROTATION_PER_CYCLE)

        # Axis enable mask
        delta *= np.array(cfg.AXIS_ENABLE, dtype=float)

        return delta

    def _clamp_workspace(self):
        """Clamp target_pose xyz to workspace boundaries."""
        for i in range(3):
            self.target_pose[i] = np.clip(self.target_pose[i],
                                          cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i])

    def _handle_buttons(self):
        """Process button events for gripper control (LJL 版: 含 disabled 早退与位置同步)."""
        if cfg.GRIPPER_MODE == "disabled":
            self.mouse.pop_button_events()
            return
        if self._gripper_pos is None:
            pos = self.arm.get_gripper_position()
            if pos is None:
                self.mouse.pop_button_events()
                return
            self._gripper_pos = float(pos)
        if cfg.GRIPPER_MODE == "binary":
            # 按一下立即到位
            for bnum, pressed in self.mouse.pop_button_events():
                if not pressed:
                    continue
                if bnum == 0:
                    self._gripper_pos = float(cfg.GRIPPER_CLOSE_POS)
                    print("Gripper CLOSE")
                elif bnum == 1:
                    self._gripper_pos = float(cfg.GRIPPER_OPEN_POS)
                    print("Gripper OPEN")
                self.arm.set_gripper_position(self._gripper_pos)
        elif cfg.GRIPPER_MODE == "incremental":
            # incremental: 按住持续变化，松开停止
            self.mouse.pop_button_events()  # 消费事件队列，保持按钮状态更新
            if self.mouse.get_button(0):
                self._gripper_pos = max(cfg.GRIPPER_CLOSE_POS,
                                        self._gripper_pos - cfg.GRIPPER_INCREMENT)
                self.arm.set_gripper_position(self._gripper_pos)
            elif self.mouse.get_button(1):
                self._gripper_pos = min(cfg.GRIPPER_OPEN_POS,
                                        self._gripper_pos + cfg.GRIPPER_INCREMENT)
                self.arm.set_gripper_position(self._gripper_pos)
        elif cfg.GRIPPER_MODE == "Switching":
            for bnum, pressed in self.mouse.pop_button_events():
                if not pressed:
                    continue
                try:
                    if bnum == 0:
                        self.open_relay(cfg.RELAY_CHANNEL)
                        print("Relay CLOSE (magnet on)")
                    elif bnum == 1:
                        self.close_relay(cfg.RELAY_CHANNEL)
                        print("Relay OPEN (magnet off)")
                except Exception as e:
                    print(f"[WARN] Relay command failed: {e}")

    def _read_force_wrench(self):
        """Read and return [Fx, Fy, Fz, Mx, My, Mz]. Returns None on failure."""
        try:
            result = self.arm.arm.rm_get_force_data()
        except Exception as e:
            print(f"[WARN] rm_get_force_data failed: {e}")
            return None

        ret = 0
        payload = result
        if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], (int, np.integer)):
            ret, payload = result
        if ret != 0:
            print(f"[WARN] rm_get_force_data returned {ret}")
            return None

        if isinstance(payload, dict):
            # Some SDK versions may return keyed dicts directly.
            if all(k in payload for k in ("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
                return np.array([
                    payload["Fx"], payload["Fy"], payload["Fz"],
                    payload["Mx"], payload["My"], payload["Mz"],
                ], dtype=float)

            # RM SDK (v1.1.4+) returns rm_force_data_t as dict with these keys.
            for key in ("zero_force_data", "work_zero_force_data", "tool_zero_force_data", "force_data", "force", "data"):
                if key in payload and payload[key] is not None:
                    payload = payload[key]
                    break
            else:
                if not self._force_payload_warned:
                    self._force_payload_warned = True
                    print(f"[WARN] Unknown force payload keys: {list(payload.keys())}")
                payload = None
        if payload is None:
            print("[WARN] Force payload missing")
            return None

        try:
            force6 = np.array(payload[:6], dtype=float)
            if force6.shape[0] < 6:
                print(f"[WARN] Invalid force payload length: {force6.shape[0]}")
                return None
        except Exception as e:
            print(f"[WARN] Force payload parse failed: {e}")
            return None

        return force6
    # ------ main loop ------

    def run(self):
        """Control loop (freq = CONTROL_RATE_HZ)."""
        dt = 1.0 / cfg.CONTROL_RATE_HZ
        print(f"Running at {cfg.CONTROL_RATE_HZ} Hz.  Ctrl+C to stop.\n")

        while True:
            t0 = time.monotonic()

            # 1. Read SpaceMouse
            raw = self.mouse.get_axes()

            # 2. Compute incremental delta
            delta = self._compute_delta(raw)

            # 查询实时反馈：UDP 模式一次拿六维力 + 当前位姿
            if cfg.UDP_FEEDBACK_ENABLE:
                force6, current_pose, feedback_age = read_udp_feedback(self.arm, cfg.UDP_TIMEOUT_S)

                # UDP 实际频率统计：_latest_udp_time 变化说明来了新帧
                cur_udp_ts = self.arm._latest_udp_time
                if cur_udp_ts != self._udp_last_ts:
                    self._udp_last_ts = cur_udp_ts
                    self._udp_frame_times.append(time.monotonic())
                    self._udp_frame_total += 1
            else:
                force6 = self._read_force_wrench()
                feedback_age = None

                ret_state, state = self.arm.arm.rm_get_current_arm_state()
                if ret_state == 0:
                    current_pose = np.array(state["pose"], dtype=float)
                else:
                    current_pose = None

            # 2.5 Outer admittance control
            if force6 is not None:
                wrench = force6.copy()

                deadzone = np.array([1.0, 1.0, 1.0, 0.05, 0.05, 0.05])
                wrench[np.abs(wrench) < deadzone] = 0.0

                M = np.array([3.0, 3.0, 3.0, 0.15, 0.15, 0.15])
                B = np.array([80.0, 80.0, 80.0, 3.0, 3.0, 3.0])
                K = np.array([300.0, 300.0, 300.0, 8.0, 8.0, 8.0])

                desired = np.zeros(6)

                vel_limit = np.array([0.02, 0.02, 0.02, 0.15, 0.15, 0.15])
                offset_limit = np.array([0.03, 0.03, 0.03, 0.20, 0.20, 0.20])

                a = (wrench - desired - B * self._adm_vel - K * self._adm_offset) / M

                self._adm_vel = np.clip(self._adm_vel + a * dt, -vel_limit, vel_limit)
                self._adm_offset = np.clip(
                    self._adm_offset + self._adm_vel * dt, -offset_limit, offset_limit
                )
            else:
                self._adm_vel[:] = 0.0

            # 3. Update target pose by axis mode
            if current_pose is not None:
                for i, axis_mode in enumerate(self.control_mode):
                    if axis_mode in (1, 4):
                        self.target_pose[i] = current_pose[i]
                        delta[i] = 0.0

            self.target_pose += delta

            # 4. Workspace clamp
            self._clamp_workspace()
            # 5. Send to robot

            # 导纳控制
            cmd_pose = self.target_pose.copy()
            cmd_pose += self._adm_offset

            param = rm.rm_force_position_move_t(
                    flag=1,
                    pose=cmd_pose.tolist(),
                    sensor=1,
                    mode=0,
                    follow=False,
                    control_mode=self.control_mode,
                    desired_force=self.desired_force,
                    limit_vel=[0.1, 0.1, 0.1, 10, 10, 10],
                    # trajectory_mode=0,
                    # radio=50,
                )
            ret = self.arm.arm.rm_force_position_move(param)

            # 显示当前位姿误差；UDP 模式下 current_pose 来自 state["waypoint"]
            if current_pose is not None:
                pose_diff = cmd_pose - current_pose

                f_str = f"F: {np.round(force6[:3], 1).tolist() if force6 is not None else 'N/A'}"
                age_str = f"{feedback_age * 1000:.0f}ms" if feedback_age is not None else "TCP"
                z_val = current_pose[2]
                diff_str = np.round(pose_diff[:3]*1000, 1).tolist()

                # UDP 实际频率（最近 1s 滑窗）
                now_t = time.monotonic()
                udp_hz = sum(1 for t in self._udp_frame_times if now_t - t <= 1.0)
                udp_str = f"udp={udp_hz}Hz" if cfg.UDP_FEEDBACK_ENABLE else "udp=off"

                sys.stdout.write(
                    f"\rZ: {z_val:.4f}m | "
                    f"{f_str} | "
                    f"age={age_str} | "
                    f"{udp_str} | "
                    f"Diff(mm): {diff_str}        "
                )
                sys.stdout.flush()

            if ret != 0:
                self._consecutive_fails += 1
                if self._consecutive_fails >= 10:
                    print(f"[ERROR] {self._consecutive_fails} consecutive movep failures — emergency stop!")
                    self.arm.arm.rm_set_arm_stop()
                    break
            else:
                self._consecutive_fails = 0

            # 6. Gripper buttons
            self._handle_buttons()

            # 7. Timing
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ---------- entry point ----------

def main():
    parser = argparse.ArgumentParser(description="SpaceMouse admittance teleop + gripper for RM75B")
    parser.add_argument("--ip", default=cfg.ROBOT_IP, help="Robot IP address")
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT, help="Robot TCP port")
    args = parser.parse_args()

    teleop = SpaceMouseTeleop(args.ip, args.port)

    def _sigint(sig, frame):
        print("\nCtrl+C received — stopping...")
        teleop.teardown(slow_stop=True)
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    teleop.setup()
    teleop.run()


if __name__ == "__main__":
    main()
