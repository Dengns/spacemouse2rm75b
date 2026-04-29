"""SpaceMouse teleoperation for RM75B — incremental Cartesian control via rm_movep_canfd."""

import sys
import argparse
import signal
import time

import numpy as np

import config_switch_safety as cfg
from spacemouse_input import SpaceMouseReader
from rm75b import RM75BInterface
from class_switch import USBRelayController 


class SpaceMouseTeleop(USBRelayController):
    """Incremental Cartesian teleop: SpaceMouse -> rm_movep_canfd -> RM75B."""

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
        self._gripper_pos = float(cfg.GRIPPER_OPEN_POS)  # 当前夹爪位置 (0~1000)
        self._relay_ready = False
        self._force_hold = False
        self._hold_pose = None
        self._z_down_limited = False
        self._force_payload_warned = False

    # ------ setup / teardown ------

    def setup(self):
        # 1. SpaceMouse
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()

        # 2. Robot arm + gripper
        self.arm = RM75BInterface(self.ip, self.port, enable_gripper=(cfg.GRIPPER_MODE != "Switching"))

        if cfg.GRIPPER_MODE == "Switching":
            try:
                self.connect()
                self._relay_ready = True
            except Exception as e:
                self._relay_ready = False
                print(f"[WARN] Relay connect failed: {e}")

        # 3. Read current pose as starting point
        # 将六维力数据清零，标定当前状态下的零位
        self.arm.arm.rm_clear_force_data()
        
        ret, state = self.arm.arm.rm_get_current_arm_state()
        if ret != 0:
            raise RuntimeError(f"Failed to read arm state (ret={ret})")
        # state["pose"] = [x, y, z, rx, ry, rz] (m / rad)
        self.target_pose = np.array(state["pose"], dtype=float)
        print(f"Start pose: {np.round(self.target_pose, 4).tolist()}")

    def teardown(self, slow_stop: bool = True):
        """Clean shutdown: stop arm, open gripper, disconnect."""
        if self.arm is not None:
            if slow_stop:
                self.arm.arm.rm_set_arm_slow_stop()
            if cfg.GRIPPER_MODE != "Switching":
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
        """Apply axis mapping, sign flip, EMA smcoothing, scaling, and clamping."""
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

    def _handle_buttons(self):
        """Process button events for gripper control."""
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
            # if not self._relay_ready:
            #     return
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

    def _update_z_down_limit(self, force6):
        """Update Z-down limit state from directional Fz force (no smoothing)."""
        if not bool(cfg.FORCE_Z_LIMIT_ENABLE):
            if self._z_down_limited:
                self._z_down_limited = False
                print("[SAFE] Z-down limit disabled -> OFF")
            return 0.0

        direction = 1.0 if float(cfg.FORCE_Z_LIMIT_DIRECTION) >= 0.0 else -1.0
        fz_dir = direction * float(force6[2])

        # 向下如果受到阻碍，力是负数，所以不等号右侧都加了-号
        if (not self._z_down_limited) and (fz_dir <= -float(cfg.FORCE_Z_LIMIT_TRIGGER_N)):
            self._z_down_limited = True
            print(
                f"[SAFE] Directional Fz {fz_dir:.2f} N >= "
                f"{float(cfg.FORCE_Z_LIMIT_TRIGGER_N):.2f} N -> Z down LOCK"
            )
        elif self._z_down_limited and (fz_dir >= -float(cfg.FORCE_Z_LIMIT_RELEASE_N)):
            self._z_down_limited = False
            print(
                f"[SAFE] Directional Fz {fz_dir:.2f} N <= "
                f"{float(cfg.FORCE_Z_LIMIT_RELEASE_N):.2f} N -> Z down UNLOCK"
            )

        return fz_dir

    # ------ main loop ------

    def run(self):
        """50 Hz control loop."""
        dt = 1.0 / cfg.CONTROL_RATE_HZ
        print(f"Running at {cfg.CONTROL_RATE_HZ} Hz.  Ctrl+C to stop.\n")
    

        while True:
            t0 = time.monotonic()

            # 1. Read SpaceMouse
            raw = self.mouse.get_axes()

            # 2. Compute incremental delta
            delta = self._compute_delta(raw)

            # 3. Read force and update stop/hold state
            # 查询六维力传感器力信息
            force6 = self._read_force_wrench()
            if force6 is None:
                if not self._force_hold:
                    self._force_hold = True
                    self._hold_pose = self.target_pose.copy()
                    print("[SAFE] Force feedback unavailable -> HOLD")
            else:
                force_norm = float(np.linalg.norm(force6[:3]))
                fz_dir = self._update_z_down_limit(force6)

                print(
                    "[FT] "
                    f"Fx={force6[0]:+7.3f} N  "
                    f"Fy={force6[1]:+7.3f} N  "
                    f"Fz={force6[2]:+7.3f} N  "
                    f"Mx={force6[3]:+7.3f} Nm  "
                    f"My={force6[4]:+7.3f} Nm  "
                    f"Mz={force6[5]:+7.3f} Nm  "
                    f"|F|={force_norm:6.3f} N  "
                    f"Fz_dir={fz_dir:+6.3f} N  "
                    f"Zlock={'ON' if self._z_down_limited else 'OFF'}",
                    end="\r",
                    flush=True,
                )

                if self._force_hold:
                    self._force_hold = False
                    self.target_pose = self._hold_pose.copy()
                    print("[SAFE] Force feedback restored -> RESUME")

            # 4. Accumulate target pose when not holding
            if not self._force_hold:
                # 遥操下移，delta[2]小于0
                if self._z_down_limited and delta[2] < 0.0:
                    delta[2] = 0.0
                # Workspace boundary: reject delta that would exceed limits (no clipping)
                for i in range(3):
                    proposed = self.target_pose[i] + delta[i]
                    if proposed < cfg.WORKSPACE_MIN[i] or proposed > cfg.WORKSPACE_MAX[i]:
                        raise RuntimeError(
                            f"[SAFE] Workspace boundary exceeded on axis {i}: "
                            f"proposed={proposed:.6f}, "
                            f"range=[{cfg.WORKSPACE_MIN[i]:.6f}, {cfg.WORKSPACE_MAX[i]:.6f}]"
                        )
                self.target_pose += delta

            # 6. Send to robot (hold pose only when force feedback is unavailable)
            pose_cmd = self._hold_pose if self._force_hold else self.target_pose
            ret = self.arm.arm.rm_movep_canfd(pose_cmd.tolist(), follow=True)
            if ret != 0:
                self._consecutive_fails += 1
                if self._consecutive_fails >= 10:
                    print(f"[ERROR] {self._consecutive_fails} consecutive movep failures — emergency stop!")
                    self.arm.arm.rm_set_arm_stop()
                    break
            else:
                self._consecutive_fails = 0

            # 7. Gripper buttons
            self._handle_buttons()

            # 8. Timing
            elapsed = time.monotonic() - t0
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ---------- entry point ----------

def main():
    parser = argparse.ArgumentParser(description="SpaceMouse teleop for RM75B")
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
