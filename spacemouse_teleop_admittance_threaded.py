"""三线程多频率 SpaceMouse 导纳遥操作 for RM75B.

架构:
    SpaceMouseInputThread (100Hz) ──┐
                                    ├─► SharedState ─► ControlThread (200Hz) ─► rm_movep_canfd(follow=True)
    UDP 回调 (200Hz, 已存在)        ─┘
                                    └─► RecordThread (10Hz, 预留)

主线程负责 setup / signal / join / teardown，三线程通过单一 SharedState + Lock 通信。
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import config_admittance_threaded as cfg
from spacemouse_input import SpaceMouseReader
from rm75b import RM75BInterface
from class_switch import USBRelayController
from udp_feedback import enable_udp_feedback, read_udp_feedback, wait_for_udp_pose


# ============================================================================
# Shared state
# ============================================================================


def _move_towards(value: float, target: float, max_step: float) -> float:
    """按最大步长把标量推向目标值；max_step<=0 时保持原值。"""
    if max_step <= 0.0:
        return value
    delta = target - value
    if abs(delta) <= max_step:
        return target
    return value + np.sign(delta) * max_step


@dataclass
class SharedState:
    """三线程共享数据。所有读写都必须 with lock_。"""

    # SpaceMouseInputThread 写, ControlThread 读
    target_pose: np.ndarray = field(default_factory=lambda: np.zeros(6))
    gripper_pos: float = 0.0
    button_events: list = field(default_factory=list)   # [(bnum, pressed), ...]

    # ControlThread 写, RecordThread 读
    current_pose: Optional[np.ndarray] = None
    force6: Optional[np.ndarray] = None
    adm_offset: np.ndarray = field(default_factory=lambda: np.zeros(6))
    cmd_pose: Optional[np.ndarray] = None
    feedback_age_ms: Optional[float] = None
    z_in_contact: bool = False

    # 控制旗
    running: bool = True

    lock_: threading.Lock = field(default_factory=threading.Lock)


# ============================================================================
# Main teleop orchestrator
# ============================================================================

class ThreadedAdmittanceTeleop(USBRelayController):
    """三线程导纳遥操作主类。继承 USBRelayController 以便使用继电器夹爪。"""

    def __init__(self, ip: str, port: int):
        super().__init__(
            serial_port=cfg.SERIAL_PORT,
            baud_rate=cfg.SERIAL_BAUD_RATE,
            timeout=cfg.SERIAL_TIMEOUT,
        )

        self.ip = ip
        self.port = port

        self.mouse: Optional[SpaceMouseReader] = None
        self.arm: Optional[RM75BInterface] = None
        self.state = SharedState()

        self.input_thread: Optional[threading.Thread] = None
        self.control_thread: Optional[threading.Thread] = None
        self.record_thread: Optional[threading.Thread] = None

        self._relay_ready = False

        # Recording
        self._record_file = None
        self._record_writer = None
        self._record_t0: Optional[float] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def setup(self):
        # 1. SpaceMouse
        self.mouse = SpaceMouseReader()
        self.mouse.open()
        self.mouse.start()

        # 2. Arm
        self.arm = RM75BInterface(
            self.ip, self.port,
            enable_gripper=(cfg.GRIPPER_MODE != "Switching"),
        )

        # 3. UDP 反馈（强制开启，三线程架构必须有实时 UDP）
        if not cfg.UDP_FEEDBACK_ENABLE:
            raise RuntimeError("Threaded admittance requires UDP_FEEDBACK_ENABLE=True")
        enable_udp_feedback(self.arm, cfg)

        # 4. 继电器
        if cfg.GRIPPER_MODE == "Switching":
            try:
                self.connect()
                self._relay_ready = True
            except Exception as e:
                print(f"[WARN] Relay connect failed: {e}")

        # 5. 力清零（即使不用 SDK 力控也清一下，UDP 推送的力数据会以此为零位）
        ret = self.arm.arm.rm_clear_force_data()
        if isinstance(ret, int) and ret != 0:
            print(f"[WARN] rm_clear_force_data ret={ret}")

        # 6. 初始位姿
        current_pose = wait_for_udp_pose(self.arm, cfg.UDP_TIMEOUT_S)
        if current_pose is None:
            raise RuntimeError("Failed to read initial pose from UDP feedback")

        # 写入 SharedState（无锁，因为线程还没启动）
        self.state.target_pose = current_pose.copy()
        self.state.current_pose = current_pose.copy()
        self.state.cmd_pose = current_pose.copy()
        self.state.gripper_pos = float(cfg.GRIPPER_OPEN_POS)

        print(f"Start pose from UDP: {np.round(current_pose, 4).tolist()}")

        # 7. 录制（可选）
        if cfg.RECORD_ENABLE:
            self._open_recorder()

    def start(self):
        self.input_thread = threading.Thread(
            target=self._spacemouse_input_loop, name="InputThread", daemon=True)
        self.control_thread = threading.Thread(
            target=self._control_loop, name="ControlThread", daemon=True)
        self.record_thread = threading.Thread(
            target=self._record_loop, name="RecordThread", daemon=True)

        self.input_thread.start()
        self.control_thread.start()
        self.record_thread.start()

        print("Threads started: Input(100Hz) / Control(200Hz) / Record(10Hz)")

    def stop(self):
        with self.state.lock_:
            self.state.running = False

    def join(self):
        """主线程阻塞：先等 stop() 被调用，然后 join 三个工作线程。"""
        # 等待 running 变 False（SIGINT 处理器或控制线程 emergency stop 都会触发）
        try:
            while True:
                with self.state.lock_:
                    if not self.state.running:
                        break
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.stop()

        # running=False 之后，工作线程下个周期就退出；给 2s 安全网防止挂死
        for t in (self.input_thread, self.control_thread, self.record_thread):
            if t is not None and t.is_alive():
                t.join(timeout=2.0)

    def teardown(self, slow_stop: bool = True):
        if self.arm is not None:
            if slow_stop:
                self.arm.arm.rm_set_arm_slow_stop()
            if cfg.GRIPPER_MODE != "Switching":
                self.arm.set_gripper_position(cfg.GRIPPER_OPEN_POS)
                time.sleep(0.3)
            self.arm.close()
        if cfg.GRIPPER_MODE == "Switching" and self._relay_ready:
            try:
                self.disconnect()
            except Exception as e:
                print(f"[WARN] Relay disconnect failed: {e}")
        if self.mouse is not None:
            self.mouse.stop()

        # 关闭记录、可选画图
        self._close_recorder()
        if cfg.RECORD_ENABLE and cfg.RECORD_PLOT_ON_EXIT:
            self._auto_plot()

        print("Shutdown complete.")

    # ------------------------------------------------------------------
    # Thread A: SpaceMouse input (100Hz)
    # ------------------------------------------------------------------

    def _spacemouse_input_loop(self):
        """读 SpaceMouse → 平滑 → scale → 积分 → 写 target_pose。"""
        dt = 1.0 / cfg.INPUT_RATE_HZ

        smoothed = np.zeros(6)
        # 线程内维护一份 target_pose 副本，避免每个周期都拿锁来读
        with self.state.lock_:
            local_target = self.state.target_pose.copy()

        last_rate_print = time.monotonic()
        ticks_since_print = 0

        while True:
            t0 = time.monotonic()
            with self.state.lock_:
                if not self.state.running:
                    break
                z_in_contact = self.state.z_in_contact
                adm_z_offset = float(self.state.adm_offset[2])

            # 1. 读 axes
            raw = self.mouse.get_axes()

            # 2. 计算 delta（沿用旧脚本 _compute_delta 逻辑）
            mapped = np.array(
                [raw[cfg.AXIS_MAP[i]] * cfg.AXIS_SIGNS[i] for i in range(6)],
                dtype=float,
            )
            smoothed = cfg.EMA_ALPHA * mapped + (1.0 - cfg.EMA_ALPHA) * smoothed

            delta = np.zeros(6)
            delta[:3] = smoothed[:3] * cfg.TRANSLATION_SCALE
            delta[3:] = smoothed[3:] * cfg.ROTATION_SCALE
            delta[:3] = np.clip(delta[:3], -cfg.MAX_TRANSLATION_PER_CYCLE,
                                cfg.MAX_TRANSLATION_PER_CYCLE)
            delta[3:] = np.clip(delta[3:], -cfg.MAX_ROTATION_PER_CYCLE,
                                cfg.MAX_ROTATION_PER_CYCLE)
            delta *= np.array(cfg.AXIS_ENABLE, dtype=float)

            if (z_in_contact
                    and bool(getattr(cfg, "ADM_Z_LOCK_TARGET_WHEN_IN_CONTACT", True))
                    and delta[2] < 0.0):
                delta[2] = 0.0

            # adm_offset[z] 撑到限位后，禁止 target 继续向同方向积累（避免 cmd 失控压入）
            z_offset_limit = float(cfg.ADM_OFFSET_LIMIT[2])
            if (z_offset_limit > 0.0
                    and abs(adm_z_offset) >= 0.95 * z_offset_limit
                    and adm_z_offset * delta[2] < 0.0):
                delta[2] = 0.0

            # 3. 积分 + workspace clamp
            local_target += delta
            for i in range(3):
                local_target[i] = np.clip(
                    local_target[i], cfg.WORKSPACE_MIN[i], cfg.WORKSPACE_MAX[i]
                )

            # 4. 写入 SharedState + 处理按钮事件
            btn_events = self.mouse.pop_button_events()
            with self.state.lock_:
                self.state.target_pose = local_target.copy()
                if btn_events:
                    self.state.button_events.extend(btn_events)

            # 频率自检
            ticks_since_print += 1
            now = time.monotonic()
            if now - last_rate_print >= 2.0:
                rate = ticks_since_print / (now - last_rate_print)
                print(f"[Input]   actual={rate:.1f}Hz")
                ticks_since_print = 0
                last_rate_print = now

            # 5. 计时
            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ------------------------------------------------------------------
    # Thread B: Control loop (200Hz)
    # ------------------------------------------------------------------

    def _control_loop(self):
        """读 UDP 反馈 → 导纳 → 合成 cmd → rm_movep_canfd(follow=True)。"""
        dt = 1.0 / cfg.CONTROL_RATE_HZ

        # 导纳状态
        adm_vel = np.zeros(6)
        adm_offset = np.zeros(6)

        # 导纳参数转 numpy 一次
        M = np.array(cfg.ADM_M, dtype=float)
        B = np.array(cfg.ADM_B, dtype=float)
        K = np.array(cfg.ADM_K, dtype=float)
        deadzone = np.array(cfg.ADM_DEADZONE, dtype=float)
        vel_limit = np.array(cfg.ADM_VEL_LIMIT, dtype=float)
        offset_limit = np.array(cfg.ADM_OFFSET_LIMIT, dtype=float)

        # 各轴导纳开关：默认 [1,1,1,1,1,1]（向后兼容旧 config）
        axis_enable_raw = getattr(cfg, "ADM_AXIS_ENABLE", [1, 1, 1, 1, 1, 1])
        adm_axis_enable = np.array(axis_enable_raw, dtype=bool)
        # planar = xy + 三个旋转，只保留 enable 中为 True 的
        planar_axes = np.array(
            [i for i in (0, 1, 3, 4, 5) if adm_axis_enable[i]], dtype=int
        )
        z_axis_enabled = bool(adm_axis_enable[2])
        print(f"[Adm] axis enable mask = {adm_axis_enable.astype(int).tolist()}  "
              f"(planar={planar_axes.tolist()}, z={z_axis_enabled})")

        # Z 向桌面接触特化参数：只对“压桌面”一侧做导纳，脱离接触时慢释放
        z_table_mode = bool(getattr(cfg, "ADM_Z_TABLE_MODE_ENABLE", False))
        z_contact_sign = float(getattr(cfg, "ADM_Z_CONTACT_SIGN", -1.0))
        if abs(z_contact_sign) < 1e-6:
            z_contact_sign = -1.0
        z_contact_enter = float(getattr(cfg, "ADM_Z_CONTACT_ENTER_N", 2.0))
        z_contact_exit = float(getattr(cfg, "ADM_Z_CONTACT_EXIT_N", 0.8))
        z_force_alpha = float(np.clip(getattr(cfg, "ADM_Z_FORCE_LPF_ALPHA", 1.0), 0.0, 1.0))
        z_release_vel = float(getattr(cfg, "ADM_Z_RELEASE_VEL", 0.0))
        z_in_contact = False
        z_force_filt = 0.0
        z_contact_force = 0.0

        consecutive_fails = 0
        last_rate_print = time.monotonic()
        ticks_since_print = 0
        last_status_print = time.monotonic()

        while True:
            t0 = time.monotonic()
            with self.state.lock_:
                if not self.state.running:
                    break

            # 1. UDP 反馈
            force6, current_pose, age = read_udp_feedback(self.arm, cfg.UDP_TIMEOUT_S)

            # 2. 读 target_pose
            with self.state.lock_:
                target_pose = self.state.target_pose.copy()
                pending_buttons = list(self.state.button_events)
                self.state.button_events.clear()
                gripper_pos = self.state.gripper_pos

            # 3. 导纳计算（可整体关掉，也可按轴关掉）
            if cfg.ADM_ENABLE and force6 is not None:
                # 软死区：|F| ≤ dz 时输出 0；|F| > dz 时输出 sign(F)·(|F|-dz)。
                # 在死区边界处连续，不会像硬死区那样从 0 突跳到 ±dz。
                excess = np.maximum(np.abs(force6) - deadzone, 0.0)
                wrench = np.sign(force6) * excess

                # xy + rotation: 仅对启用的 planar 轴做对称导纳
                if len(planar_axes) > 0:
                    a_other = (
                        wrench[planar_axes]
                        - B[planar_axes] * adm_vel[planar_axes]
                        - K[planar_axes] * adm_offset[planar_axes]
                    ) / M[planar_axes]
                    adm_vel[planar_axes] = np.clip(
                        adm_vel[planar_axes] + a_other * dt,
                        -vel_limit[planar_axes],
                        vel_limit[planar_axes],
                    )
                    adm_offset[planar_axes] = np.clip(
                        adm_offset[planar_axes] + adm_vel[planar_axes] * dt,
                        -offset_limit[planar_axes],
                        offset_limit[planar_axes],
                    )

                if not z_axis_enabled:
                    # Z 轴关闭：清零状态并跳过下面的 Z 块
                    adm_vel[2] = 0.0
                    adm_offset[2] = 0.0
                    z_in_contact = False
                    z_force_filt = 0.0
                    z_contact_force = 0.0
                elif z_table_mode:
                    z_force_filt = z_force_alpha * float(force6[2]) + (1.0 - z_force_alpha) * z_force_filt
                    z_contact_force = max(0.0, z_contact_sign * z_force_filt)

                    if z_in_contact:
                        if z_contact_force <= z_contact_exit:
                            z_in_contact = False
                    elif z_contact_force >= z_contact_enter:
                        z_in_contact = True

                    if z_in_contact:
                        # Z 轴用减法软死区，避免接触阈值附近的力突跳。
                        z_wrench = max(0.0, z_contact_force - deadzone[2])
                        a_z = (z_wrench - B[2] * adm_vel[2] - K[2] * adm_offset[2]) / M[2]
                        adm_vel[2] = np.clip(adm_vel[2] + a_z * dt, -vel_limit[2], vel_limit[2])
                        adm_offset[2] = np.clip(
                            adm_offset[2] + adm_vel[2] * dt,
                            -offset_limit[2],
                            offset_limit[2],
                        )
                    else:
                        adm_vel[2] = 0.0
                        adm_offset[2] = _move_towards(
                            adm_offset[2], 0.0, z_release_vel * dt
                        )
                else:
                    z_contact_force = 0.0
                    z_in_contact = False
                    a_z = (wrench[2] - B[2] * adm_vel[2] - K[2] * adm_offset[2]) / M[2]
                    adm_vel[2] = np.clip(adm_vel[2] + a_z * dt, -vel_limit[2], vel_limit[2])
                    adm_offset[2] = np.clip(
                        adm_offset[2] + adm_vel[2] * dt,
                        -offset_limit[2],
                        offset_limit[2],
                    )
            else:
                # 关闭导纳 或 无力反馈 → 速度清零，offset 保持
                adm_vel[:] = 0.0
                z_in_contact = False
                z_contact_force = 0.0
                z_force_filt = 0.0
                if not cfg.ADM_ENABLE:
                    adm_offset[:] = 0.0   # 关闭时直接归零

            # 4. cmd_pose = target + adm_offset (导纳关闭时 offset=0)
            cmd_pose = target_pose + adm_offset

            # 5. 下发
            ret = self.arm.arm.rm_movep_canfd(
                cmd_pose.tolist(),
                follow=cfg.MOVEP_FOLLOW,
                trajectory_mode=cfg.MOVEP_TRAJECTORY_MODE,
                radio=cfg.MOVEP_RADIO,
            )
            if ret != 0:
                consecutive_fails += 1
                if consecutive_fails >= cfg.MOVEP_MAX_CONSECUTIVE_FAILS:
                    print(f"[ERROR] {consecutive_fails} consecutive movep_canfd "
                          f"failures — emergency stop!")
                    self.arm.arm.rm_set_arm_stop()
                    with self.state.lock_:
                        self.state.running = False
                    break
            else:
                consecutive_fails = 0

            # 6. 写回 SharedState
            with self.state.lock_:
                if current_pose is not None:
                    self.state.current_pose = current_pose.copy()
                if force6 is not None:
                    self.state.force6 = force6.copy()
                self.state.adm_offset = adm_offset.copy()
                self.state.cmd_pose = cmd_pose.copy()
                self.state.feedback_age_ms = age * 1000.0 if age is not None else None
                self.state.z_in_contact = z_in_contact

            # 7. 夹爪事件
            if pending_buttons:
                self._handle_button_events(pending_buttons, gripper_pos)

            # 频率自检 + 状态打印
            ticks_since_print += 1
            now = time.monotonic()
            if now - last_rate_print >= 2.0:
                rate = ticks_since_print / (now - last_rate_print)
                print(f"[Control] actual={rate:.1f}Hz  "
                      f"age={age * 1000:.0f}ms  "
                      f"fz={force6[2]:.2f}N  "
                      f"fz_c={z_contact_force:.2f}N  "
                      f"z_contact={'ON' if z_in_contact else 'OFF'}  "
                      f"adm_z={adm_offset[2] * 1000:.1f}mm"
                      if (age is not None and force6 is not None)
                      else f"[Control] actual={rate:.1f}Hz (no fb)")
                ticks_since_print = 0
                last_rate_print = now

            # 8. 计时
            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _handle_button_events(self, events, current_gripper_pos):
        """根据 GRIPPER_MODE 处理按钮事件。线程：ControlThread 内调用。"""
        new_pos = current_gripper_pos

        if cfg.GRIPPER_MODE == "binary":
            for bnum, pressed in events:
                if not pressed:
                    continue
                if bnum == 0:
                    new_pos = float(cfg.GRIPPER_CLOSE_POS)
                    print("Gripper CLOSE")
                elif bnum == 1:
                    new_pos = float(cfg.GRIPPER_OPEN_POS)
                    print("Gripper OPEN")
            if new_pos != current_gripper_pos:
                self.arm.set_gripper_position(new_pos)

        elif cfg.GRIPPER_MODE == "Switching":
            for bnum, pressed in events:
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

        # incremental 模式在 100Hz 主循环里不太合适，这里跳过。
        # 真要用 incremental，应该在 InputThread 里 hold 状态读取。

        if new_pos != current_gripper_pos:
            with self.state.lock_:
                self.state.gripper_pos = new_pos

    # ------------------------------------------------------------------
    # Thread C: Record (10Hz, 预留)
    # ------------------------------------------------------------------

    def _record_loop(self):
        """10Hz 读 SharedState 快照交给 _record_step。当前只是骨架。"""
        dt = 1.0 / cfg.RECORD_RATE_HZ

        while True:
            t0 = time.monotonic()
            with self.state.lock_:
                if not self.state.running:
                    break
                snapshot = {
                    "t": t0,
                    "target_pose": (self.state.target_pose.copy()
                                    if self.state.target_pose is not None else None),
                    "current_pose": (self.state.current_pose.copy()
                                     if self.state.current_pose is not None else None),
                    "cmd_pose": (self.state.cmd_pose.copy()
                                 if self.state.cmd_pose is not None else None),
                    "force6": (self.state.force6.copy()
                               if self.state.force6 is not None else None),
                    "adm_offset": self.state.adm_offset.copy(),
                    "feedback_age_ms": self.state.feedback_age_ms,
                }

            self._record_step(snapshot)

            elapsed = time.monotonic() - t0
            sleep_t = dt - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _record_step(self, snapshot: dict):
        """每 100ms 一次的状态快照 → CSV 一行。RECORD_ENABLE=False 时是 no-op。"""
        if self._record_writer is None:
            return

        if self._record_t0 is None:
            self._record_t0 = snapshot["t"]
        rel_t = snapshot["t"] - self._record_t0

        row = [f"{rel_t:.4f}"]
        for key in ("target_pose", "current_pose", "cmd_pose", "force6", "adm_offset"):
            val = snapshot[key]
            if val is None:
                row.extend([""] * 6)
            else:
                row.extend(f"{v:.6f}" for v in val)
        row.append(f"{snapshot['feedback_age_ms']:.2f}"
                   if snapshot["feedback_age_ms"] is not None else "")
        self._record_writer.writerow(row)

    # ------------------------------------------------------------------
    # Recording helpers
    # ------------------------------------------------------------------

    _RECORD_HEADER = (
        ["t"]
        + [f"tgt_{a}" for a in ("x", "y", "z", "rx", "ry", "rz")]
        + [f"cur_{a}" for a in ("x", "y", "z", "rx", "ry", "rz")]
        + [f"cmd_{a}" for a in ("x", "y", "z", "rx", "ry", "rz")]
        + [f"f_{a}"   for a in ("x", "y", "z", "rx", "ry", "rz")]
        + [f"adm_{a}" for a in ("x", "y", "z", "rx", "ry", "rz")]
        + ["age_ms"]
    )

    def _open_recorder(self):
        os.makedirs(os.path.dirname(cfg.RECORD_FILE), exist_ok=True)
        self._record_file = open(cfg.RECORD_FILE, "w", newline="")
        self._record_writer = csv.writer(self._record_file)
        self._record_writer.writerow(self._RECORD_HEADER)
        print(f"Recording → {cfg.RECORD_FILE}")

    def _close_recorder(self):
        if self._record_file is not None:
            self._record_file.close()
            print(f"Record saved: {cfg.RECORD_FILE}")
        self._record_file = None
        self._record_writer = None

    def _auto_plot(self):
        """退出后调用 plot_admittance_log.py 画图（同步，错误直接打印到终端）。"""
        script = os.path.join(os.path.dirname(__file__), "plot_admittance_log.py")
        if not os.path.exists(script):
            print(f"[WARN] plot script not found: {script}")
            return

        # 检查 CSV 是否真的有数据
        if not os.path.exists(cfg.RECORD_FILE) or os.path.getsize(cfg.RECORD_FILE) < 100:
            print(f"[WARN] record file empty or missing: {cfg.RECORD_FILE}")
            return

        cmd = [sys.executable, script, cfg.RECORD_FILE]

        # 无 DISPLAY 时直接保存图片，避免 Agg 后端下 plt.show() 静默无效果
        if "DISPLAY" not in os.environ:
            png_path = cfg.RECORD_FILE.replace(".csv", ".png")
            cmd.extend(["--save", png_path])
            print(f"No DISPLAY found → saving plot to {png_path}")

        print(f"Plotting: {' '.join(cmd)}")
        try:
            subprocess.run(cmd, check=False)
        except Exception as e:
            print(f"[WARN] Auto plot failed: {e}")


# ============================================================================
# Entry point
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Threaded admittance teleop for RM75B")
    parser.add_argument("--ip", default=cfg.ROBOT_IP)
    parser.add_argument("--port", type=int, default=cfg.ROBOT_PORT)
    args = parser.parse_args()

    teleop = ThreadedAdmittanceTeleop(args.ip, args.port)

    def _sigint(sig, frame):
        print("\nCtrl+C received — stopping...")
        teleop.stop()

    signal.signal(signal.SIGINT, _sigint)

    teleop.setup()
    teleop.start()
    teleop.join()
    teleop.teardown(slow_stop=True)
    sys.exit(0)


if __name__ == "__main__":
    main()
