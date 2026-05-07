import sys
import os
import math
import time
from typing import List, Union
import threading
import numpy as np

# Try pip-installed Robotic-Arm first, fall back to local source
try:
    from Robotic_Arm.rm_robot_interface import (
        RoboticArm,
        rm_thread_mode_e,
        rm_peripheral_read_write_params_t,
        rm_udp_custom_config_t,
        rm_realtime_push_config_t,
        rm_realtime_arm_state_callback_ptr,

    )
except ImportError:
    _local_sdk = os.path.join(os.path.dirname(__file__), "../../../../repos/RMDemo_ModbusRTU-ZhiXing/src")
    if os.path.isdir(_local_sdk):
        sys.path.insert(0, _local_sdk)
    from Robotic_Arm.rm_robot_interface import (
        RoboticArm,
        rm_thread_mode_e,
        rm_peripheral_read_write_params_t,
        rm_udp_custom_config_t,
        rm_realtime_push_config_t,
        rm_realtime_arm_state_callback_ptr,

    )

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

# Modbus constants for ZhiXing 90D gripper
_MODBUS_PORT = 1            # End-effector RS485
_MODBUS_BAUDRATE = 115200
_MODBUS_TIMEOUT = 2         # unit: 100ms
_GRIPPER_DEVICE_ADDR = 1    # Modbus slave address
_GRIPPER_ENABLE_REG = 256   # Write 1 to enable
_GRIPPER_POS_HIGH_REG = 258 # 32-bit position high word
_GRIPPER_POS_LOW_REG = 259  # 32-bit position low word   (range 0~1000)
_GRIPPER_SPEED_REG = 260    # Speed 0~100%
_GRIPPER_TRIGGER_REG = 264  # Write 1 to trigger motion

_GRIPPER_CHANGE_THRESHOLD = 10   # position dead-band (out of 0~1000)
_GRIPPER_MIN_INTERVAL = 0.05     # seconds between Modbus writes


class RM75BInterface:
    """Hardware interface for the RealMan RM75-B 7-DOF arm via TCP/IP,
    with ZhiXing 90D gripper controlled via Modbus RTU over the arm's end-effector RS485.
    """

    def __init__(self, ip: str = "192.168.5.46", port: int = 8080, enable_gripper: bool = True):
        self.ip = ip
        self.port = port
        self.enable_gripper = enable_gripper
        self._last_gripper_pos = -9999  # force first write
        self._last_gripper_time = 0.0

        #udp setting
        self._udp_callback = None
        self._latest_udp_state = None
        self._latest_udp_time = 0.0 
        self._last_udp_print_time = 0.0
        self._udp_lock = threading.Lock()

        self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = self.arm.rm_create_robot_arm(ip, port)
        if handle.id == -1:
            raise RuntimeError(f"Failed to connect to RM75-B at {ip}:{port}")
        print(f"Connected to RM75-B at {ip}:{port}  (handle id={handle.id})")

        if enable_gripper:
            self._init_gripper()

    def _init_gripper(self):
        """Configure Modbus RTU on end-effector RS485 and enable the ZhiXing 90D."""
        ret = self.arm.rm_set_modbus_mode(_MODBUS_PORT, _MODBUS_BAUDRATE, _MODBUS_TIMEOUT)
        if ret != 0:
            print(f"[WARN] rm_set_modbus_mode returned {ret}")
        time.sleep(0.5)

        # Enable gripper: write 1 to register 256
        params = rm_peripheral_read_write_params_t(_GRIPPER_DEVICE_ADDR, _GRIPPER_ENABLE_REG, 1)
        ret = self.arm.rm_write_single_register(params, 1)
        if ret != 0:
            print(f"[WARN] Gripper enable returned {ret}")
        time.sleep(1.0)
        print("ZhiXing 90D gripper enabled via Modbus RTU.")

    def enable_udp_realtime_feedback(self, target_ip: str, target_port: int, cycle_ms: int, force_coordinate: int):
        custom = rm_udp_custom_config_t()
        custom.joint_speed = 0   #关节速度
        custom.lift_state = 0    #升降机构状态
        custom.expand_state = 0  #扩展关节状态

        config = rm_realtime_push_config_t(
            cycle_ms,
            True,
            target_port,
            force_coordinate,
            target_ip,
            custom,
        )

        ret = self.arm.rm_set_realtime_push(config)
        if ret != 0:
            raise RuntimeError(f"rm_set_realtime_push failed (ret={ret})")

        hz = 1000.0 / cycle_ms
        print(
            f"UDP realtime feedback enabled: "
            f"target={target_ip}:{target_port}, "
            f"cycle={cycle_ms}ms, "
            f"frequency={hz:.2f}Hz"
        )

    def register_udp_feedback_callback(self):
        def _on_udp_state(data):
            now = time.monotonic()
            state = data.to_dict(recurse=True)

            with self._udp_lock:
                self._latest_udp_state = state
                self._latest_udp_time = now

            if now - self._last_udp_print_time >= 2.0:
                self._last_udp_print_time = now
                print("[UDP] feedback cached")

        self._udp_callback = rm_realtime_arm_state_callback_ptr(_on_udp_state)
        self.arm.rm_realtime_arm_state_call_back(self._udp_callback)
        print("UDP realtime feedback callback registered.")

    def get_latest_udp_state(self, max_age_s: float = 0.05):
        with self._udp_lock:
            state = self._latest_udp_state
            timestamp = self._latest_udp_time

        if state is None:
            return None, None

        #超过50ms的数据就return None
        age = time.monotonic() - timestamp
        if age > max_age_s:
            return None, age

        return state, age

    def get_joint_positions(self) -> np.ndarray:
        """Returns current joint positions in radians. Shape: (7,)"""
        ret, degrees = self.arm.rm_get_joint_degree()
        if ret != 0:
            print(f"[WARN] rm_get_joint_degree returned {ret}")
            return np.zeros(7)
        return np.array(degrees[:7]) * DEG_TO_RAD

    def get_joint_velocities(self) -> np.ndarray:
        """Returns joint velocities. Shape: (7,)
        Note: RM basic API does not expose velocity feedback."""
        return np.zeros(7)

    def set_joint_positions(self, positions: Union[List[float], np.ndarray]):
        """Set joint positions in radians. Shape: (7,)
        Uses rm_movej_canfd with follow=False (low-following mode) for 50Hz streaming."""
        degrees = [float(p * RAD_TO_DEG) for p in positions]
        ret = self.arm.rm_movej_canfd(degrees, follow=False)
        if ret != 0:
            print(f"[WARN] rm_movej_canfd returned {ret}")

    def _write_gripper_reg(self, addr, value):
        """Write a single 16-bit Modbus register on the gripper."""
        params = rm_peripheral_read_write_params_t(_GRIPPER_DEVICE_ADDR, addr, 1)
        self.arm.rm_write_single_register(params, value)

    def set_gripper_position(self, position: float):
        """Set gripper position (0~1000, 0=closed, 1000=open).
        Writes 32-bit position as two separate 16-bit registers (high + low word),
        then triggers motion. Rate-limited to avoid flooding Modbus."""
        if not self.enable_gripper:
            return

        pos_int = int(max(0, min(1000, round(position))))

        # Skip if position hasn't changed enough
        if abs(pos_int - self._last_gripper_pos) < _GRIPPER_CHANGE_THRESHOLD:
            return

        # Skip if not enough time since last write
        now = time.time()
        if now - self._last_gripper_time < _GRIPPER_MIN_INTERVAL:
            return

        # Write 32-bit position as two 16-bit registers
        self._write_gripper_reg(_GRIPPER_POS_HIGH_REG, (pos_int >> 16) & 0xFFFF)
        self._write_gripper_reg(_GRIPPER_POS_LOW_REG, pos_int & 0xFFFF)
        # Trigger motion
        self._write_gripper_reg(_GRIPPER_TRIGGER_REG, 1)

        self._last_gripper_pos = pos_int
        self._last_gripper_time = now

    def go_home(self, joints_deg=None):
        """Move to home position at 20% speed, blocking.
        Args:
            joints_deg: target joint angles in degrees. None = all zeros.
        """
        if joints_deg is None:
            joints_deg = [0.0] * 7
        ret = self.arm.rm_movej(list(joints_deg), v=20, r=0, connect=0, block=1)
        if ret != 0:
            print(f"[WARN] rm_movej go_home returned {ret}")

    def close(self):
        """Close Modbus port and disconnect from arm."""
        if self.enable_gripper:
            try:
                self.arm.rm_close_modbus_mode(_MODBUS_PORT)
            except Exception as e:
                print(f"[WARN] rm_close_modbus_mode: {e}")
        try:
            self.arm.rm_delete_robot_arm()
        except Exception as e:
            print(f"[WARN] rm_delete_robot_arm: {e}")
        print(f"RM75-B at {self.ip}:{self.port} disconnected.")
