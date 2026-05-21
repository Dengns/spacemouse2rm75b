"""SpaceMouse teleoperation configuration for RM75B."""

import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# --- Robot connection ---
ROBOT_IP = "192.168.5.200"
ROBOT_PORT = 8080

# --- Control loop ---
CONTROL_RATE_HZ = 100
ARM_STATE_POLL_HZ = 200

# --- libspnav ---
LIBSPNAV_PATH = os.path.join(_HERE, "libspnav.so.0.4")

# --- SpaceMouse input processing ---
DEADZONE = 40                            # raw axis dead-zone (full range ~+-350)
TRANSLATION_SCALE = 0.0004 / 350.0       # full deflection -> 0.0004 m/cycle (0.02 m/s @ 50Hz)
ROTATION_SCALE = 0.002 / 350.0           # full deflection -> 0.005 rad/cycle (0.25 rad/s @ 50Hz)

MAX_TRANSLATION_PER_CYCLE = 0.001        # m, hard clamp per cycle
MAX_ROTATION_PER_CYCLE = 0.01            # rad, hard clamp per cycle

EMA_ALPHA = 1.0                          # exponential moving average smoothing

# --- Velocity streaming (rm_movev_canfd) ---
# Input mapped to velocity command:
# v_xyz = raw_axis * TRANSLATION_VEL_SCALE (m/s)
# v_rpy = raw_axis * ROTATION_VEL_SCALE (rad/s)
TRANSLATION_VEL_SCALE = 0.01/ 350.0     # full deflection -> 0.10 m/s
ROTATION_VEL_SCALE = 0.050 / 350.0        # full deflection -> 0.50 rad/s
MAX_LINEAR_VEL = 0.10                    # m/s hard clamp
MAX_ANGULAR_VEL = 0.50                   # rad/s hard clamp

# rm_set_movev_canfd_init parameters
MOVEV_AVOID_SINGULARITY = 1              # 0=disable, 1=enable
MOVEV_FRAME_TYPE = 1                     # 0=work frame, 1=tool frame
MOVEV_DT = 0.01                          # seconds, should match loop period at 100 Hz
MOVEV_TRAJECTORY_MODE = 2                # transparent passthrough mode
MOVEV_RADIO = 50                          # acceleration ratio (SDK-defined semantics)

# --- Axis mapping: SpaceMouse -> robot [x, y, z, rx, ry, rz] ---
AXIS_SIGNS = [1, -1, 1, -1, 1, 1]       # flip signs to match intuitive directions
AXIS_MAP = [2, 0, 1, 5, 3, 4]           # reorder axes if needed
AXIS_ENABLE = [1, 1, 1, 0, 0, 0]        # 各轴开关: [x, y, z, rx, ry, rz], 0=屏蔽 1=启用

# --- Workspace limits (meters, in robot base frame) ---
WORKSPACE_MIN = [-0.5, -0.5, 0.0]
WORKSPACE_MAX = [0.5, 0.5, 0.7]

# --- Gripper ---
GRIPPER_OPEN_POS = 1000
GRIPPER_CLOSE_POS = 0
GRIPPER_MODE = "Switching"          # "binary" = 按一下全开/全闭; "incremental" = 按住持续开合;
# “Switching” = USB继电器模块控制继电器吸合，左键闭合，吸起芯片；右键断开，放下芯片
GRIPPER_INCREMENT = 50               # incremental 模式下每周期变化量 (0~1000 范围)

# --- Outputs ---
OUTPUT_DIR = os.path.join(_HERE, "outputs")

# --- USB relay (used when GRIPPER_MODE == "Switching") ---
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD_RATE = 9600
SERIAL_TIMEOUT = 0.5
RELAY_CHANNEL = 1
# --- Outputs ---
OUTPUT_DIR = os.path.join(_HERE, "outputs")

