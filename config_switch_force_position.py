"""SpaceMouse teleoperation configuration for RM75B."""

import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# --- Robot connection ---
ROBOT_IP = "192.168.1.18"
ROBOT_PORT = 8080

# --- UDP realtime feedback ---
UDP_FEEDBACK_ENABLE = True
UDP_TARGET_IP = "192.168.1.104"  # 运行 Python 程序的电脑 IP，不是机器人 IP
UDP_TARGET_PORT = 8089
UDP_CYCLE_MS = 5                  # 200HZ
UDP_FORCE_COORDINATE = 0          # 0=传感器坐标系, 1=工作坐标系, 2=工具坐标系
UDP_TIMEOUT_S = 0.05              # 超过 50ms 没收到反馈，就断


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

# --- Force-position streaming (rm_force_position_move_pose) ---
# Signature:

# --- Directional Z-force limit (allow Z up, block Z down when triggered) ---
# 这里单纯是数值，方向在主程序里面加了负号
FORCE_Z_LIMIT_ENABLE = True             # True: enable directional Fz protection
FORCE_Z_LIMIT_DIRECTION = 1             # +1: trigger when Fz > threshold, -1: trigger when Fz < -threshold
FORCE_Z_LIMIT_TRIGGER_N = 1.0           # lock threshold (N), enters "Z down blocked" state 锁定阈值
FORCE_Z_LIMIT_RELEASE_N = 0.6           # unlock threshold (N), should be lower than trigger for hysteresis 放松的阈值

# --- Axis mapping: SpaceMouse -> robot [x, y, z, rx, ry, rz] ---
AXIS_SIGNS = [1, -1, 1, -1, 1, 1]       # flip signs to match intuitive directions
AXIS_MAP = [2, 0, 1, 5, 3, 4]           # reorder axes if needed
AXIS_ENABLE = [1, 1, 1, 0, 0, 1]        # 各轴开关: [x, y, z, rx, ry, rz], 0=屏蔽 1=启用

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


