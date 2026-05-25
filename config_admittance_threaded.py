"""SpaceMouse threaded admittance teleoperation config for RM75B."""

import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# --- Robot connection ---
ROBOT_IP = "192.168.5.200"
ROBOT_PORT = 8080

# --- UDP realtime feedback ---
UDP_FEEDBACK_ENABLE = True
UDP_TARGET_IP = "192.168.5.123"  # 运行 Python 程序的电脑 IP，不是机器人 IP
UDP_TARGET_PORT = 8089
UDP_CYCLE_MS = 1                  # 实际周期 = UDP_CYCLE_MS * 5 ms (1 → 5ms → 200Hz)
UDP_FORCE_COORDINATE = 1          # 0=传感器坐标系, 1=工作坐标系, 2=工具坐标系
UDP_TIMEOUT_S = 0.05              # 超过 50ms 没收到反馈视为失效

# --- Multi-thread frequencies ---
INPUT_RATE_HZ   = 100            # SpaceMouseInputThread
CONTROL_RATE_HZ = 200             # ControlThread (与 UDP 同频)
RECORD_RATE_HZ  = 200             # RecordThread (与控制同频，可看到高频抖动)

# --- libspnav ---
LIBSPNAV_PATH = os.path.join(_HERE, "libspnav.so.0.4")

# --- SpaceMouse input processing ---
DEADZONE = 40                            # raw axis dead-zone (full range ~+-350)
# 注意：积分量级 = SCALE * 输入_RATE_HZ。100Hz 下与原 50Hz 比要减半 per-cycle 量
TRANSLATION_SCALE = 0.0004 / 350.0       # full deflection -> 0.0002 m/cycle @100Hz = 0.02 m/s
ROTATION_SCALE    = 0.001  / 350.0       # full deflection -> 0.001 rad/cycle @100Hz = 0.10 rad/s

MAX_TRANSLATION_PER_CYCLE = 0.0005       # m, hard clamp per input cycle
MAX_ROTATION_PER_CYCLE    = 0.005        # rad, hard clamp per input cycle

EMA_ALPHA = 1.0                          # 1.0 = no smoothing (直接取最新输入)

# --- Axis mapping: SpaceMouse -> robot [x, y, z, rx, ry, rz] ---
AXIS_SIGNS  = [1, -1, 1, -1, 1, 1]
AXIS_MAP    = [2, 0, 1, 5, 3, 4]
AXIS_ENABLE = [1, 1, 1, 0, 0, 1]         # 0=屏蔽 1=启用

# --- Workspace limits (meters, in robot base frame) ---
WORKSPACE_MIN = [-0.5, -0.5, 0.0]
WORKSPACE_MAX = [ 0.5,  0.5, 0.7]

# --- Admittance parameters (M-B-K, 6-DoF, [x y z rx ry rz]) ---
ADM_ENABLE        = True                                   # 总开关：False 时跳过全部导纳，cmd_pose = target_pose
ADM_AXIS_ENABLE   = [1, 1, 1, 0, 0, 0]                     # 各轴导纳开关 [x y z rx ry rz]，0=关 1=开
ADM_M             = [3.0,  3.0,  3.0,  0.15, 0.15, 0.15]   # 虚拟质量
ADM_B             = [80.0, 80.0, 800.0, 3.0,  3.0,  3.0]    # 阻尼
ADM_K             = [300.0, 300.0, 800.0, 8.0, 8.0, 8.0]   # 刚度
ADM_DEADZONE      = [1.0,  1.0,  1.0,  0.05, 0.05, 0.05]   # 力/力矩死区 (N / N·m)
ADM_VEL_LIMIT     = [0.02, 0.02, 0.02, 0.15, 0.15, 0.15]   # 导纳速度限
ADM_OFFSET_LIMIT  = [0.03, 0.03, 0.03, 0.20, 0.20, 0.20]   # 导纳位移限

# --- Z-axis desk contact tuning ---
# 目标：xy 保持普通对称导纳；z 仅在“压桌面”这一侧进入柔顺，脱离接触后慢释放，避免明显弹起。
ADM_Z_TABLE_MODE_ENABLE = True
ADM_Z_CONTACT_SIGN      = 1.0    # 取值使得“压桌面时” ADM_Z_CONTACT_SIGN * Fz > 0
ADM_Z_CONTACT_ENTER_N   = 2.0     # 压缩法向力超过该值 -> 进入接触状态
ADM_Z_CONTACT_EXIT_N    = 0.8     # 压缩法向力低于该值 -> 退出接触状态（滞回避免抖动）
ADM_Z_FORCE_LPF_ALPHA   = 0.35    # Z 向接触判定低通；1.0=不过滤
ADM_Z_RELEASE_VEL       = 0.008   # m/s，脱离接触后 adm_offset[z] 回零的最大速度
ADM_Z_LOCK_TARGET_WHEN_IN_CONTACT = False  # 接触时禁止继续向下积累 target_pose[z]

# --- rm_movep_canfd 下发参数 ---
MOVEP_FOLLOW = True                      # True 高跟随 (要求周期 <= 10ms)
MOVEP_TRAJECTORY_MODE = 2                # 0=完全透传, 1=曲线拟合, 2=滤波
MOVEP_RADIO = 400                         # 平滑系数: 曲线拟合 0~100, 滤波 0~999, 越大越光滑
MOVEP_MAX_CONSECUTIVE_FAILS = 10         # 连续失败超过此数 → emergency stop

# --- Gripper ---
GRIPPER_OPEN_POS = 1000
GRIPPER_CLOSE_POS = 0
GRIPPER_MODE = "Switching"               # "binary" / "incremental" / "Switching"
GRIPPER_INCREMENT = 50                   # incremental 模式下每周期变化量

# --- USB relay (used when GRIPPER_MODE == "Switching") ---
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD_RATE = 9600
SERIAL_TIMEOUT = 0.5
RELAY_CHANNEL = 1

# --- Outputs ---
OUTPUT_DIR = os.path.join(_HERE, "outputs")

# --- Recording / plotting ---
RECORD_ENABLE       = True                                  # True 时把每周期状态写到 CSV
RECORD_FILE         = os.path.join(OUTPUT_DIR, "admittance_log.csv")
RECORD_PLOT_ON_EXIT = True                                  # True 时退出后自动调 plot_admittance_log.py
