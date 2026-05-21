# spacemouse_teleop_admittance.py 框架梳理

> 说明：原来用了 mermaid 图，若你的编辑器没装 mermaid 插件就看不到。
> 这里换成 ASCII 流程图，任何 markdown 渲染器都能看。

---

## 一、整体结构

```
┌─────────────────────────────────────────────────────────┐
│                       main()                            │
│  1. 解析参数  2. 注册 SIGINT  3. setup()  4. run()      │
└──────────────────────────┬──────────────────────────────┘
                           │
            ┌──────────────┴──────────────┐
            ▼                             ▼
       setup()                         run()
   一次性初始化                       50Hz 主循环
            │                             │
            ▼                             ▼
       teardown()  ◄──────── Ctrl+C ─────┘
       优雅关停
```

---

## 二、setup() 初始化流程

```
  打开 SpaceMouse 线程 (HID 后台读)
            │
            ▼
  连接 RM75B (TCP)
            │
            ▼
  ┌─ UDP_FEEDBACK_ENABLE ? ─┐
  │ 是                       │ 否
  ▼                         ▼
  enable_udp_feedback        (跳过)
  注册 UDP 回调缓存最新帧
            │
            ▼
  ┌─ GRIPPER_MODE = Switching ? ─┐
  │ 是                            │ 否
  ▼                              ▼
  连接 USB 继电器               (跳过)
            │
            ▼
  rm_clear_force_data        ← 力传感器清零
            │
            ▼
  rm_start_force_position_move ← 开启力位混合
            │
            ▼
  读取初始位姿 (UDP 或 TCP)
            │
            ▼
  target_pose ← 当前位姿
```

---

## 三、run() 50Hz 主循环（核心）

```
  ┌─────────────────────────────────────────────────┐
  │ 每周期 dt = 1/CONTROL_RATE_HZ                   │
  └─────────────────────────────────────────────────┘
            │
            ▼
  [1] 读 SpaceMouse 原始 6 轴
            │
            ▼
  [2] _compute_delta:
        映射 → 反向 → EMA → 缩放 → clamp → 使能掩码
            │
            ▼
  [3] 取反馈：
       UDP 模式 ─► read_udp_feedback  (force6, pose, age)
       TCP 模式 ─► _read_force_wrench + rm_get_current_arm_state
            │
            ▼
  [4] 导纳计算 (外环, MBK 二阶模型)         ★ 核心 1
       wrench 去死区 (1N / 0.05Nm)
       a = (wrench - B·v - K·x) / M
       v += a·dt    并 clip(±vel_limit)
       x += v·dt    并 clip(±offset_limit)
       → 输出 _adm_offset
            │
            ▼
  [5] 同步 target (仅 control_mode∈{1,4} 时生效)
            │
            ▼
  [6] target_pose += delta            ← SpaceMouse 增量
            │
            ▼
  [7] _clamp_workspace                ← XYZ 工作空间限制
            │
            ▼
  [8] cmd_pose = target_pose + _adm_offset
            │
            ▼
  [9] 构造 rm_force_position_move_t           ★ 核心 2
       pose         = cmd_pose
       control_mode = [3,3,3,3,3,3]   (六轴柔顺)
       desired_force= [0,0,5,0,0,0]   (Z 向 5N)
       limit_vel    = [0.1, ..., 10, ...]
            │
            ▼
  [10] rm_force_position_move 下发到 SDK
            │
            ▼
  [11] 打印 Z / F / age / Diff
            │
            ▼
  [12] ret != 0 ? 累计失败 ≥10 → rm_set_arm_stop
            │
            ▼
  [13] _handle_buttons → 夹爪/继电器
            │
            ▼
  [14] sleep 补偿到下个周期
            │
            └──────────── 下一周期 ◄──────────┘
```

---

## 四、双环控制结构（重点）

```
        SpaceMouse 输入                力传感器反馈
              │                             │
              ▼                             ▼
        _compute_delta              [外环] 导纳控制
              │                       MBK 二阶模型
              ▼                             │
        target_pose ◄── 增量积分     _adm_offset
              │                             │
              └──────────┬──────────────────┘
                         ▼
                  cmd_pose = target + offset
                         │
                         ▼
              [内环] SDK 力位混合控制
              rm_force_position_move
              control_mode=3 + desired_force
                         │
                         ▼
                    关节指令下发
                         │
                         ▼
                      机械臂
```

| 层级       | 实现位置       | 输入             | 输出             | 作用                         |
|------------|----------------|------------------|------------------|------------------------------|
| 外环 导纳  | Python L259-281| 六维力 force6    | 位置偏移 offset  | 力 → 虚拟弹簧阻尼质量响应    |
| 内环 力位  | SDK 内部       | cmd_pose + 期望力| 关节指令         | mode=3 柔顺补偿              |

---

## 五、_handle_buttons() 夹爪三种模式

```
  ┌─ GRIPPER_MODE ─┐
  │                │
  ├── "binary"     →  按一下立即到位 (open/close)
  │
  ├── "incremental"→  按住持续渐变, 松开停止
  │
  └── "Switching"  →  控制 USB 继电器 (电磁铁开关)
```

---

## 六、teardown() 关停流程

```
  rm_set_arm_slow_stop                ← 机械臂减速停止
            │
            ▼
  ┌─ GRIPPER_MODE != Switching ? ─┐
  │ 是                             │ 否
  ▼                               ▼
  set_gripper_position(OPEN)     (跳过)
  sleep 0.3
            │
            ▼
  arm.close()                          ← 断开 TCP
            │
            ▼
  GRIPPER_MODE == Switching ?
            │ 是
            ▼
  disconnect 继电器
            │
            ▼
  mouse.stop()                         ← 关 SpaceMouse 线程
```

---

## 七、关键状态变量

| 变量             | 含义                                |
|------------------|-------------------------------------|
| `target_pose`    | SpaceMouse 增量积分得到的目标位姿   |
| `_adm_offset`    | 导纳产生的位置偏移                  |
| `_adm_vel`       | 导纳积分用的速度状态                |
| `_smoothed`      | SpaceMouse 输入 EMA 平滑状态        |
| `cmd_pose`       | 最终下发 = target_pose + adm_offset |
| `control_mode`   | [3,3,3,3,3,3] 六轴均柔顺            |
| `desired_force`  | [0,0,5,0,0,0] 仅 Z 向期望 5N        |

---

## 八、外部依赖

```
  SpaceMouse  ──HID──►  spacemouse_input.SpaceMouseReader
  RM75B 臂   ──TCP──►  rm_robot_interface  (命令下发)
             ◄─UDP──   register_udp_feedback_callback (实时反馈)
  USB 继电器 ─串口─►  class_switch.USBRelayController
```
