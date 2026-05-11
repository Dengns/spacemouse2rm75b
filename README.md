# SpaceMouse + RM75B Toolkit

SpaceMouse 控制 RealMan RM75-B 的一组脚本，包含：
- 实时遥操作
- 位置/速度模式运动录制
- CSV 结果可视化出图

## Repo Structure

```text
spacemouse2rm75b/
├── config.py
├── rm75b.py
├── spacemouse_input.py
├── spacemouse_teleop.py
├── record_motion.py
├── record_motionteststop.py
├── record_motionteststopfollow.py
├── record_motionteststop_velocity.py   # 新增：速度透传录制
├── test_latency.py
├── plot_motion.py
├── libspnav.so.0.4
├── outputs/                            # 录制数据和图统一放这里
└── README.md
```

## Dependencies

- `numpy`
- `matplotlib`
- `Robotic-Arm` (RealMan Python SDK)
- `spacenavd` (SpaceMouse daemon)

## Quick Start

1. 修改 `config.py` 里的机器人 IP。  
2. 确认 `spacenavd` 正常运行。  
3. 运行脚本。

### 速度模式录制（推荐）

```bash
python3 record_motionteststop_velocity.py --ip 192.168.5.105 --hz 100
```

默认输出：
- CSV: `outputs/motion_velocity.csv`
- PNG: `outputs/motion_velocity.png`（脚本结束后自动生成）

如果要用 UDP 实时反馈重新测试 actual pose 误差：

```bash
python3 record_motionteststop_velocity.py --ip 192.168.1.18 --hz 100 --feedback udp --udp-target-ip 192.168.1.104 --out outputs/motion_velocity_udp.csv
```

其中 `--udp-target-ip` 是运行 Python 程序这台电脑的 IP，不是机器人 IP。

### 手动绘图

```bash
python3 plot_motion.py outputs/motion_velocity.csv --no-show
```

## Notes

- `record_motionteststop_velocity.py` 使用 `rm_movev_canfd`（速度透传）。
- `plot_motion.py` 同时兼容旧 CSV 和新速度 CSV（带 `cmd_v*` 字段）。
- 所有历史 `csv/png` 已整理进 `outputs/`。
