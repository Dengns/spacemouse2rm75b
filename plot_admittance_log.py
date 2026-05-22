"""画导纳遥操作日志。

用法:
    python3 plot_admittance_log.py [csv_path]
    csv_path 不传时默认读 cfg.RECORD_FILE。

读取列: t, tgt_*, cur_*, cmd_*, f_*, adm_*, age_ms （由
spacemouse_teleop_admittance_threaded.py 的 RecordThread 写入）。

画 4×2 子图:
  行 1-3: x / y / z 位置追踪（target/current/cmd 三条线）+ 同行右侧的力
  行 4:   导纳偏移 xyz / age_ms
"""

import argparse
import os
import sys

import numpy as np

try:
    import matplotlib
    matplotlib.use("TkAgg" if "DISPLAY" in os.environ else "Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("matplotlib not installed. Try: pip install matplotlib")
    sys.exit(1)


def load_csv(path):
    """读取 CSV 并返回字典 {column_name: np.ndarray}。空字符串变 nan。"""
    import csv
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    data = {col: [] for col in header}
    for row in rows:
        for i, col in enumerate(header):
            v = row[i]
            try:
                data[col].append(float(v) if v != "" else np.nan)
            except ValueError:
                data[col].append(np.nan)
    return {k: np.asarray(v) for k, v in data.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", nargs="?", default=None)
    parser.add_argument("--save", default=None,
                        help="不显示窗口，直接保存图片到该路径")
    args = parser.parse_args()

    if args.csv_path is None:
        try:
            import config_admittance_threaded as cfg
            args.csv_path = cfg.RECORD_FILE
        except ImportError:
            print("Need csv_path argument when config not importable")
            sys.exit(1)

    if not os.path.exists(args.csv_path):
        print(f"File not found: {args.csv_path}")
        sys.exit(1)

    print(f"Loading: {args.csv_path}")
    d = load_csv(args.csv_path)
    t = d["t"]
    print(f"  rows: {len(t)}, duration: {t[-1] - t[0]:.2f}s")

    fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
    fig.suptitle(f"Admittance Teleop Log — {os.path.basename(args.csv_path)}")

    # 行 1-3: 平移追踪 (左) + 力 (右)
    for i, axis in enumerate(("x", "y", "z")):
        ax_pos = axes[i, 0]
        ax_pos.plot(t, d[f"tgt_{axis}"] * 1000, label="target",  linewidth=1.5)
        ax_pos.plot(t, d[f"cur_{axis}"] * 1000, label="current", linewidth=1.0)
        ax_pos.plot(t, d[f"cmd_{axis}"] * 1000, label="cmd",     linewidth=1.0,
                    linestyle="--", alpha=0.7)
        ax_pos.set_ylabel(f"{axis} (mm)")
        ax_pos.grid(True, alpha=0.3)
        if i == 0:
            ax_pos.legend(loc="upper right", fontsize=8)

        ax_f = axes[i, 1]
        ax_f.plot(t, d[f"f_{axis}"], label=f"F{axis}", color="red", linewidth=1.0)
        ax_f.plot(t, d[f"adm_{axis}"] * 1000, label=f"adm_{axis}*1000",
                  color="green", linewidth=1.0, alpha=0.7)
        ax_f.set_ylabel(f"F{axis} (N) / adm (mm)")
        ax_f.grid(True, alpha=0.3)
        ax_f.legend(loc="upper right", fontsize=8)
        ax_f.axhline(0, color="gray", linewidth=0.5)

    # 行 4 左: 追踪误差 (current - target)，三轴叠图
    ax_err = axes[3, 0]
    for axis, color in zip(("x", "y", "z"), ("C0", "C1", "C2")):
        err = (d[f"cur_{axis}"] - d[f"tgt_{axis}"]) * 1000
        ax_err.plot(t, err, label=f"err_{axis}", color=color, linewidth=1.0)
    ax_err.set_ylabel("tracking err (mm)")
    ax_err.set_xlabel("time (s)")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="upper right", fontsize=8)
    ax_err.axhline(0, color="gray", linewidth=0.5)

    # 行 4 右: UDP age_ms
    ax_age = axes[3, 1]
    ax_age.plot(t, d["age_ms"], color="purple", linewidth=1.0)
    ax_age.set_ylabel("UDP age (ms)")
    ax_age.set_xlabel("time (s)")
    ax_age.grid(True, alpha=0.3)
    ax_age.axhline(5, color="gray", linewidth=0.5, linestyle="--")  # 期望周期参考线

    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=120)
        print(f"Saved: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
