"""导纳控制可视化模拟器 (单轴 / Z 轴)

不连机械臂，纯粹模拟 spacemouse_teleop_admittance_threaded.py 里的导纳数学，
让你拖滑条调 M/B/K/死区/限速/限位/滤波系数，即时看 adm_offset 响应曲线。

额外支持桌面接触专用的 Z 轴模式：
    - 接触时：只对“压桌面”的单侧法向力做导纳
    - 脱离时：不让弹簧项立刻把 offset 拉回，而是按 release_vel 慢释放
这样更容易调到“xy 正常、z 离桌不明显弹起”的参数。

用法:
    python3 admittance_simulator.py

输入信号四选一（左下 Radio）:
    Step    - 阶跃，t=t0 时从 0 跳到 Amp
    Impulse - 单脉冲（5ms），在 t=t0
    Sine    - 正弦，Amp * sin(2π·Freq·t)
    Manual  - 切到这个模式后，在上图鼠标左键点击加锚点，自动线性插值。Clear 清空。

调好后点 "Print params" 把当前滑条值打印到终端，复制回 config_admittance_threaded.py。
"""

import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons

try:
    import config_admittance_threaded as cfg
except ImportError:
    print("Warning: config_admittance_threaded not found, using fallback defaults.")
    cfg = None


# ------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------

DT = 0.005                       # 5ms，与生产 200Hz 控制一致
DURATION = 5.0                   # 仿真总时长 (s)
N = int(DURATION / DT)
T = np.arange(N) * DT

# Z 轴默认值（index 2）
def _cfg_z(name, fallback):
    if cfg is None:
        return fallback
    return getattr(cfg, name)[2]

DEF_M      = _cfg_z("ADM_M",       3.0)
DEF_B      = _cfg_z("ADM_B",       80.0)
DEF_K      = _cfg_z("ADM_K",       300.0)
DEF_DZ     = _cfg_z("ADM_DEADZONE", 1.0)
DEF_VELLIM = _cfg_z("ADM_VEL_LIMIT", 0.02)
DEF_OFFLIM = _cfg_z("ADM_OFFSET_LIMIT", 0.03)
DEF_ALPHA  = float(getattr(cfg, "ADM_Z_FORCE_LPF_ALPHA", 1.0)) if cfg is not None else 1.0

DEF_TABLE_MODE = bool(getattr(cfg, "ADM_Z_TABLE_MODE_ENABLE", True)) if cfg is not None else True
DEF_CONTACT_SIGN = float(getattr(cfg, "ADM_Z_CONTACT_SIGN", -1.0)) if cfg is not None else -1.0
DEF_CONTACT_ENTER = float(getattr(cfg, "ADM_Z_CONTACT_ENTER_N", 2.0)) if cfg is not None else 2.0
DEF_CONTACT_EXIT = float(getattr(cfg, "ADM_Z_CONTACT_EXIT_N", 0.8)) if cfg is not None else 0.8
DEF_RELEASE_VEL = float(getattr(cfg, "ADM_Z_RELEASE_VEL", 0.004)) if cfg is not None else 0.004

# Default signal
DEF_AMP  = 5.0
DEF_FREQ = 2.0
DEF_T0   = 1.0


# ------------------------------------------------------------------
# Core simulation (faithfully replicates ControlThread admittance)
# ------------------------------------------------------------------


def _move_towards(value, target, max_step):
    if max_step <= 0.0:
        return value
    delta = target - value
    if abs(delta) <= max_step:
        return target
    return value + np.sign(delta) * max_step


def simulate(
    F,
    M,
    B,
    K,
    deadzone,
    vel_lim,
    off_lim,
    alpha,
    table_mode=False,
    contact_sign=-1.0,
    contact_enter=2.0,
    contact_exit=0.8,
    release_vel=0.004,
    dt=DT,
):
    """单轴导纳递推，返回 (offset[N], vel[N], wrench[N], compression[N], contact[N])。"""
    f_filt = 0.0
    vel = 0.0
    offset = 0.0
    in_contact = False
    out_off = np.zeros_like(F)
    out_vel = np.zeros_like(F)
    out_w   = np.zeros_like(F)
    out_comp = np.zeros_like(F)
    out_contact = np.zeros_like(F)
    for i, f in enumerate(F):
        # Low-pass filter on force (alpha=1.0 → pass-through)
        f_filt = alpha * f + (1.0 - alpha) * f_filt

        if table_mode:
            sign = contact_sign if abs(contact_sign) > 1e-6 else -1.0
            compression = max(0.0, sign * f_filt)
            if in_contact:
                if compression <= contact_exit:
                    in_contact = False
            elif compression >= contact_enter:
                in_contact = True

            if in_contact:
                # Z 轴桌面模式使用减法软死区，避免接触阈值附近突跳。
                w = max(0.0, compression - deadzone)
                a = (w - B * vel - K * offset) / M
                vel = np.clip(vel + a * dt, -vel_lim, vel_lim)
                offset = np.clip(offset + vel * dt, -off_lim, off_lim)
            else:
                w = 0.0
                vel = 0.0
                offset = _move_towards(offset, 0.0, release_vel * dt)
        else:
            compression = 0.0
            # Hard deadzone (与生产代码一致)
            w = 0.0 if abs(f_filt) < deadzone else f_filt
            # MBK integration
            a = (w - B * vel - K * offset) / M
            vel = np.clip(vel + a * dt, -vel_lim, vel_lim)
            offset = np.clip(offset + vel * dt, -off_lim, off_lim)

        out_off[i] = offset
        out_vel[i] = vel
        out_w[i]   = w
        out_comp[i] = compression
        out_contact[i] = 1.0 if in_contact else 0.0
    return out_off, out_vel, out_w, out_comp, out_contact


# ------------------------------------------------------------------
# Signal generation
# ------------------------------------------------------------------

def make_force(kind, amp, freq, t0, manual_anchors):
    F = np.zeros(N)
    if kind == "Step":
        F[T >= t0] = amp
    elif kind == "Impulse":
        idx = int(t0 / DT)
        if 0 <= idx < N:
            F[idx] = amp
    elif kind == "Sine":
        F = amp * np.sin(2.0 * np.pi * freq * T)
    elif kind == "Manual":
        if len(manual_anchors) >= 1:
            anchors = sorted(manual_anchors, key=lambda a: a[0])
            ts = np.array([a[0] for a in anchors])
            fs = np.array([a[1] for a in anchors])
            F = np.interp(T, ts, fs, left=0.0, right=0.0)
    return F


# ------------------------------------------------------------------
# App
# ------------------------------------------------------------------

class AdmittanceSimulatorApp:
    def __init__(self):
        self.signal_kind = "Step"
        self.manual_anchors = []   # list of (t, F)
        self.z_table_mode = DEF_TABLE_MODE
        self.contact_sign = DEF_CONTACT_SIGN if abs(DEF_CONTACT_SIGN) > 1e-6 else -1.0

        self.fig = plt.figure(figsize=(13, 10))
        self.fig.canvas.manager.set_window_title("Admittance Simulator (Z axis)")

        # ----- Top plots -----
        self.ax_force = self.fig.add_axes([0.07, 0.78, 0.88, 0.18])
        self.ax_resp  = self.fig.add_axes([0.07, 0.55, 0.88, 0.18])
        self.ax_resp_vel = self.ax_resp.twinx()

        # ----- Radio: signal kind -----
        ax_radio = self.fig.add_axes([0.05, 0.36, 0.10, 0.14])
        ax_radio.set_title("Signal", fontsize=10)
        self.radio = RadioButtons(ax_radio, ("Step", "Impulse", "Sine", "Manual"), active=0)
        self.radio.on_clicked(self._on_signal_change)

        # ----- Check: z desk mode -----
        ax_check = self.fig.add_axes([0.05, 0.30, 0.12, 0.05])
        ax_check.set_title("Z mode", fontsize=10)
        self.check = CheckButtons(ax_check, ["Desk contact"], [self.z_table_mode])
        self.check.on_clicked(self._on_mode_toggle)

        # ----- Radio: contact sign -----
        ax_sign = self.fig.add_axes([0.05, 0.18, 0.12, 0.10])
        ax_sign.set_title("Contact sign", fontsize=10)
        sign_active = 0 if self.contact_sign < 0 else 1
        self.radio_sign = RadioButtons(ax_sign, ("Fz<0", "Fz>0"), active=sign_active)
        self.radio_sign.on_clicked(self._on_sign_change)

        # ----- Buttons -----
        ax_clear = self.fig.add_axes([0.05, 0.11, 0.10, 0.035])
        self.btn_clear = Button(ax_clear, "Clear")
        self.btn_clear.on_clicked(self._on_clear)

        ax_print = self.fig.add_axes([0.05, 0.065, 0.10, 0.035])
        self.btn_print = Button(ax_print, "Print params")
        self.btn_print.on_clicked(self._on_print)

        # ----- Sliders -----
        slider_x = 0.30
        slider_w = 0.62
        s_h = 0.018

        def make_slider(y, label, vmin, vmax, vinit, valfmt="%.3f"):
            ax = self.fig.add_axes([slider_x, y, slider_w, s_h])
            return Slider(ax, label, vmin, vmax, valinit=vinit, valfmt=valfmt)

        # Signal params
        self.s_amp  = make_slider(0.50, "Amp (N)",  0.0, 10.0, DEF_AMP,  valfmt="%.2f")
        self.s_freq = make_slider(0.465, "Freq (Hz)", 0.5, 20.0, DEF_FREQ, valfmt="%.2f")
        self.s_t0   = make_slider(0.43, "t0 (s)",    0.0,  4.0, DEF_T0,   valfmt="%.2f")

        # Admittance params
        self.s_M      = make_slider(0.385, "M",            0.5,  10.0,   DEF_M,      valfmt="%.2f")
        self.s_B      = make_slider(0.35, "B",           10.0, 200.0,   DEF_B,      valfmt="%.1f")
        self.s_K      = make_slider(0.315, "K",           50.0, 500.0,   DEF_K,      valfmt="%.1f")
        self.s_dz     = make_slider(0.28, "Deadzone (N)", 0.0,   3.0,   DEF_DZ,     valfmt="%.2f")
        self.s_vlim   = make_slider(0.245, "VelLim (m/s)", 0.005, 0.05,  DEF_VELLIM, valfmt="%.4f")
        self.s_olim   = make_slider(0.21, "OffLim (m)",   0.01,  0.10,  DEF_OFFLIM, valfmt="%.4f")
        self.s_alpha  = make_slider(0.175, "Alpha (LPF)",  0.05,  1.0,   DEF_ALPHA,  valfmt="%.3f")
        self.s_enter  = make_slider(0.14, "Contact ON (N)", 0.2, 10.0, DEF_CONTACT_ENTER, valfmt="%.2f")
        self.s_exit   = make_slider(0.105, "Contact OFF (N)", 0.0, 10.0, DEF_CONTACT_EXIT, valfmt="%.2f")
        self.s_relvel = make_slider(0.07, "Release vel (m/s)", 0.0, 0.03, DEF_RELEASE_VEL, valfmt="%.4f")

        for s in (self.s_amp, self.s_freq, self.s_t0,
                  self.s_M, self.s_B, self.s_K, self.s_dz,
                  self.s_vlim, self.s_olim, self.s_alpha,
                  self.s_enter, self.s_exit, self.s_relvel):
            s.on_changed(self._update)

        # ----- Mouse click for Manual mode -----
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        # ----- Initial draw -----
        self._update()

    # --------------------------------------------------------------
    # Callbacks
    # --------------------------------------------------------------

    def _on_signal_change(self, label):
        self.signal_kind = label
        self._update()

    def _on_mode_toggle(self, _label):
        self.z_table_mode = not self.z_table_mode
        self._update()

    def _on_sign_change(self, label):
        self.contact_sign = -1.0 if label == "Fz<0" else 1.0
        self._update()

    def _on_clear(self, event):
        self.manual_anchors = []
        self._update()

    def _on_print(self, event):
        msg = (
            f"\n=== Current slider values (Z axis) ===\n"
            f"  ADM_M[2]            = {self.s_M.val:.3f}\n"
            f"  ADM_B[2]            = {self.s_B.val:.3f}\n"
            f"  ADM_K[2]            = {self.s_K.val:.3f}\n"
            f"  ADM_DEADZONE[2]     = {self.s_dz.val:.3f}\n"
            f"  ADM_VEL_LIMIT[2]    = {self.s_vlim.val:.5f}\n"
            f"  ADM_OFFSET_LIMIT[2] = {self.s_olim.val:.5f}\n"
            f"  ADM_Z_TABLE_MODE_ENABLE = {self.z_table_mode}\n"
            f"  ADM_Z_CONTACT_SIGN      = {self.contact_sign:.1f}\n"
            f"  ADM_Z_CONTACT_ENTER_N   = {self.s_enter.val:.3f}\n"
            f"  ADM_Z_CONTACT_EXIT_N    = {self.s_exit.val:.3f}\n"
            f"  ADM_Z_FORCE_LPF_ALPHA   = {self.s_alpha.val:.3f}\n"
            f"  ADM_Z_RELEASE_VEL       = {self.s_relvel.val:.5f}\n"
            f"========================================"
        )
        print(msg)

    def _on_click(self, event):
        if event.inaxes is not self.ax_force:
            return
        if self.signal_kind != "Manual":
            return
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 1:   # left click → add anchor
            t = max(0.0, min(DURATION, event.xdata))
            self.manual_anchors.append((t, event.ydata))
            self._update()
        elif event.button == 3: # right click → remove nearest anchor
            if not self.manual_anchors:
                return
            x = event.xdata
            i_near = min(range(len(self.manual_anchors)),
                         key=lambda i: abs(self.manual_anchors[i][0] - x))
            self.manual_anchors.pop(i_near)
            self._update()

    # --------------------------------------------------------------
    # Plot update
    # --------------------------------------------------------------

    def _update(self, _val=None):
        F = make_force(
            self.signal_kind,
            self.s_amp.val, self.s_freq.val, self.s_t0.val,
            self.manual_anchors,
        )
        offset, vel, w, compression, contact = simulate(
            F,
            self.s_M.val, self.s_B.val, self.s_K.val,
            self.s_dz.val, self.s_vlim.val, self.s_olim.val,
            self.s_alpha.val,
            table_mode=self.z_table_mode,
            contact_sign=self.contact_sign,
            contact_enter=self.s_enter.val,
            contact_exit=self.s_exit.val,
            release_vel=self.s_relvel.val,
        )

        # --- Force panel ---
        ax = self.ax_force
        ax.clear()
        ax.plot(T, F, color="C3", label="Force input", linewidth=1.2)
        if self.z_table_mode:
            ax.plot(T, compression, color="C0", linestyle=":", alpha=0.9,
                    label="compression", linewidth=1.0)
        if self.s_alpha.val < 1.0 - 1e-6 or self.s_dz.val > 0 or self.z_table_mode:
            ax.plot(T, w, color="C1", linestyle="--", alpha=0.7,
                    label="effective wrench", linewidth=1.0)
        dz = self.s_dz.val
        if dz > 0:
            ax.axhline( dz, color="gray", linestyle=":", alpha=0.5)
            ax.axhline(-dz, color="gray", linestyle=":", alpha=0.5)
        if self.signal_kind == "Manual" and self.manual_anchors:
            xs = [a[0] for a in self.manual_anchors]
            ys = [a[1] for a in self.manual_anchors]
            ax.plot(xs, ys, "o", color="blue", markersize=5,
                    label=f"anchors ({len(xs)})")
        title = f"Force input — {self.signal_kind}"
        if self.signal_kind == "Manual":
            title += "  (left-click: add, right-click: remove nearest)"
        if self.z_table_mode:
            title += "  |  Z desk mode ON"
        ax.set_title(title, fontsize=10)
        ax.set_ylabel("N")
        ax.set_xlim(0, DURATION)
        if self.z_table_mode and np.any(contact > 0.5):
            ymin, ymax = ax.get_ylim()
            ax.fill_between(T, ymin, ymax, where=contact > 0.5, color="C2", alpha=0.05)
            ax.set_ylim(ymin, ymax)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

        # --- Response panel ---
        self.ax_resp.clear()
        self.ax_resp_vel.clear()

        l1, = self.ax_resp.plot(T, offset * 1000, color="C2",
                                 label="offset (mm)", linewidth=1.4)
        l2, = self.ax_resp_vel.plot(T, vel * 1000, color="C4",
                                     alpha=0.5, linestyle="-",
                                     label="vel (mm/s)", linewidth=1.0)
        olim_mm = self.s_olim.val * 1000.0
        self.ax_resp.axhline( olim_mm, color="gray", linestyle=":", alpha=0.5)
        self.ax_resp.axhline(-olim_mm, color="gray", linestyle=":", alpha=0.5)

        # Stats text
        steady = offset[-50:].mean() * 1000  # 末段平均做稳态
        peak = np.max(np.abs(offset)) * 1000
        mode_name = "desk release" if self.z_table_mode else "symmetric MBK"
        contact_ratio = 100.0 * contact.mean() if self.z_table_mode else 0.0
        self.ax_resp.set_title(
            f"Admittance response ({mode_name}) — steady ≈ {steady:.2f}mm, "
            f"peak |off| ≈ {peak:.2f}mm"
            + (f", contact {contact_ratio:.0f}%" if self.z_table_mode else ""),
            fontsize=10,
        )
        self.ax_resp.set_ylabel("offset (mm)", color="C2")
        self.ax_resp_vel.set_ylabel("vel (mm/s)", color="C4")
        self.ax_resp.set_xlabel("time (s)")
        self.ax_resp.set_xlim(0, DURATION)
        self.ax_resp.grid(True, alpha=0.3)
        self.ax_resp.legend([l1, l2], [l1.get_label(), l2.get_label()],
                            loc="upper right", fontsize=8)

        self.fig.canvas.draw_idle()


# ------------------------------------------------------------------
# Entry
# ------------------------------------------------------------------

def main():
    app = AdmittanceSimulatorApp()
    plt.show()


if __name__ == "__main__":
    main()
