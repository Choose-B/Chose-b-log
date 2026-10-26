"""
Generate figures for source/_posts/功率控制下的力控底盘.md.

Output (source/img/power-chassis/):
- framework.svg         控制框架
- geometry.svg          X 型全向轮底盘几何与轮号
- power-quadratic.svg   动作功率二次函数与 rho 的取值
- gradient-descent.svg  梯度下降限制 v_tar
- airborne-fsm.svg      滞空与落地状态机
- banner.avif           文章顶部大图 (banner_img)
- index.avif            首页封面 (index_img)

Wheel numbering follows B_tau in the post: viewed from above with x forward,
1 = front-right, 2 = rear-right, 3 = rear-left, 4 = front-left, and positive
tau_i pushes the chassis counter-clockwise.

Usage:
    python3 tools/figures/power_chassis.py
"""

import io
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon
from PIL import Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "source", "img", "power-chassis")

INK = "#1f2933"
MUTED = "#6b7686"
LINE = "#c9d1dc"
BLUE = "#2f6fde"
ORANGE = "#e8833a"
GREEN = "#2e9d6a"
RED = "#d64545"
PURPLE = "#7c5cd6"
FILL_BLUE = "#eaf1fd"
FILL_ORANGE = "#fdf1e7"
FILL_GREEN = "#e8f6ef"
FILL_GRAY = "#f3f5f8"

plt.rcParams.update({
    "font.family": "Noto Sans CJK SC",
    "mathtext.fontset": "stix",
    "svg.fonttype": "path",
    "axes.unicode_minus": False,
    "font.size": 12,
    "text.color": INK,
    "axes.labelcolor": INK,
    "axes.edgecolor": MUTED,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
})


# ---------------------------------------------------------------- geometry

def chassis_shapes(R=0.9, wheel_hl=0.30, wheel_hw=0.12, rot=0.0):
    """X-type omni chassis in page coordinates (forward = +y), rotated by rot (rad)."""
    c, s = np.cos(rot), np.sin(rot)
    rmat = np.array([[c, -s], [s, c]])

    a = 0.72
    k = (R - wheel_hw - 0.04) * np.sqrt(2) - a
    body = np.array([(a, k), (k, a), (-k, a), (-a, k),
                     (-a, -k), (-k, -a), (k, -a), (a, -k)]) @ rmat.T

    wheels = []
    for theta in np.deg2rad([45, -45, -135, 135]):
        radial = np.array([np.cos(theta), np.sin(theta)])
        drive = np.array([-np.sin(theta), np.cos(theta)])
        center = R * radial
        poly = np.array([center + drive * wheel_hl + radial * wheel_hw,
                         center - drive * wheel_hl + radial * wheel_hw,
                         center - drive * wheel_hl - radial * wheel_hw,
                         center + drive * wheel_hl - radial * wheel_hw])
        rollers = [(center + drive * t + radial * wheel_hw, center + drive * t - radial * wheel_hw)
                   for t in np.linspace(-wheel_hl, wheel_hl, 8)[1:-1]]
        wheels.append({
            "center": rmat @ center,
            "radial": rmat @ radial,
            "drive": rmat @ drive,
            "poly": poly @ rmat.T,
            "rollers": [(rmat @ p, rmat @ q) for p, q in rollers],
        })
    return body, wheels


def arrow(ax, p0, p1, color=INK, lw=1.6, style="-|>", ms=14, ls="-", cs="arc3", z=3):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle=style, mutation_scale=ms, color=color,
                                 lw=lw, linestyle=ls, connectionstyle=cs, zorder=z,
                                 shrinkA=0, shrinkB=0))


def box(ax, x, y, w, h, fc, ec, lw=1.4, r=0.18, z=1):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=f"round,pad=0,rounding_size={r}",
                                fc=fc, ec=ec, lw=lw, zorder=z))


def canvas(w, h, xlim, ylim):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.subplots_adjust(0, 0, 1, 1)
    return fig, ax


# ---------------------------------------------------------------- inline figures

def fig_framework():
    fig, ax = canvas(13, 6.4, (0, 16.2), (0, 8))

    # main layers
    box(ax, 0.4, 3.0, 3.4, 2.4, FILL_ORANGE, ORANGE)
    ax.text(2.1, 4.75, "能量控制", ha="center", va="center", fontsize=15, weight="bold")
    ax.text(2.1, 4.05, "能量环 PD", ha="center", va="center", color=MUTED)
    ax.text(2.1, 3.5, r"$P_{exp}=P_{PID}+P_{referee}$", ha="center", va="center", fontsize=12)

    box(ax, 5.6, 2.6, 5.0, 3.2, FILL_BLUE, BLUE)
    ax.text(8.1, 5.3, "功率规划", ha="center", va="center", fontsize=15, weight="bold")
    box(ax, 5.9, 3.95, 4.4, 0.85, "white", LINE, lw=1, r=0.1, z=2)
    ax.text(6.1, 4.375, "① 速度限制", va="center", fontsize=12, zorder=3)
    ax.text(10.1, 4.375, r"梯度下降 $\rightarrow v_{tar}$", ha="right", va="center",
            fontsize=12, color=MUTED, zorder=3)
    box(ax, 5.9, 2.85, 4.4, 0.85, "white", LINE, lw=1, r=0.1, z=2)
    ax.text(6.1, 3.275, "② 加速度限幅", va="center", fontsize=12, zorder=3)
    ax.text(10.1, 3.275, r"选取 $\rho \rightarrow a_{tar}$", ha="right", va="center",
            fontsize=12, color=MUTED, zorder=3)

    box(ax, 12.4, 3.0, 3.4, 2.4, FILL_GREEN, GREEN)
    ax.text(14.1, 4.75, "电机输出", ha="center", va="center", fontsize=15, weight="bold")
    ax.text(14.1, 4.05, "速度环 PID", ha="center", va="center", color=MUTED)
    ax.text(14.1, 3.45, r"$I_{out}=\frac{k_\tau}{N}\tau_{out}$", ha="center", va="center", fontsize=13)

    # lower row
    box(ax, 5.6, 0.35, 5.0, 1.45, FILL_GRAY, MUTED)
    ax.text(8.1, 1.38, "FF-RLS 系统辨识", ha="center", va="center", fontsize=13, weight="bold")
    ax.text(8.1, 0.78, r"电机 $k_1,\ k_2,\ c_i$      底盘 $m,\ J,\ C_1 \sim C_3$",
            ha="center", va="center", fontsize=11.5, color=MUTED)
    box(ax, 12.4, 0.35, 3.4, 1.45, FILL_GRAY, MUTED)
    ax.text(14.1, 1.08, "轮组电机 ×4", ha="center", va="center", fontsize=13, weight="bold")

    # external inputs
    ax.text(2.1, 7.25, "裁判系统 / 超级电容", ha="center", va="center", color=MUTED, fontsize=11)
    ax.text(2.1, 6.65, r"$W_{mea},\ W_{exp},\ W_{buf},\ P_{req}$", ha="center", va="center")
    arrow(ax, (2.1, 6.3), (2.1, 5.45))
    ax.text(6.9, 7.25, "决策层", ha="center", va="center", color=MUTED, fontsize=11)
    ax.text(6.9, 6.65, r"$v_{exp}$", ha="center", va="center", fontsize=13)
    arrow(ax, (6.9, 6.3), (6.9, 5.85))
    ax.text(9.3, 7.25, "传感器", ha="center", va="center", color=MUTED, fontsize=11)
    ax.text(9.3, 6.65, r"$v_{mea},\ g_{mea},\ P_{mea}$", ha="center", va="center")
    arrow(ax, (9.3, 6.3), (9.3, 5.85))

    # main flow
    arrow(ax, (3.85, 4.2), (5.55, 4.2), color=ORANGE, lw=2)
    ax.text(4.7, 4.45, r"$P_{exp}$", ha="center", va="bottom", fontsize=13)
    arrow(ax, (10.65, 4.6), (12.35, 4.6), color=BLUE, lw=2)
    ax.text(11.5, 4.85, r"$v_{tar},\ \tau_{out}$", ha="center", va="bottom", fontsize=13)
    arrow(ax, (12.35, 3.6), (10.65, 3.6), color=GREEN, lw=1.6, ls="--")
    ax.text(11.5, 3.35, r"$\tau_{PID}$", ha="center", va="top", fontsize=13)
    arrow(ax, (14.1, 2.95), (14.1, 1.85), color=GREEN, lw=2)
    ax.text(14.25, 2.4, r"$I_{out}$", va="center", fontsize=13)
    arrow(ax, (12.35, 1.08), (10.65, 1.08), color=MUTED)
    ax.text(11.5, 1.25, r"$P_{mea,i},\ \omega,\ \tau$", ha="center", va="bottom", fontsize=11.5)
    arrow(ax, (8.1, 1.85), (8.1, 2.55), color=MUTED)
    ax.text(8.25, 2.2, r"$\hat{\theta}_k$", va="center", fontsize=13)
    return fig


def fig_geometry():
    fig, ax = canvas(6.6, 6.9, (-1.75, 1.75), (-1.95, 1.72))
    body, wheels = chassis_shapes()

    ax.add_patch(Polygon(body, closed=True, fc=FILL_GRAY, ec=MUTED, lw=1.5, zorder=1))
    for i, w in enumerate(wheels, start=1):
        ax.add_patch(Polygon(w["poly"], closed=True, fc="white", ec=INK, lw=1.5, zorder=2))
        for p, q in w["rollers"]:
            ax.plot(*zip(p, q), color=MUTED, lw=0.9, zorder=3)
        tail = w["center"] + w["drive"] * 0.33
        head = w["center"] + w["drive"] * 0.78
        arrow(ax, tail, head, color=ORANGE, lw=2.2, ms=16)
        lab = head + w["drive"] * 0.1 - w["radial"] * 0.12
        ax.text(*lab, rf"$\tau_{i}$", ha="center", va="center", fontsize=15, color=ORANGE)
        num = w["center"] - w["drive"] * 0.52 + w["radial"] * 0.08
        ax.add_patch(plt.Circle(num, 0.12, fc=INK, ec="none", zorder=4))
        ax.text(*num, str(i), ha="center", va="center", color="white", fontsize=12,
                weight="bold", zorder=5)

    # R
    c1 = wheels[0]["center"]
    ax.plot([0, c1[0]], [0, c1[1]], color=BLUE, lw=1.2, ls="--", zorder=2)
    ax.text(*(c1 * 0.55 + np.array([0.09, -0.09])), r"$R$", fontsize=15, color=BLUE,
            ha="center", va="center")

    # 2r on wheel 3
    w3 = wheels[2]
    off = w3["radial"] * 0.22
    p = w3["center"] - w3["drive"] * 0.30 + off
    q = w3["center"] + w3["drive"] * 0.30 + off
    arrow(ax, p, q, color=BLUE, lw=1.1, style="<|-|>", ms=9)
    ax.text(*((p + q) / 2 + w3["radial"] * 0.17), r"$2r$", fontsize=14, color=BLUE,
            ha="center", va="center")

    # chassis frame
    arrow(ax, (0, 0), (0, 0.52), color=INK, lw=1.8)
    ax.text(0.06, 0.58, r"$x$（车头）", va="center", fontsize=12)
    arrow(ax, (0, 0), (-0.52, 0), color=INK, lw=1.8)
    ax.text(-0.56, 0.1, r"$y$", ha="center", va="center", fontsize=13)
    t = np.linspace(np.deg2rad(-150), np.deg2rad(110), 60)
    rr = 0.2
    ax.plot(rr * np.cos(t[:-3]), rr * np.sin(t[:-3]), color=PURPLE, lw=1.6, zorder=3)
    arrow(ax, (rr * np.cos(t[-4]), rr * np.sin(t[-4])), (rr * np.cos(t[-1]), rr * np.sin(t[-1])),
          color=PURPLE, lw=1.6, ms=11)
    ax.text(0.24, -0.22, r"$\omega$", fontsize=14, color=PURPLE, ha="center", va="center")

    ax.text(0, -1.72, r"俯视图。箭头为 $\tau_i>0$ 时轮组的推力方向，轮号与 $B_\tau$ 的列一一对应",
            ha="center", va="center", fontsize=10.5, color=MUTED)
    return fig


def fig_power_quadratic():
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    cases = [
        # A, b, P_bud, rho_pid, xlim, title, pick
        (1.0, 2.0, 8.0, 3.2, (-1.0, 4.0), r"(a) 加速时功率不够：取 $\rho^*=\rho_2$", 2),
        (1.0, -10.0, -16.0, 1.0, (-0.6, 9.4), r"(b) 已超功率、减速可回收功率：取 $\rho^*=\rho_1$", 1),
    ]
    for ax, (A, b, P_bud, rho_pid, xlim, title, pick) in zip(axes, cases):
        x = np.linspace(*xlim, 400)
        P = A * x ** 2 + b * x
        delta = b * b + 4 * A * P_bud
        r1 = (-b - np.sqrt(delta)) / (2 * A)
        r2 = (-b + np.sqrt(delta)) / (2 * A)
        lo, hi = max(0.0, r1), r2

        ax.axvspan(xlim[0], 0, color=FILL_GRAY, zorder=0)
        ax.axhline(0, color=LINE, lw=1, zorder=1)
        ax.axvline(0, color=LINE, lw=1, zorder=1)
        ax.plot(x, P, color=INK, lw=2, zorder=2)
        m = (x >= lo) & (x <= hi)
        ax.plot(x[m], P[m], color=GREEN, lw=3.5, zorder=3)
        ax.axhline(P_bud, color=BLUE, lw=1.4, ls="--", zorder=1)

        ylim = (min(P.min(), P_bud) - 4, max(P.max(), 0) + 3)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.text(xlim[1] - 0.05, P_bud + (ylim[1] - ylim[0]) * 0.025, r"$P_{bud}$", color=BLUE,
                ha="right", va="bottom", fontsize=13)
        if pick == 2:
            ax.text(xlim[0] + 0.1, ylim[1] - (ylim[1] - ylim[0]) * 0.06, r"$\rho<0$" + "\n不可取",
                    color=MUTED, va="top", fontsize=10.5)
        ax.plot([lo, hi], [ylim[0] + 0.6] * 2, color=GREEN, lw=5, solid_capstyle="butt", zorder=3)
        ax.text((lo + hi) / 2, ylim[0] + 1.1, "可行区间", color=GREEN, ha="center", va="bottom",
                fontsize=10.5)

        p_pid = A * rho_pid ** 2 + b * rho_pid
        rho_star = r2 if pick == 2 else r1
        p_star = A * rho_star ** 2 + b * rho_star
        ax.plot([rho_pid], [p_pid], "o", color=RED, ms=8, zorder=5)
        ax.plot([rho_star], [p_star], "o", color=GREEN, ms=9, zorder=5)
        dy = (ylim[1] - ylim[0]) * 0.07
        ax.text(rho_pid, p_pid + dy, r"$\rho_{PID}$" + "\n超预算", color=RED, ha="center",
                va="bottom", fontsize=11)
        ax.text(rho_star + (0.12 if pick == 2 else -0.2), p_star - dy * 0.4, r"$\rho^*$",
                color=GREEN, ha="left" if pick == 2 else "right", va="top", fontsize=13)
        arrow(ax, (rho_pid, p_pid), (rho_star, p_star), color=MUTED, lw=1.2, ms=12,
              cs="arc3,rad=0.35" if pick == 2 else "arc3,rad=-0.35", z=4)

        ax.set_title(title, fontsize=13, pad=10)
        ax.set_xlabel(r"$\rho$（沿 $\vec{u}_a$ 的加速度大小）")
        ax.set_ylabel(r"$\hat{P}_c(\rho)=A\rho^2+b\rho$")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    fig.tight_layout(w_pad=3)
    return fig


def _toy_power(v0, s):
    # Illustrative P_n over (v0, R*omega): spinning costs more than translating.
    return 18 + 12 * v0 + 20 * s + 14 * v0 ** 2 + 30 * s ** 2


def fig_gradient_descent():
    P_exp, P_safe = 160.0, 10.0
    level = P_exp - P_safe
    start = np.array([2.6, 2.2])

    path = [start.copy()]
    v = start.copy()
    for _ in range(40):
        P = _toy_power(*v)
        if P <= level:
            break
        g = np.array([12 + 28 * v[0], 20 + 60 * v[1]])
        u = g / np.linalg.norm(g)
        dv = (P - level) / P_exp * 0.35
        v = np.clip(v - dv * u, 0, start)
        path.append(v.copy())
    path = np.array(path)

    # proportional scaling for comparison
    lam_lo, lam_hi = 0.0, 1.0
    for _ in range(60):
        lam = (lam_lo + lam_hi) / 2
        if _toy_power(*(lam * start)) > level:
            lam_hi = lam
        else:
            lam_lo = lam
    scaled = lam_lo * start

    fig, ax = plt.subplots(figsize=(7.4, 6))
    xs, ys = np.meshgrid(np.linspace(0, 3, 300), np.linspace(0, 2.6, 300))
    Z = _toy_power(xs, ys)
    cs = ax.contour(xs, ys, Z, levels=[50, 100, 200, 250, 300, 350], colors=LINE, linewidths=1)
    ax.clabel(cs, fmt="%d W", fontsize=9, colors=MUTED)
    ax.contourf(xs, ys, Z, levels=[0, level], colors=[FILL_BLUE])
    ax.contour(xs, ys, Z, levels=[level], colors=[BLUE], linewidths=2.2)

    ax.plot([0, start[0]], [0, start[1]], color=MUTED, lw=1.2, ls="--")
    ax.plot(*scaled, "o", mfc="white", mec=MUTED, mew=1.8, ms=9, zorder=5)
    ax.annotate("等比例缩放", scaled, xytext=(-12, 10), textcoords="offset points",
                color=MUTED, fontsize=11, ha="right")

    ax.plot(path[:, 0], path[:, 1], color=ORANGE, lw=2, zorder=4)
    ax.plot(path[1:-1, 0], path[1:-1, 1], "o", color=ORANGE, ms=4, zorder=4)
    ax.plot(*start, "o", color=RED, ms=9, zorder=5)
    ax.annotate(r"$v_{exp}$", start, xytext=(10, -18), textcoords="offset points", color=RED, fontsize=14)
    ax.plot(*path[-1], "o", color=GREEN, ms=10, zorder=5)
    ax.annotate(r"$v_{tar}$（梯度下降）", path[-1], xytext=(10, -20), textcoords="offset points",
                color=GREEN, fontsize=12)

    ax.text(0.12, 0.2, r"$\hat{P}_n \leq P_{exp}-P_{safe}$", color=BLUE, fontsize=13)
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 2.6)
    ax.set_xlabel(r"平动速度 $v_0$ (m/s)")
    ax.set_ylabel(r"自旋线速度 $R\omega$ (m/s)")
    ax.set_title("沿功率梯度优先削减更耗功率的自旋分量（示意数据）", fontsize=12, pad=10)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout()
    return fig


def fig_airborne_fsm():
    fig, ax = canvas(13, 5.4, (0, 16.2), (0, 6.7))
    states = [
        (0.4, "正常", FILL_GREEN, GREEN, [
            r"$v_{mea}=B_v\,\omega_{mea}$",
            "前馈 + 功率规划",
            "RLS 在线辨识",
        ]),
        (5.9, "滞空", FILL_ORANGE, ORANGE, [
            r"$v_{mea}$：IMU 外推",
            r"$\omega_{tar}=J_v\,v_{est}$",
            r"前馈、$a_{tar}$ 规划关闭",
            "PID 积分、底盘 RLS 冻结",
        ]),
        (11.4, "落地", FILL_BLUE, BLUE, [
            r"$\omega_{tar}=J_v\,v_{est}$ 平滑过渡",
            "降低力矩上限后逐步恢复",
            "积分、底盘 RLS 继续冻结",
        ]),
    ]
    w, y0, h = 4.4, 1.6, 3.2
    for x, name, fc, ec, items in states:
        box(ax, x, y0, w, h, fc, ec, lw=1.6, r=0.25)
        ax.text(x + w / 2, y0 + h - 0.5, name, ha="center", va="center", fontsize=16, weight="bold")
        for j, item in enumerate(items):
            ax.text(x + 0.35, y0 + h - 1.2 - j * 0.52, "· " + item, va="center", fontsize=11.5)

    arrow(ax, (4.85, 3.9), (5.85, 3.9), lw=1.8)
    ax.text(5.35, 5.35, r"$|f_z|<0.3g$ 持续数 ms" + "\n" + r"或 ≥2 个轮组 $\hat{F}_{t,i}\approx 0$",
            ha="center", va="bottom", fontsize=11, color=MUTED, linespacing=1.6)
    ax.plot([5.35, 5.35], [5.3, 4.0], color=LINE, lw=1)
    arrow(ax, (10.35, 3.9), (11.35, 3.9), lw=1.8)
    ax.text(10.85, 5.35, r"$f_z$ 尖峰 $>2g$", ha="center", va="bottom", fontsize=11, color=MUTED)
    ax.plot([10.85, 10.85], [5.3, 4.0], color=LINE, lw=1)

    arrow(ax, (13.6, 1.55), (2.6, 1.55), lw=1.8, cs="arc3,rad=-0.18")
    ax.text(8.1, 0.25, r"约 50~100 ms 后，$r_1$、$r_2$ 回落，切回轮速估计",
            ha="center", va="center", fontsize=11, color=MUTED)
    return fig


# ---------------------------------------------------------------- banner / index
# Mechanical line art in the style of the RoboMaster rule manual cover, drawn on a
# dark background. Units are mm; the top view keeps the X layout of fig_geometry
# (forward = +y, wheel 1 front-right, numbering clockwise).

LA_BG = "#0e1319"
LA_INK = "#d3dbe4"
LA_DIM = "#8794a3"
LA_ACCENT = "#1ea7e1"

WHEEL_R = 76
WHEEL_HW = 28
CHASSIS_R = 300


def _rot(pts, ang):
    c, s = np.cos(ang), np.sin(ang)
    return np.asarray(pts, float) @ np.array([[c, s], [-s, c]])


def _circle(c, r, n=72):
    t = np.linspace(0, 2 * np.pi, n + 1)
    return np.column_stack([c[0] + r * np.cos(t), c[1] + r * np.sin(t)])


def _rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _rounded(pts, r, n=8):
    """Closed polyline of a polygon with filleted corners."""
    pts = np.asarray(pts, float)
    out = []
    for i in range(len(pts)):
        p, a, b = pts[i], pts[i - 1], pts[(i + 1) % len(pts)]
        u1 = (a - p) / np.linalg.norm(a - p)
        u2 = (b - p) / np.linalg.norm(b - p)
        half = np.arccos(np.clip(u1 @ u2, -1, 1)) / 2
        rr = min(r, 0.45 * min(np.linalg.norm(a - p), np.linalg.norm(b - p)) * np.tan(half))
        bis = (u1 + u2) / np.linalg.norm(u1 + u2)
        c = p + bis * rr / np.sin(half)
        t1, t2 = p + u1 * rr / np.tan(half), p + u2 * rr / np.tan(half)
        a1 = np.arctan2(t1[1] - c[1], t1[0] - c[0])
        a2 = np.arctan2(t2[1] - c[1], t2[0] - c[0])
        d = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
        for k in np.linspace(0, 1, n):
            out.append(c + rr * np.array([np.cos(a1 + d * k), np.sin(a1 + d * k)]))
    out.append(out[0])
    return np.array(out)


def _inset(pts, d):
    """Offset a convex counter-clockwise polygon inward by d."""
    pts = np.asarray(pts, float)
    lines = []
    for i in range(len(pts)):
        p, q = pts[i], pts[(i + 1) % len(pts)]
        e = (q - p) / np.linalg.norm(q - p)
        lines.append((p + np.array([-e[1], e[0]]) * d, e))
    out = []
    for i in range(len(pts)):
        (p1, e1), (p2, e2) = lines[i - 1], lines[i]
        s = np.linalg.solve(np.column_stack([e1, -e2]), p2 - p1)
        out.append(p1 + e1 * s[0])
    return np.array(out)


class _Pen:
    """Painter's-order line art: shapes are filled with the background to hide what is below."""

    def __init__(self, ax, lw):
        self.ax, self.lw = ax, lw

    def line(self, pts, lw=1.0, color=LA_INK, alpha=0.85, ls="-", z=3):
        pts = np.asarray(pts, float)
        self.ax.plot(pts[:, 0], pts[:, 1], color=color, lw=self.lw * lw, alpha=alpha, ls=ls,
                     zorder=z, solid_capstyle="round", solid_joinstyle="round")

    def shape(self, pts, lw=1.0, color=LA_INK, alpha=0.85, z=3):
        pts = np.asarray(pts, float)
        self.ax.fill(pts[:, 0], pts[:, 1], color=LA_BG, lw=0, zorder=z)
        self.line(pts, lw, color, alpha, z=z)

    def hole(self, c, r, alpha=0.6, lw=0.6):
        self.shape(_circle(c, r, 20), lw=lw, alpha=alpha)

    def text(self, xy, s, size=1.0, color=LA_DIM, **kw):
        self.ax.text(*xy, s, color=color, fontsize=self.lw * 11 * size, zorder=6, **kw)

    def note(self, anchor, at, lines, ha="left"):
        anchor, at = np.asarray(anchor, float), np.asarray(at, float)
        elbow = at + np.array([-14 if ha == "left" else 14, 0])
        self.line([anchor, elbow, at + np.array([0, 0])], lw=0.6, color=LA_DIM, alpha=0.8, z=5)
        self.ax.plot(*anchor, "o", ms=self.lw * 3, color=LA_INK, zorder=5)
        dx = 6 if ha == "left" else -6
        self.text(at + (dx, 8), lines[0], size=1.05, color=LA_INK, ha=ha, va="bottom", weight="bold")
        for j, s in enumerate(lines[1:]):
            self.text(at + (dx, -6 - j * 19), s, size=0.85, ha=ha, va="top")

    def dim(self, p0, p1, label, normal, off, color=LA_DIM):
        p0, p1, n = np.asarray(p0, float), np.asarray(p1, float), np.asarray(normal, float)
        q0, q1 = p0 + n * off, p1 + n * off
        if off:
            self.line([p0 + n * 4, q0 + n * 6], lw=0.5, color=color, alpha=0.8, z=5)
            self.line([p1 + n * 4, q1 + n * 6], lw=0.5, color=color, alpha=0.8, z=5)
        self.ax.add_patch(FancyArrowPatch(q0, q1, arrowstyle="<|-|>", mutation_scale=self.lw * 7,
                                          color=color, lw=self.lw * 0.6, zorder=5,
                                          shrinkA=0, shrinkB=0))
        e = q1 - q0
        ang = np.degrees(np.arctan2(e[1], e[0]))
        if ang > 90 or ang < -90:
            ang += 180
        self.text((q0 + q1) / 2 + n * 9, label, color=color, ha="center", va="center",
                  rotation=ang, rotation_mode="anchor",
                  bbox=dict(fc=LA_BG, ec="none", pad=1.5))


def _shock(pen, W, p0, p1):
    """Coil-over shock absorber in a local frame mapped by W."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    L = np.linalg.norm(p1 - p0)
    e = (p1 - p0) / L
    n = np.array([-e[1], e[0]])

    def seg(a, b, w):
        return [p0 + e * a + n * w, p0 + e * b + n * w, p0 + e * b - n * w, p0 + e * a - n * w]

    pen.shape(W(_rounded(seg(8, L * 0.58, 7), 3)), lw=0.9)
    pen.shape(W(seg(L * 0.58, L - 8, 2.6)), lw=0.7)
    for a in (14, L - 18):
        pen.shape(W(_rounded(seg(a, a + 4, 12.5), 1.5)), lw=0.8)
    t = np.linspace(0, 2 * np.pi, 40)
    for a in np.linspace(24, L - 26, 9):
        along = a + 2.2 * np.cos(t) + 3.2 * np.sin(t)
        across = 11.5 * np.sin(t)
        pen.line(W([p0 + e * s + n * w for s, w in zip(along, across)]), lw=0.7, alpha=0.75)
    for a in (0, L):
        pen.shape(W(_circle(p0 + e * a, 7.5, 28)), lw=0.9)
        pen.hole(W([p0 + e * a])[0], 3)


def _wheel_module(pen, W):
    """Omni wheel, M3508 motor, suspension arm and shock, in the wheel frame (s: rolling, t: axle out)."""
    pen.shape(W(_rounded(_rect(22, 42, -158, -30), 9)), lw=1.0)
    for t0 in (-147, -41):
        pen.hole(W([(32, t0)])[0], 4.5, alpha=0.75)
    _shock(pen, W, (-46, -182), (-40, -44))

    pen.shape(W(_rounded(_rect(-21, 21, -110, -70), 4)), lw=1.0)
    for t in np.arange(-105, -72, 4.5):
        pen.line(W([(-21, t), (21, t)]), lw=0.45, alpha=0.45)
    pen.shape(W(_rounded(_rect(-14, 14, -117, -110), 2)), lw=0.8)
    pen.shape(W(_rounded(_rect(-18, 18, -70, -42), 3)), lw=0.9)
    for t in (-61, -51):
        pen.line(W([(-18, t), (18, t)]), lw=0.5, alpha=0.55)
    pen.shape(W(_rounded(_rect(-15, 15, -42, -28), 2)), lw=0.8)

    pen.shape(W(_rect(-60, 60, -4.5, 4.5)), lw=0.8)
    pen.shape(W(_rounded(_rect(-24, 24, WHEEL_HW, WHEEL_HW + 6), 2)), lw=0.8)
    rollers = []
    for (t0, t1), offset in (((-WHEEL_HW, -5), 0.0), ((5, WHEEL_HW), 18.0)):
        for phi in np.deg2rad(np.arange(0, 360, 36) + offset):
            if np.cos(phi) > 0.08:
                rollers.append((np.cos(phi), np.sin(phi), t0, t1))
    for cphi, sphi, t0, t1 in sorted(rollers):
        sc = 67 * sphi
        hl = (34 * cphi + 18 * abs(sphi) * 0.6) / 2
        pen.shape(W(_rounded(_rect(sc - hl, sc + hl, t0 + 0.8, t1 - 0.8), 9)), lw=0.85,
                  alpha=0.55 + 0.35 * cphi)
        if cphi > 0.5:
            pen.line(W([(sc - hl + 5, (t0 + t1) / 2), (sc + hl - 5, (t0 + t1) / 2)]), lw=0.4, alpha=0.35)


def _top_view(pen, numbers=True):
    a = 210
    k = (CHASSIS_R - WHEEL_HW - 80) * np.sqrt(2) - a
    plate = np.array([(a, k), (k, a), (-k, a), (-a, k), (-a, -k), (-k, -a), (k, -a), (a, -k)])
    pen.shape(_rounded(plate, 10), lw=1.3, z=2)
    pen.line(_rounded(_inset(plate, 7), 6), lw=0.5, alpha=0.45, z=2)

    trap = np.array([(-48, 118), (48, 118), (30, 168), (-30, 168)])
    tri = np.array([(100, -30), (140, 0), (100, 30)])
    for q in range(4):
        pen.shape(_rounded(_rot(trap, q * np.pi / 2), 8), lw=0.8, alpha=0.7)
        pen.shape(_rounded(_rot(tri, np.pi / 4 + q * np.pi / 2), 6), lw=0.8, alpha=0.7)
    inner = _inset(plate, 15)
    for i in range(8):
        p, q = inner[i], inner[(i + 1) % 8]
        for f in np.linspace(0.12, 0.88, max(2, int(np.linalg.norm(q - p) // 45))):
            pen.hole(p + (q - p) * f, 2.6)

    for x0, x1, y0, y1 in ((-152, -112, -62, 62), (112, 152, -52, 52)):
        pen.shape(_rounded(_rect(x0, x1, y0, y1), 4), lw=0.8, alpha=0.75)
        for cx, cy in ((x0 + 6, y0 + 6), (x1 - 6, y0 + 6), (x1 - 6, y1 - 6), (x0 + 6, y1 - 6)):
            pen.hole((cx, cy), 2, alpha=0.5, lw=0.5)
        for yy in np.arange(y0 + 16, y1 - 12, 7):
            pen.shape(_rect(x0 + 13, x1 - 13, yy, yy + 3.5), lw=0.4, alpha=0.45)

    for r, lw, al in ((104, 1.0, 0.8), (96, 0.5, 0.5), (74, 0.8, 0.7)):
        pen.line(_circle((0, 0), r, 140), lw=lw, alpha=al)
    for ang in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        pen.hole((85 * np.cos(ang), 85 * np.sin(ang)), 2.6, alpha=0.7)

    for q in range(4):
        A = lambda pts, q=q: _rot(pts, q * np.pi / 2)
        for sx in (-42, 42):
            pen.shape(A(_rounded([(sx - 8, a - 8), (sx + 8, a - 8), (sx + 5, 229), (sx - 5, 229)], 2)),
                      lw=0.7, alpha=0.75)
        pen.shape(A(_rounded([(-70, 229), (70, 229), (64, 245), (-64, 245)], 3)), lw=1.1)
        pen.line(A(_rounded([(-48, 233), (48, 233), (44, 241), (-44, 241)], 2)), lw=0.5, alpha=0.5)
        for sx in (-1, 1):
            pen.shape(A(_rounded(_rect(sx * 71 - 3, sx * 71 + 3, 224, 243), 2)), lw=0.9,
                      color=LA_ACCENT, alpha=0.9)

    for i, theta in enumerate(np.deg2rad([45, -45, -135, 135]), start=1):
        v = np.array([np.cos(theta), np.sin(theta)])
        u = np.array([-np.sin(theta), np.cos(theta)])
        c = CHASSIS_R * v
        W = lambda pts, c=c, u=u, v=v: c + np.asarray(pts, float)[:, :1] * u + np.asarray(pts, float)[:, 1:] * v
        _wheel_module(pen, W)
        if numbers:
            tag = c + u * (WHEEL_R + 30) + v * 6
            pen.shape(_circle(tag, 12, 40), lw=0.9, color=LA_ACCENT, alpha=0.95, z=5)
            pen.text(tag, str(i), color=LA_ACCENT, ha="center", va="center", weight="bold")

    _gimbal(pen)


def _gimbal(pen):
    pen.shape(_circle((0, 0), 42, 80), lw=1.0)
    pen.line(_circle((0, 0), 32, 80), lw=0.5, alpha=0.5)
    pen.shape(_rounded(_rect(-58, 58, -172, -62), 14), lw=1.1)
    pen.line(_circle((0, -117), 34, 80), lw=0.7, alpha=0.7)
    pen.line([(-50, -75), (50, -75)], lw=0.5, alpha=0.5)
    pen.shape(_rect(-8, 8, -62, 84), lw=0.7)
    for sx in (-1, 1):
        pen.shape(_rounded(_rect(sx * 54 - 6, sx * 54 + 6, -58, 72), 4), lw=0.9)
    pen.shape(_rounded(_rect(-102, -66, -30, 30), 5), lw=1.0)
    for x in np.arange(-98, -70, 4):
        pen.line([(x, -30), (x, 30)], lw=0.4, alpha=0.45)
    pen.shape(_rounded(_rect(-48, 48, 50, 168), 10), lw=1.0)
    for sx in (-1, 1):
        pen.shape(_circle((sx * 30, 112), 25, 80), lw=1.0)
        pen.line(_circle((sx * 30, 112), 10, 40), lw=0.6, alpha=0.7)
        for ang in np.linspace(0, 2 * np.pi, 3, endpoint=False):
            pen.hole((sx * 30 + 17 * np.cos(ang), 112 + 17 * np.sin(ang)), 1.8, alpha=0.6)
    pen.shape(_rounded(_rect(-9, 9, 138, 318), 3), lw=1.0)
    pen.shape(_rounded(_rect(-12.5, 12.5, 304, 332), 3), lw=1.0)
    for y in (150, 290):
        pen.line([(-9, y), (9, y)], lw=0.5, alpha=0.6)
    pen.shape(_rounded(_rect(24, 50, 150, 196), 4), lw=0.9)
    pen.line(_circle((37, 186), 6, 30), lw=0.6, alpha=0.8)


def _side_wheel(pen, c, k=1.0):
    c = np.asarray(c, float)
    S = lambda pts: c + np.asarray(pts, float) * k

    def roller(phi, alpha):
        ctr = 67 * np.array([np.cos(phi), np.sin(phi)])
        td = np.array([-np.sin(phi), np.cos(phi)])
        rd = np.array([np.cos(phi), np.sin(phi)])
        pts = [ctr + td * a + rd * b for a, b in ((-16, -9), (16, -9), (16, 9), (-16, 9))]
        pen.shape(S(_rounded(pts, 8.6)), lw=0.9, alpha=alpha)
        pen.line(S([ctr - td * 10, ctr + td * 10]), lw=0.4, alpha=alpha * 0.4)
        for sgn in (-1, 1):
            pen.hole(S([ctr + td * sgn * 17.5])[0], 1.6 * k, alpha=alpha, lw=0.5)

    for j in range(10):
        roller(np.deg2rad(18 + 36 * j), 0.5)
    pen.shape(S(_circle((0, 0), 57, 140)), lw=0.9)
    for j in range(10):
        roller(np.deg2rad(36 * j), 0.9)
    for j in range(10):
        ang = np.deg2rad(36 * j + 18)
        tab = _rot(_rounded(_rect(47, 60, -4, 4), 2), -ang)
        pen.shape(S(tab), lw=0.6, alpha=0.7)
    pen.line(S(_circle((0, 0), 52, 120)), lw=0.5, alpha=0.5)
    pen.line(S(_circle((0, 0), 26, 80)), lw=1.0)
    pen.line(S(_circle((0, 0), 9, 40)), lw=0.8)
    for j in range(6):
        ang = np.deg2rad(60 * j)
        pen.hole(S([(17 * np.cos(ang), 17 * np.sin(ang))])[0], 2.2 * k)
        ang += np.deg2rad(30)
        pen.shape(S(_circle((39 * np.cos(ang), 39 * np.sin(ang)), 6.5, 30)), lw=0.7, alpha=0.75)
    for d in ((1, 0), (0, 1)):
        d = np.array(d)
        pen.line([c - d * 100 * k, c + d * 100 * k], lw=0.5, color=LA_DIM, alpha=0.6,
                 ls=(0, (12, 3, 2, 3)))


def _la_canvas(width_px, height_px, center, px_per_mm, lw):
    fig = plt.figure(figsize=(width_px / 100, height_px / 100), dpi=100, facecolor=LA_BG)
    ax = fig.add_axes([0, 0, 1, 1])
    hw, hh = width_px / px_per_mm / 2, height_px / px_per_mm / 2
    ax.set_xlim(center[0] - hw, center[0] + hw)
    ax.set_ylim(center[1] - hh, center[1] + hh)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_facecolor(LA_BG)
    for g in np.arange(-2000, 2001, 20):
        major = g % 100 == 0
        kw = dict(color="white", lw=0.6 if major else 0.4, alpha=0.075 if major else 0.035, zorder=0)
        ax.axvline(g, **kw)
        ax.axhline(g, **kw)
    return fig, ax, _Pen(ax, lw)


def _centerlines(pen, ext):
    dash = (0, (14, 4, 2, 4))
    for ang in (0, 90, 45, -45):
        d = np.array([np.cos(np.deg2rad(ang)), np.sin(np.deg2rad(ang))])
        e = ext if ang in (0, 90) else CHASSIS_R + 70
        pen.line([-d * e, d * e], lw=0.5, color=LA_DIM, alpha=0.55, ls=dash, z=1)


def _save_avif(fig, path, quality):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=100, facecolor=LA_BG)
    plt.close(fig)
    buf.seek(0)
    Image.open(buf).convert("RGB").save(path, "AVIF", quality=quality)


def fig_banner():
    fig, ax, pen = _la_canvas(2560, 1120, center=(-440, 45), px_per_mm=1.3, lw=1.0)
    _centerlines(pen, 470)
    _top_view(pen)
    w1 = CHASSIS_R * np.array([np.cos(np.pi / 4), np.sin(np.pi / 4)])
    pen.dim((0, 0), w1, "R 300", normal=(-np.sqrt(0.5), np.sqrt(0.5)), off=0, color=LA_ACCENT)
    pen.dim((-245, -330), (245, -330), "490", normal=(0, -1), off=0)
    pen.line([(-245, -248), (-245, -336)], lw=0.5, color=LA_DIM, alpha=0.8)
    pen.line([(245, -248), (245, -336)], lw=0.5, color=LA_DIM, alpha=0.8)

    pen.note((268, 175), (345, 330), ["OMNI WHEEL ×4", "Ø152 mm · double row",
                                      "X-type 45° layout"])
    pen.note((148, -148), (345, -120), ["M3508 · P19", r"$\tau_{wheel}=N\,\tau_{motor}$",
                                        "power feedback"])
    pen.note((-165, -104), (-330, -330), ["SUSPENSION", "coil-over shock absorber"], ha="right")
    pen.note((-73.5, 73.5), (-330, 330), ["YAW BEARING", "slip ring · 12 × M3"], ha="right")

    for y in (215, -165):
        _side_wheel(pen, (-1330, y), k=1.5)
    pen.dim((-1330 - 114, 215 + 114), (-1330 + 114, 215 + 114), "Ø152", normal=(0, 1), off=22)
    pen.note((-1330 + 80, -165 + 75), (-1060, -40), ["OMNI WHEEL · SIDE", "10 rollers per row",
                                                     "rows offset 18°"])
    _save_avif(fig, os.path.join(OUT, "banner.avif"), quality=82)


def fig_index():
    fig, ax, pen = _la_canvas(960, 560, center=(0, 40), px_per_mm=0.86, lw=1.25)
    _centerlines(pen, 420)
    _top_view(pen, numbers=False)
    _side_wheel(pen, (-520, -230), k=1.2)
    _side_wheel(pen, (520, 300), k=1.2)
    _save_avif(fig, os.path.join(OUT, "index.avif"), quality=82)


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, make in (("framework", fig_framework),
                       ("geometry", fig_geometry),
                       ("power-quadratic", fig_power_quadratic),
                       ("gradient-descent", fig_gradient_descent),
                       ("airborne-fsm", fig_airborne_fsm)):
        fig = make()
        fig.savefig(os.path.join(OUT, f"{name}.svg"), facecolor="white", bbox_inches="tight",
                    pad_inches=0.15)
        plt.close(fig)
    fig_banner()
    fig_index()
    print(f"figures written to {OUT}")


if __name__ == "__main__":
    main()
