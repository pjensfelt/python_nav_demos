"""Drawing helpers.

Same window layout as the localization demos: the text panel on the left,
the plot on the right, and a small plot in the bottom-left corner -- here
the strip charts of v, w and the cross-track error that were figure 3 in
the MATLAB demo. Artists are created once and only their data is updated
each frame.
"""

import numpy as np

from .params import DemoState, TUNABLES
from .sim import Follower

XLIM = YLIM = (-3.0, 3.0)   # the MATLAB demo's axis(3*[-1 1 -1 1])
ROBOT_RADIUS = 0.2          # display_robot.m


def setup_axes(fig, title):
    ax = fig.add_axes([0.30, 0.08, 0.68, 0.86])
    ax.set_xlim(*XLIM)
    ax.set_ylim(*YLIM)
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.grid(True, alpha=0.2)
    fig.text(0.99, 0.01, "P. Jensfelt, KTH 2026", ha="right", va="bottom",
             fontsize=7, color="0.6")
    return ax


def arc(x, y, a, kappa, length, n=60):
    """Points along the arc of constant curvature kappa starting at pose
    (x, y, a) -- where the robot would go if it held its current command."""
    s = np.linspace(0.0, length, n)
    if abs(kappa) < 1e-6:
        return x + s * np.cos(a), y + s * np.sin(a)
    return (x + (np.sin(a + kappa * s) - np.sin(a)) / kappa,
            y - (np.cos(a + kappa * s) - np.cos(a)) / kappa)


class RobotArtist:
    """A circle with a heading line, as display_robot.m drew it."""

    def __init__(self, ax, color="k"):
        (self.body,) = ax.plot([], [], color=color, lw=2, zorder=6)
        (self.head,) = ax.plot([], [], color=color, lw=2, zorder=6)
        (self.center,) = ax.plot([], [], "x", color=color, ms=5, zorder=6)
        self._circle = np.linspace(0, 2 * np.pi, 73)

    def set_pose(self, x, y, a):
        R = ROBOT_RADIUS
        self.body.set_data(x + R * np.cos(self._circle), y + R * np.sin(self._circle))
        self.head.set_data(x + np.cos(a) * np.array([R, R + 0.3]),
                           y + np.sin(a) * np.array([R, R + 0.3]))
        self.center.set_data([x], [y])

    @property
    def artists(self):
        return [self.body, self.head, self.center]


class GeometryArtist:
    """What the controller is looking at: the lookahead circle, the closest
    point on the path, the lookahead (target) point, and the arc the robot
    would drive if it kept its current command."""

    def __init__(self, ax):
        (self.circle,) = ax.plot([], [], color="r", lw=1, zorder=4)
        (self.to_target,) = ax.plot([], [], ":", color="r", lw=1, zorder=4)
        (self.target,) = ax.plot([], [], "o", color="r", ms=7, mfc="none", mew=1.5, zorder=7)
        (self.closest,) = ax.plot([], [], ".", color="k", ms=7, zorder=7)
        (self.arc,) = ax.plot([], [], "--", color="g", lw=1.5, zorder=4)
        self._circ = np.linspace(0, 2 * np.pi, 121)

    def set(self, f: Follower, lookahead, visible=True):
        for h in self.artists:
            h.set_visible(visible)
        if not visible:
            return
        x, y, a = f.robot.pose
        self.circle.set_data(x + lookahead * np.cos(self._circ),
                             y + lookahead * np.sin(self._circ))
        self.closest.set_data([f.closest_pt[0]], [f.closest_pt[1]])
        if f.done:
            self.target.set_data([], [])
            self.to_target.set_data([], [])
            self.arc.set_data([], [])
            return
        tx, ty = f.target
        self.target.set_data([tx], [ty])
        self.to_target.set_data([x, tx], [y, ty])
        if f.vRef > 1e-3:
            kappa = f.wRef / f.vRef
            length = 1.5 * max(np.hypot(tx - x, ty - y), 0.1)
            if abs(kappa) > 1e-6:
                length = min(length, 2 * np.pi / abs(kappa))
            self.arc.set_data(*arc(x, y, a, kappa, length))
        else:
            self.arc.set_data([], [])  # turning on the spot: no arc to show

    @property
    def artists(self):
        return [self.circle, self.to_target, self.target, self.closest, self.arc]


class StripCharts:
    """v, w and cross-track error against time (figure 3 of the MATLAB demo)."""

    def __init__(self, fig, left=0.045, width=0.215, bottom=0.05, height=0.09, gap=0.035):
        specs = [("v [m/s]", "C0"), ("w [deg/s]", "C1"), ("e [m]", "C3")]
        self.axes, self.lines = [], []
        for i, (label, color) in enumerate(specs):
            y = bottom + (len(specs) - 1 - i) * (height + gap)
            ax = fig.add_axes([left, y, width, height])
            ax.tick_params(labelsize=6, pad=1)
            ax.set_ylabel(label, fontsize=7, labelpad=1)
            ax.grid(True, alpha=0.3)
            ax.axhline(0, color="0.6", lw=0.6)
            if i < len(specs) - 1:
                ax.tick_params(labelbottom=False)
            (line,) = ax.plot([], [], color=color, lw=1)
            self.axes.append(ax)
            self.lines.append(line)
        self.axes[-1].set_xlabel("t [s]", fontsize=7, labelpad=1)

    def set(self, f: Follower):
        t = np.asarray(f.log_t)
        data = [np.asarray(f.log_v), np.rad2deg(np.asarray(f.log_w)), np.asarray(f.log_e)]
        tmax = max(5.0, t[-1] if len(t) else 0.0)
        for ax, line, d in zip(self.axes, self.lines, data):
            line.set_data(t, d)
            ax.set_xlim(0.0, tmax)
            lim = max(np.max(np.abs(d)) if len(d) else 0.0, 1e-3) * 1.15
            ax.set_ylim(-lim, lim)

    @property
    def artists(self):
        return self.lines


class Panel:
    """Left-hand text panel: the tunables and the status."""

    def __init__(self, fig):
        self.text = fig.text(0.015, 0.97, "", family="monospace", fontsize=9,
                             va="top", ha="left")

    def update(self, state: DemoState, f: Follower):
        rows = ["        VALUE", "        -----"]
        for i, t in enumerate(TUNABLES):
            txt = t.format(state.value(t.name))
            cell = ("[%s]" if i == state.cursor else " %s ") % txt.center(9)
            note = "" if t.law in (None, state.law) else "  (unused)"
            rows.append(f"{t.label:>7} {cell}{note}")

        if f.done:
            status = f"GOAL at t={f.t_done:.2f}s"
        else:
            status = "paused" if state.paused else "running"
        rows += [
            "",
            f"law:    {state.law}",
            f"turn in place: {'on' if state.turnInPlace else 'off'} (b)"
            + ("" if state.law == "pure pursuit" else " (unused)"),
            f"path:   {f.path.name} ({f.path.length:.2f} m)",
            f"status: {status}",
            "",
            f"t     = {f.t:6.2f} s",
            f"v     = {f.robot.v:+6.2f} m/s",
            f"w     = {np.rad2deg(f.robot.w):+6.1f} deg/s",
            f"e     = {f.e:+6.3f} m",
            f"max|e|= {f.max_abs_e:6.3f} m",
            "",
            "press 'h' for keys",
        ]
        self.text.set_text("\n".join(rows))

    @property
    def artists(self):
        return [self.text]
