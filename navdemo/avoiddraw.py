"""Drawing for run_avoid.py: the world with what the map doesn't know, the
people, the scan, and each method's own picture -- the forces of the
potential field, DWA's candidate arcs, VFH's polar histogram."""

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle as CirclePatch, Patch, Polygon as PolygonPatch

from .avoid import GOAL_MODES
from .avoidstate import AvoidState, TUNABLES
from .world import Circle

C_UNMAPPED = (0.75, 0.45, 0.15)
C_PERSON = (0.95, 0.55, 0.1)
C_ATT, C_REP, C_SUM = "tab:green", "tab:red", "k"


class UnmappedArtist:
    """Obstacles that are real but not in the map: filled brown, dashed."""

    def __init__(self, ax):
        self.ax = ax
        self.patches = []

    def set_world(self, obstacles):
        for p in self.patches:
            p.remove()
        self.patches = []
        style = dict(facecolor=(*C_UNMAPPED, 0.35), edgecolor=C_UNMAPPED, lw=1.5,
                     ls="--", zorder=5)
        for ob in obstacles:
            patch = CirclePatch(ob.c, ob.r, **style) if isinstance(ob, Circle) \
                else PolygonPatch(ob.v, closed=True, **style)
            self.patches.append(self.ax.add_patch(patch))

    def set_visible(self, v):
        for p in self.patches:
            p.set_visible(v)


class PeopleArtist:
    """The people, and faintly the paths they walk."""

    def __init__(self, ax):
        self.ax = ax
        self.circles, self.paths = [], []

    def set_world(self, movers):
        for h in self.circles + self.paths:
            h.remove()
        self.circles = [self.ax.add_patch(CirclePatch(m.c, m.r, facecolor=(*C_PERSON, 0.8),
                                                      edgecolor="k", lw=1, zorder=8))
                        for m in movers]
        self.paths = [self.ax.plot(m.path.xy[:, 0], m.path.xy[:, 1], ":", color=C_PERSON,
                                   lw=1, zorder=3)[0] for m in movers]

    def update(self, movers, visible=True):
        for c, m in zip(self.circles, movers):
            c.center = tuple(m.c)
            c.set_visible(visible)
        for p in self.paths:
            p.set_visible(visible)


C_LOCAL = (0.45, 0.2, 0.6)        # the local map's cells, by certainty


class LocalMapArtist:
    """The local map: its cells shaded by certainty, and its window."""

    def __init__(self, ax):
        self.ax = ax
        self.im = None
        (self.box,) = ax.plot([], [], color=C_LOCAL, lw=1, ls="--", zorder=4)

    def set(self, local, visible):
        if not visible or local.i0 is None:
            if self.im is not None:
                self.im.set_visible(False)
            self.box.set_data([], [])
            return
        img = np.zeros((local.n, local.n, 4))
        img[..., :3] = C_LOCAL
        img[..., 3] = 0.75 * local.c
        img = img.transpose(1, 0, 2)
        if self.im is None:
            self.im = self.ax.imshow(img, origin="lower", extent=local.extent,
                                     interpolation="nearest", zorder=4)
        else:
            self.im.set_data(img)
            self.im.set_extent(local.extent)
        self.im.set_visible(True)
        x0, x1, y0, y1 = local.extent
        self.box.set_data([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0])


class FovArtist:
    """The edges of the sensor's field of view, when it isn't all round."""

    def __init__(self, ax):
        (self.lines,) = ax.plot([], [], color="r", lw=0.8, alpha=0.6, zorder=7)

    def set(self, pose, fov, rng):
        if fov >= 2 * np.pi - 1e-9:
            self.lines.set_data([], [])
            return
        x, y, a = pose
        l, r = a + fov / 2, a - fov / 2
        self.lines.set_data([x + rng * np.cos(l), x, x + rng * np.cos(r)],
                            [y + rng * np.sin(l), y, y + rng * np.sin(r)])


class MethodArtist:
    """What the method is looking at."""

    def __init__(self, ax):
        self.ax = ax
        (self.att,) = ax.plot([], [], color=C_ATT, lw=2.5, zorder=9)
        (self.rep,) = ax.plot([], [], color=C_REP, lw=2.5, zorder=9)
        (self.sum,) = ax.plot([], [], color=C_SUM, lw=2.5, zorder=9)
        self.arcs = LineCollection([], linewidths=0.8, zorder=6)
        ax.add_collection(self.arcs)
        (self.best,) = ax.plot([], [], color="tab:green", lw=2.5, zorder=7)
        self.hist = LineCollection([], linewidths=1.5, zorder=6)
        ax.add_collection(self.hist)
        (self.choice,) = ax.plot([], [], color="tab:blue", lw=2.5, zorder=9)
        (self.goal_dir,) = ax.plot([], [], "--", color="0.3", lw=1, zorder=9)

    def clear(self):
        for h in (self.att, self.rep, self.sum, self.best, self.choice, self.goal_dir):
            h.set_data([], [])
        self.arcs.set_segments([])
        self.hist.set_segments([])

    def set(self, sim, method, visible):
        self.clear()
        v = sim.viz
        if not visible or not v:
            return
        x, y = sim.robot.x, sim.robot.y
        if method == "potential field" and "f" in v:
            L = 0.8       # m per unit of force, capped so a big push stays on screen
            for line, f in ((self.att, v["f_att"]), (self.rep, v["f_rep"]), (self.sum, v["f"])):
                n = np.hypot(*f)
                f = f * min(1.0, 2.0 / max(n, 1e-9))
                line.set_data([x, x + L * f[0]], [y, y + L * f[1]])
        elif method == "DWA" and "arcs" in v:
            self.arcs.set_segments(v["arcs"])
            self.arcs.set_colors([(0.4, 0.4, 0.4, 0.5) if ok else (0.9, 0.2, 0.2, 0.35)
                                  for ok in v["ok"]])
            if v["best"] is not None:
                b = v["arcs"][v["best"]]
                self.best.set_data(b[:, 0], b[:, 1])
        elif method == "VFH" and "hist" in v:
            h, free = v["hist"], v["free"]
            n = len(h)
            ang = -np.pi + (np.arange(n) + 0.5) * 2 * np.pi / n
            r0 = 0.3
            r1 = r0 + 0.12 * np.minimum(h, 8.0)
            segs = [[(x + r0 * np.cos(a), y + r0 * np.sin(a)),
                     (x + max(r, r0 + 0.02) * np.cos(a), y + max(r, r0 + 0.02) * np.sin(a))]
                    for a, r in zip(ang, r1)]
            self.hist.set_segments(segs)
            self.hist.set_colors([(0.2, 0.6, 0.3, 0.7) if f else (0.85, 0.2, 0.2, 0.8)
                                  for f in free])
            g = v["goal_dir"]
            self.goal_dir.set_data([x, x + 1.2 * np.cos(g)], [y, y + 1.2 * np.sin(g)])
            if v["choice"] is not None:
                c = v["choice"]
                self.choice.set_data([x, x + 1.0 * np.cos(c)], [y, y + 1.0 * np.sin(c)])


class Legend:
    """Lists only what is on screen, as in the planning demo."""

    def __init__(self, fig):
        self.fig = fig
        self.legend = None
        self.labels = None

    @staticmethod
    def entries(state: AvoidState, sim):
        e = []
        if state.show_geometry:
            e.append(Patch(facecolor=(0, 0, 0, 0.10), edgecolor="k", label="obstacle (in the map)"))
            if sim.world.unmapped:
                e.append(Patch(facecolor=(*C_UNMAPPED, 0.35), edgecolor=C_UNMAPPED, ls="--",
                               label="obstacle NOT in the map"))
        if sim.world.movers and state.people_on:
            e.append(Patch(facecolor=(*C_PERSON, 0.8), edgecolor="k", label="person"))
        if state.show_scan:
            e.append(Line2D([], [], color="r", marker=".", lw=0, ms=4, label="lidar hit"))
        if state.local_map:
            e.append(Patch(facecolor=(*C_LOCAL, 0.6), edgecolor=C_LOCAL, ls="--",
                           label="local map (darker = more certain)"))
        if state.goal_mode == GOAL_MODES[1] and sim.plan is not None:
            if state.show_map:
                e.append(Patch(facecolor=(0.55, 0.60, 0.80, 0.45), label="map (A*'s view)"))
            e.append(Line2D([], [], color="C0", lw=2, label="A* path"))
            e.append(Line2D([], [], color="C0", marker="o", lw=0, ms=7, label="carrot (local goal)"))
        if state.show_method and sim.viz:
            if state.method == "potential field":
                e += [Line2D([], [], color=C_ATT, lw=2.5, label="pull of the goal"),
                      Line2D([], [], color=C_REP, lw=2.5, label="push of the hits"),
                      Line2D([], [], color=C_SUM, lw=2.5, label="sum: where it steers")]
            elif state.method == "DWA":
                e += [Line2D([], [], color=(0.4, 0.4, 0.4), lw=1, label="arc: admissible"),
                      Line2D([], [], color=(0.9, 0.2, 0.2), lw=1, label="arc: can't stop in time"),
                      Line2D([], [], color="tab:green", lw=2.5, label="arc: best score")]
            else:
                e += [Line2D([], [], color=(0.2, 0.6, 0.3), lw=2.5, label="histogram: free"),
                      Line2D([], [], color=(0.85, 0.2, 0.2), lw=2.5, label="histogram: blocked"),
                      Line2D([], [], color="0.3", ls="--", lw=1, label="direction to goal"),
                      Line2D([], [], color="tab:blue", lw=2.5, label="direction chosen")]
        if state.show_trail and len(sim.trail) > 1:
            e.append(Line2D([], [], color="r", lw=1, label="driven"))
        return e

    def update(self, state, sim):
        handles = self.entries(state, sim)
        labels = tuple(h.get_label() for h in handles)
        if labels == self.labels:
            return
        if self.legend is not None:
            self.legend.remove()
        self.legend = self.fig.legend(handles=handles, loc="lower left",
                                      bbox_to_anchor=(0.01, 0.02), fontsize=7,
                                      frameon=False, ncol=1) if handles else None
        self.labels = labels


class Panel:
    def __init__(self, fig):
        self.text = fig.text(0.015, 0.97, "", family="monospace", fontsize=8.5,
                             va="top", ha="left")

    def update(self, state: AvoidState, sim, status):
        rows = [f"method:  {state.method} (c)",
                f"goal:    {state.goal_mode} (m)",
                (f"people:  {'wait for the robot' if state.polite else 'walk blindly'} (w), on (p)"
                 if state.people_on else "people:  off (p)"),
                (f"local:   on (l), {'hits + clearing' if state.map_clear else 'hits only'} (u), "
                 f"forget {'on' if state.map_forget else 'off'} (f)" if state.local_map
                 else "local:   off -- latest scan only (l)"),
                "", "          VALUE", "          -----"]
        vis = state.visible()
        for i, t in enumerate(TUNABLES):
            if i not in vis:
                continue
            txt = t.format(state.value(t.name))
            cell = ("[%s]" if i == state.cursor else " %s ") % txt.center(8)
            rows.append(f"{t.label:>9} {cell}")
        clear = sim.min_clear
        rows += ["", f"world:  {sim.world.name}",
                 f"status: {status}",
                 f"t = {sim.t:5.1f} s   v = {sim.robot.v:+.2f} m/s",
                 f"closest so far: {clear:.2f} m" if np.isfinite(clear) else "",
                 "", "press 'h' for keys"]
        self.text.set_text("\n".join(rows))
