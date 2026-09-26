"""Drawing for run_grid.py."""

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle as CirclePatch, Patch, Polygon as PolygonPatch, Rectangle

from .gridstate import GridState, TUNABLES
from .plandraw import C_INFLATE, C_OBSTACLE
from .world import Circle

C_REACH = (0.3, 0.75, 0.35, 0.18)
C_OUTSIDE = (0.6, 0.6, 0.6, 0.25)     # cells entirely outside the world
ROBOT_RADIUS = 0.2


class GridView:
    """The world and its grid: cells, cell lines, changed cells, flood fill,
    the real obstacles, and the probe points."""

    MAX_LINES = 120

    def __init__(self, ax):
        self.ax = ax
        self.cells = None
        self.overlay = None
        self.lines = LineCollection([], colors=[(0, 0, 0, 0.18)], linewidths=0.5, zorder=2)
        ax.add_collection(self.lines)
        self.patches = []
        self.probe_lines = []
        (self.samples,) = ax.plot([], [], ".", color="tab:orange", ms=2.5, zorder=7)
        self.robot = ax.add_patch(CirclePatch((0, 0), ROBOT_RADIUS, fill=False, ec="k",
                                              lw=1.2, ls=":", zorder=9))
        self._line_key = None

    def _image(self, img, grid, attr, zorder, transform):
        im = getattr(self, attr)
        if im is None or im.get_array().shape != img.shape:
            if im is not None:
                im.remove()
            im = self.ax.imshow(img, origin="lower", extent=grid.extent,
                                interpolation="nearest", zorder=zorder)
            setattr(self, attr, im)
        else:
            im.set_data(img)
            im.set_extent(grid.extent)
        im.set_transform(transform)
        return im

    def set(self, state: GridState, grid, obstacles, room, grid_to_axes, probes, reach,
            samples=None, robot_at=None):
        """Cells, cell lines and the changed-cells overlay are drawn in the
        grid's own frame, placed by `grid_to_axes` (its shift and rotation);
        everything else is in the fixed drawing frame."""
        # cells
        img = np.zeros((grid.nx, grid.ny, 4))
        img[grid.occ] = C_INFLATE
        img[grid.obstacle] = C_OBSTACLE
        img[~grid.known] = C_OUTSIDE
        self._image(img.transpose(1, 0, 2), grid, "cells", 1, grid_to_axes).set_visible(state.show_grid)

        # changed cells (vs the reference pose) and the flood fill
        over = np.zeros((grid.nx, grid.ny, 4))
        if state.show_reach and reach is not None:
            over[reach] = C_REACH
        self._image(over.transpose(1, 0, 2), grid, "overlay", 3, grid_to_axes)

        # cell lines
        key = (grid.nx, grid.ny, grid.extent)
        if key != self._line_key:
            segs = []
            if max(grid.nx, grid.ny) <= self.MAX_LINES:
                x0, x1, y0, y1 = grid.extent
                segs += [[(x, y0), (x, y1)] for x in np.linspace(x0, x1, grid.nx + 1)]
                segs += [[(x0, y), (x1, y)] for y in np.linspace(y0, y1, grid.ny + 1)]
            self.lines.set_segments(segs)
            self._line_key = key
        self.lines.set_transform(grid_to_axes)
        self.lines.set_visible(state.show_grid)

        # the real obstacles, where they are now
        for p in self.patches:
            p.remove()
        self.patches = []
        style = dict(facecolor=(0, 0, 0, 0.12), edgecolor="k", lw=1.5, zorder=5)
        for ob in obstacles:
            patch = CirclePatch(ob.c, ob.r, **style) if isinstance(ob, Circle) \
                else PolygonPatch(ob.v, closed=True, **style)
            patch.set_visible(state.show_geometry)
            self.patches.append(self.ax.add_patch(patch))
        # the sample points ("samples" rule)
        if samples is None:
            self.samples.set_data([], [])
        else:
            self.samples.set_data(samples[:, 0], samples[:, 1])

        # the room's walls: the world's boundary is a wall too
        wall = PolygonPatch(room, closed=True, fill=False, ec="k", lw=2.5, zorder=5)
        wall.set_visible(state.show_geometry)
        self.patches.append(self.ax.add_patch(wall))
        # the grid's outline (it may be rotated away from the world)
        xmin, xmax, ymin, ymax = grid.extent
        outline = Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, fill=False, ec="0.5",
                            lw=1, zorder=5)
        outline.set_transform(grid_to_axes)
        self.patches.append(self.ax.add_patch(outline))

        # probes: the two points (drawn where they are in the world), and the
        # path the planner found between them (in the grid's frame, so it
        # follows the grid when that is moved)
        while len(self.probe_lines) < len(probes):
            (pts,) = self.ax.plot([], [], "o", ms=6, zorder=8)
            (path,) = self.ax.plot([], [], "-", color="tab:green", lw=1.5, zorder=8)
            self.probe_lines.append((pts, path))
        for i, (pts, path_line) in enumerate(self.probe_lines):
            if i < len(probes):
                name, a, b, ok, path = probes[i]
                pts.set_data([a[0], b[0]], [a[1], b[1]])
                pts.set_color("tab:green" if ok else "tab:red")
                p = np.array(path) if path else np.empty((0, 2))
                path_line.set_data(p[:, 0], p[:, 1])
                path_line.set_transform(grid_to_axes)
            else:
                pts.set_data([], [])
                path_line.set_data([], [])
        if robot_at is not None:
            self.robot.center = robot_at


def add_legend(fig):
    handles = [
        Patch(facecolor=(0, 0, 0, 0.12), edgecolor="k", label="real obstacle"),
        Line2D([], [], color="tab:orange", marker=".", lw=0, label="sample point"),
        Patch(facecolor=C_OBSTACLE, label="occupied cell"),
        Patch(facecolor=C_INFLATE, label="inflation cell"),
        Line2D([], [], color="tab:green", marker="o", lw=1.5, label="probe: planned path"),
        Line2D([], [], color="tab:red", marker="o", lw=0, label="probe: no path"),
        Patch(facecolor=C_REACH, label="cells the planner expanded (f)"),
        Line2D([], [], color="k", lw=1.2, ls=":", label="robot size (r = 0.2 m)"),
    ]
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.01, 0.03),
               fontsize=7, frameon=False, labelspacing=0.4)


def _pose_line(label, pose, res):
    dx, dy = pose.offset
    return f"{label} {dx:+.2f}, {dy:+.2f} m, {np.rad2deg(pose.angle):+.0f}°"


def _ms(seconds):
    ms = 1000 * seconds
    return f"{ms:.1f} ms" if ms < 100 else f"{ms:,.0f} ms"


def _bytes(n):
    for unit, size in (("MB", 1e6), ("kB", 1e3)):
        if n >= size:
            return f"{n / size:.1f} {unit}"
    return f"{n} B"


class Panel:
    def __init__(self, fig):
        self.text = fig.text(0.015, 0.97, "", family="monospace", fontsize=8.5,
                             va="top", ha="left")

    def update(self, state: GridState, world, number, grid, probes):
        res = state.value("res")
        order = state.effective_inflate_order
        forced = state.rule == "samples" and state.inflate_order != order
        rows = [f"cells:   {state.rule} (m)"
                + (", sensor-like" if state.rule == "samples" else ", uses the model"),
                f"inflate: {order} (i)"
                + (", forced: no model" if forced else
                   ", needs a model" if order == "world first" else ", as from sensor data"),
                "", "          VALUE", "          -----"]
        vis = state.visible()
        for i, t in enumerate(TUNABLES):
            if i not in vis:
                continue
            txt = t.format(state.value(t.name))
            cell = ("[%s]" if i == state.cursor else " %s ") % txt.center(8)
            rows.append(f"{t.label:>9} {cell}")
        n = grid.nx * grid.ny
        rows += ["",
                 (f"world:  {number}: {world.name}" if number else f"world:  {world.name}")
                 + (f", variant {state.variant} (v)" if state.value("imperfect") > 0 else ""),
                 f"grid:   {grid.nx} x {grid.ny} = {n:,} cells, {_bytes(n)}"]
        rows += [
                 (f"time:   {_ms(grid.t_build)} grid + {_ms(grid.t_search)} planning"
                  if grid.t_search is not None else f"time:   {_ms(grid.t_build)} to make the grid"),
                 _pose_line("grid at: ", state.grid, res) + ("  <- w" if state.moving == "grid" else ""),
                 _pose_line("world at:", state.world, res) + ("  <- w" if state.moving == "world" else ""),
                 ("planning: on (p)" if state.planning
                  else "planning: off (p)")]
        for name, ok, why, _, length, straight in probes:
            if ok:
                rows.append(f"  {name:>10}: path {length:.2f} m"
                            f" (+{100 * (length / straight - 1):.0f}% on the line)")
            else:
                rows.append(f"  {name:>10}: blocked"
                            + ("" if why == "no way through" else f" ({why})"))
        if state.sweep_kind is not None:
            rows.append("sweeping the rotation ..." if state.sweep_kind == "r"
                        else f"sweeping along the grid's {state.sweep_kind} ...")
        rows += ["press 'h' for keys"]
        self.text.set_text("\n".join(rows))
