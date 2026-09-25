"""Drawing for run_planning.py: the real obstacles and the grid on top of
each other, the search, the plans, the sensor and the robot.

The real obstacles are drawn as outlines over the grid cells, so you can
see directly where the grid's picture of the world differs from the world.
"""

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle as CirclePatch, Patch, Polygon as PolygonPatch, Rectangle

from .planstate import PlanState, TUNABLES
from .world import Circle

# RGBA per cell type
C_UNKNOWN = (0.93, 0.89, 0.78, 0.85)
C_INFLATE = (0.55, 0.60, 0.80, 0.45)
C_OBSTACLE = (0.25, 0.28, 0.45, 0.85)


def setup_axes(fig, title):
    ax = fig.add_axes([0.30, 0.08, 0.68, 0.86])
    ax.set_aspect("equal")
    ax.set_title(title)
    fig.text(0.99, 0.01, "P. Jensfelt, KTH 2026", ha="right", va="bottom",
             fontsize=7, color="0.6")
    return ax


def set_world_limits(ax, world, margin=0.3):
    xmin, xmax, ymin, ymax = world.bounds
    ax.set_xlim(xmin - margin, xmax + margin)
    ax.set_ylim(ymin - margin, ymax + margin)


class ObstacleArtist:
    """The real geometry: outlines with a faint fill."""

    def __init__(self, ax):
        self.ax = ax
        self.patches = []

    def set_world(self, world):
        for p in self.patches:
            p.remove()
        self.patches = []
        style = dict(facecolor=(0, 0, 0, 0.10), edgecolor="k", lw=1.5, zorder=5)
        for ob in world.obstacles:
            if isinstance(ob, Circle):
                patch = CirclePatch(ob.c, ob.r, **style)
            else:
                patch = PolygonPatch(ob.v, closed=True, **style)
            self.patches.append(self.ax.add_patch(patch))
        xmin, xmax, ymin, ymax = world.bounds
        self.patches.append(self.ax.add_patch(Rectangle(
            (xmin, ymin), xmax - xmin, ymax - ymin, fill=False, edgecolor="k", lw=1.5, zorder=5)))

    def set_visible(self, v):
        for p in self.patches:
            p.set_visible(v)

    @property
    def artists(self):
        return self.patches


class GridArtist:
    """The occupancy grid as an image, plus cell lines when cells are big
    enough to see."""

    MAX_LINES = 60   # draw cell borders only up to this many cells per side

    def __init__(self, ax):
        self.ax = ax
        self.im = None
        self.lines = LineCollection([], colors=[(0, 0, 0, 0.12)], linewidths=0.5, zorder=2)
        ax.add_collection(self.lines)

    def set(self, grid, visible=True, faded=False):
        img = np.zeros((grid.nx, grid.ny, 4))
        img[~grid.known] = C_UNKNOWN
        img[grid.occ] = C_INFLATE
        img[grid.obstacle] = C_OBSTACLE
        if faded:        # the planner isn't using the grid right now
            img[..., 3] *= 0.3
        img = img.transpose(1, 0, 2)   # imshow wants [row = y, column = x]
        if self.im is None or self.im.get_array().shape != img.shape:
            if self.im is not None:
                self.im.remove()
            self.im = self.ax.imshow(img, origin="lower", extent=grid.extent,
                                     interpolation="nearest", zorder=1)
            segs = []
            if max(grid.nx, grid.ny) <= self.MAX_LINES:
                x0, x1, y0, y1 = grid.extent
                segs += [[(x, y0), (x, y1)] for x in np.linspace(x0, x1, grid.nx + 1)]
                segs += [[(x0, y), (x1, y)] for y in np.linspace(y0, y1, grid.ny + 1)]
            self.lines.set_segments(segs)
        else:
            self.im.set_data(img)
            self.im.set_extent(grid.extent)
        self.im.set_visible(visible)
        self.lines.set_visible(visible)

    @property
    def artists(self):
        return ([self.im] if self.im is not None else []) + [self.lines]


class SearchArtist:
    """What the planner did: the cells A* expanded (coloured by order), or
    the RRT/RRT* tree -- replayed up to event `k`."""

    def __init__(self, ax):
        self.ax = ax
        self.im = None
        self.tree = LineCollection([], colors=[(0.1, 0.55, 0.3, 0.6)], linewidths=0.7, zorder=3)
        ax.add_collection(self.tree)

    def total(self, result):
        if result is None:
            return 0
        return len(result.expanded) if result.expanded else len(result.events)

    def set(self, result, grid, k, visible=True):
        self.tree.set_segments([])
        if self.im is not None:
            self.im.set_visible(False)
        if result is None or not visible:
            return
        if result.expanded:
            order = np.full((grid.nx, grid.ny), np.nan)
            cells = np.array(result.expanded[:k])
            if len(cells):
                order[cells[:, 0], cells[:, 1]] = np.arange(len(cells))
            img = np.ma.masked_invalid(order.T)
            if self.im is None or self.im.get_array().shape != img.shape:
                if self.im is not None:
                    self.im.remove()
                self.im = self.ax.imshow(img, origin="lower", extent=grid.extent, cmap="plasma",
                                         alpha=0.35, interpolation="nearest", zorder=2)
            self.im.set_data(img)
            self.im.set_extent(grid.extent)
            self.im.set_clim(0, max(len(result.expanded), 1))
            self.im.set_visible(True)
        elif result.nodes is not None:
            # Replay parent assignments up to event k: later events for the
            # same child are RRT* rewires and replace its earlier edge.
            parent = {}
            for child, par in result.events[:k]:
                parent[child] = par
            n = result.nodes
            self.tree.set_segments([[n[c], n[p]] for c, p in parent.items()])

    @property
    def artists(self):
        return ([self.im] if self.im is not None else []) + [self.tree]


class CSpaceArtist:
    """The real obstacles grown by the inflation radius -- what RRT/RRT*
    avoid when they check the exact geometry. Drawn as the level curve
    distance = inflate of the world's distance field, which is exactly the
    grown shape (rounded corners and all) without any Minkowski-sum code."""

    def __init__(self, ax, spacing=0.03):
        self.ax = ax
        self.spacing = spacing
        self.cs = None
        self.key = None

    def set(self, world, inflate, visible):
        key = (id(world), inflate)
        if visible and key != self.key:
            self.clear()
            xmin, xmax, ymin, ymax = world.bounds
            xs = np.arange(xmin, xmax + self.spacing, self.spacing)
            ys = np.arange(ymin, ymax + self.spacing, self.spacing)
            X, Y = np.meshgrid(xs, ys)
            D = world.distance(np.column_stack([X.ravel(), Y.ravel()])).reshape(X.shape)
            if inflate > 0:
                self.cs = self.ax.contour(X, Y, D, levels=[inflate], colors=["C3"],
                                          linewidths=1.2, linestyles="--", zorder=5)
            self.key = key
        if self.cs is not None:
            self.cs.set_visible(visible)

    def clear(self):
        if self.cs is not None:
            self.cs.remove()
            self.cs = None
        self.key = None


class ScanArtist:
    """The latest lidar scan: faint rays, red dots where they hit."""

    def __init__(self, ax):
        self.rays = LineCollection([], colors=[(0.9, 0.2, 0.2, 0.12)], linewidths=0.6, zorder=4)
        ax.add_collection(self.rays)
        (self.hits,) = ax.plot([], [], ".", color="r", ms=2.5, zorder=8)

    def set(self, scan):
        if scan is None:
            self.rays.set_segments([])
            self.hits.set_data([], [])
            return
        x, y, ang, rng, hit = scan
        ends = np.column_stack([x + rng * np.cos(ang), y + rng * np.sin(ang)])
        self.rays.set_segments([[(x, y), tuple(e)] for e in ends])
        self.hits.set_data(ends[hit, 0], ends[hit, 1])

    @property
    def artists(self):
        return [self.rays, self.hits]


def add_legend(fig):
    handles = [
        Patch(facecolor=(0, 0, 0, 0.10), edgecolor="k", label="real obstacle"),
        Patch(facecolor=C_OBSTACLE, label="occupied cell"),
        Patch(facecolor=C_INFLATE, label="inflation"),
        Patch(facecolor=C_UNKNOWN, label="unknown (planned as free)"),
        Line2D([], [], color=(0.1, 0.55, 0.3), lw=1, label="RRT tree"),
        Patch(facecolor=(0.9, 0.6, 0.2, 0.5), label="A* expanded"),
        Line2D([], [], color="C0", lw=2.5, label="plan"),
        Line2D([], [], color="0.4", lw=1, ls="--", label="earlier plans"),
        Line2D([], [], color="C3", lw=1.2, ls="--", label="grown obstacle (exact checks)"),
        Line2D([], [], color="r", lw=1, label="driven"),
    ]
    fig.legend(handles=handles, loc="lower left", bbox_to_anchor=(0.01, 0.02),
               fontsize=7, frameon=False, ncol=1)


class Panel:
    def __init__(self, fig):
        self.text = fig.text(0.015, 0.97, "", family="monospace", fontsize=8.5,
                             va="top", ha="left")

    def update(self, state: PlanState, mission, status):
        g = mission.grid
        rows = [f"map:     {'mapped as we go' if state.mapped else 'known'} (k)",
                f"cells:   {state.raster} (m)",
                f"planner: {state.planner} (p)"]
        if state.planner == "A*":
            rows.append(f"         {'8' if state.eight else '4'}-connected (n)")
        rows.append(f"checks:  {state.checks_text()} (x)")
        if state.planner != "A*":
            if state.planner == "RRT":
                rows.append(f"         {'stop at 1st path' if state.stop_at_goal else 'all iterations'} (f)")
        rows.append(f"shortcut: {'on' if state.shortcut else 'off'} (s)   law: {state.law} (c)")
        if state.law == "pure pursuit":
            rows.append(f"turn in place: {'on' if state.turn_in_place else 'off'} (b)")
        rows += ["", "          VALUE", "          -----"]
        vis = state.visible()
        for i, t in enumerate(TUNABLES):
            if i not in vis:
                continue
            txt = t.format(state.value(t.name))
            cell = ("[%s]" if i == state.cursor else " %s ") % txt.center(8)
            rows.append(f"{t.label:>9} {cell}")

        rows += ["", f"world:  {mission.world.name}",
                 f"grid:   {g.nx} x {g.ny} cells, {100 * g.occ.mean():.0f}% occupied"]
        r = mission.result
        if r is not None:
            what = (f"{len(r.expanded)} cells expanded" if r.expanded
                    else f"{len(r.nodes)} nodes, {r.iterations} iter")
            plen = f"{r.cost:.2f} m" if r.path else "none"
            rows += [f"plan:   {plen}, {what}", f"        {1000 * r.seconds:.0f} ms"]
        rows += [f"status: {status}",
                 f"t = {mission.t:5.1f} s   driven = {mission.driven:5.1f} m",
                 f"v = {mission.robot.v:+.2f} m/s  replans = {mission.replans}",
                 "", "press 'h' for keys"]
        self.text.set_text("\n".join(rows))

    @property
    def artists(self):
        return [self.text]
