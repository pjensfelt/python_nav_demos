"""Drawing for run_planning.py: the real obstacles and the grid on top of
each other, the search, the plans, the sensor and the robot.

The real obstacles are drawn as outlines over the grid cells, so you can
see directly where the grid's picture of the world differs from the world.
"""

import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle as CirclePatch, Patch, Polygon as PolygonPatch, Rectangle

from .planners import field_potential, path_length
from .planstate import PlanState, TUNABLES
from .world import Circle

# RGBA per cell type
C_UNKNOWN = (0.93, 0.89, 0.78, 0.85)
C_INFLATE = (0.55, 0.60, 0.80, 0.45)
C_OBSTACLE = (0.25, 0.28, 0.45, 0.85)
C_COST = (0.85, 0.25, 0.15)        # A*'s cost near obstacles, as a tint on free cells


def setup_axes(fig, title):
    ax = fig.add_axes([0.30, 0.08, 0.68, 0.86])
    ax.set_aspect("equal")
    ax.set_title(title)
    fig.text(0.99, 0.01, "P. Jensfelt, KTH 2026", ha="right", va="bottom",
             fontsize=7, color="0.6")
    return ax


def set_world_limits(ax, world, margin=0.3, grow=False):
    """Fit the view to the world. With `grow`, only ever widen it (so it
    doesn't jump about while the world is rotated), and settle back to the
    world as drawn when it is unrotated again."""
    xmin, xmax, ymin, ymax = world.bounds
    lo = np.array([xmin - margin, ymin - margin])
    hi = np.array([xmax + margin, ymax + margin])
    if grow:
        (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
        lo, hi = np.minimum(lo, [x0, y0]), np.maximum(hi, [x1, y1])
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])


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
        self.patches.append(self.ax.add_patch(PolygonPatch(
            world.room, closed=True, fill=False, edgecolor="k", lw=1.5, zorder=5)))

    def set_visible(self, v):
        for p in self.patches:
            p.set_visible(v)

    @property
    def artists(self):
        return self.patches


class GridArtist:
    """The occupancy grid as an image, plus cell lines when cells are big
    enough to see."""

    MAX_LINES = 150  # draw cell borders only up to this many cells per side (0.1 m cells: ~100)

    def __init__(self, ax):
        self.ax = ax
        self.im = None
        self.lines = LineCollection([], colors=[(0, 0, 0, 0.12)], linewidths=0.5, zorder=2)
        ax.add_collection(self.lines)

    def set(self, grid, visible=True, faded=False, cost=None):
        img = np.zeros((grid.nx, grid.ny, 4))
        if cost is not None:
            img[..., :3] = C_COST
            img[..., 3] = 0.6 * cost
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

    def set(self, result, grid, k, visible=True, lw=0.7):
        self.tree.set_segments([])
        self.tree.set_linewidth(lw)
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


class PotentialArtist:
    """The potential field the planner descends, as a shading (low = dark,
    high = bright), drawn with the search (y) -- so the valleys it gets
    stuck in can be seen."""

    SPACING = 0.05     # m between the points it is evaluated at

    def __init__(self, ax):
        self.ax = ax
        self.im = None
        self.key = None

    def set(self, world, samples, goal, cfg, visible):
        if not visible or samples is None:
            if self.im is not None:
                self.im.set_visible(False)
            return
        key = (id(samples), tuple(goal), cfg["k_rep"], cfg["d0"], cfg["inflate"], cfg["spacing"])
        if key != self.key:
            self.key = key
            x0, x1, y0, y1 = world.bounds
            xs = np.arange(x0, x1 + 1e-9, self.SPACING)
            ys = np.arange(y0, y1 + 1e-9, self.SPACING)
            X, Y = np.meshgrid(xs, ys)
            u = field_potential(np.column_stack([X.ravel(), Y.ravel()]), np.asarray(goal),
                                samples, cfg["k_rep"], cfg["d0"], cfg["inflate"], cfg["spacing"])
            # capped just above the largest pull towards the goal: the
            # repulsion shoots up at the walls and would otherwise leave
            # everything else one colour
            U = u.reshape(X.shape)
            dg = np.hypot(X - goal[0], Y - goal[1])
            U = np.minimum(U, 1.15 * dg.max())
            ext = (x0 - self.SPACING / 2, x1 + self.SPACING / 2,
                   y0 - self.SPACING / 2, y1 + self.SPACING / 2)
            if self.im is None:
                self.im = self.ax.imshow(U, origin="lower", extent=ext, cmap="plasma",
                                         alpha=0.45, interpolation="bilinear", zorder=2)
            else:
                self.im.set_data(U)
                self.im.set_extent(ext)
            self.im.set_clim(U.min(), U.max())
        self.im.set_visible(True)


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
        key = (world, inflate)                # the world object itself, not its id
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


class Legend:
    """The legend, listing only what is on screen right now: the entries
    come and go with the display toggles, the planner and the mode."""

    def __init__(self, fig):
        self.fig = fig
        self.legend = None
        self.labels = None

    @staticmethod
    def entries(state: PlanState, mission):
        r = mission.result
        a_star = state.planner == "A*"
        e = []
        if state.show_geometry:
            e.append(Patch(facecolor=(0, 0, 0, 0.10), edgecolor="k", label="real obstacle"))
        if not state.mapped and state.show_samples:
            e.append(Line2D([], [], color="tab:orange", marker=".", lw=0,
                            label="sample point (known map)"))
        if state.show_grid:
            e.append(Patch(facecolor=C_OBSTACLE, label="map: occupied cell"))
            e.append(Patch(facecolor=C_INFLATE, label="planning map: inflation"))
            if state.mapped:
                e.append(Patch(facecolor=C_UNKNOWN, label="unknown (planned as free)"))
            if a_star and mission.cost is not None:
                e.append(Patch(facecolor=(*C_COST, 0.4), label="A*: cost near obstacles"))
        if state.show_search and r is not None:
            if a_star and r.expanded:
                e.append(Patch(facecolor=(0.9, 0.6, 0.2, 0.5), label="A* expanded"))
            elif state.planner == "potential field" and r.nodes is not None:
                e.append(Line2D([], [], color=(0.1, 0.55, 0.3), lw=1, label="field descent"))
                e.append(Patch(facecolor=(0.95, 0.75, 0.2, 0.6), label="potential (dark = low)"))
            elif not a_star and r.nodes is not None:
                e.append(Line2D([], [], color=(0.1, 0.55, 0.3), lw=1, label="RRT tree"))
        if mission.path:
            e.append(Line2D([], [], color="C0", lw=2.5,
                            label=f"plan, {mission.path_mode}" if mission.shortened else "plan"))
            if mission.shortened:
                e.append(Line2D([], [], color="C0", lw=1, ls=":", label="plan as planned"))
        if mission.old_paths:
            e.append(Line2D([], [], color="0.4", lw=1, ls="--", label="earlier plans"))
        if state.uses_exact_geometry and state.show_geometry:
            e.append(Line2D([], [], color="C3", lw=1.2, ls="--",
                            label="grown obstacle (exact checks)"))
        if state.execute:
            if state.show_trail and len(mission.trail) > 1:
                e.append(Line2D([], [], color="r", lw=1, label="driven"))
            if state.value("loc_xy") > 0 or state.value("loc_th") > 0:
                e.append(Line2D([], [], color="tab:purple", lw=1.5, ls="--",
                                label="pose estimate (loc. jitter)"))
        return e

    def update(self, state, mission):
        handles = self.entries(state, mission)
        labels = tuple(h.get_label() for h in handles)
        if labels == self.labels:
            return                      # unchanged: don't rebuild every frame
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

    def update(self, state: PlanState, mission, status):
        g = mission.grid
        rows = [f"map:     {'mapped as we go (lidar)' if state.mapped else 'known, from samples'} (m)",
                f"world:   rotated {np.rad2deg(state.world_angle):+.0f}° (, . k l; 0 back)",
                f"planner: {state.planner} (p)"]
        if state.planner == "A*":
            rows.append(f"         {'8' if state.eight else '4'}-connected (n)")
        rows.append(f"checks:  {state.checks_text()} (x)")
        if state.planner != "A*":
            if state.planner == "RRT":
                rows.append(f"         {'stop at 1st path' if state.stop_at_goal else 'all iterations'} (f)")
        if state.execute:
            rows.append(f"execute: on (e), law: {state.law} (c)")
            if state.law == "pure pursuit":
                rows.append(f"         turn in place: {'on' if state.turn_in_place else 'off'} (b)")
        else:
            rows.append("execute: off (e)")
        rows += ["", "          VALUE", "          -----"]
        vis = state.visible()
        for i, t in enumerate(TUNABLES):
            if i not in vis:
                continue
            txt = t.format(state.value(t.name))
            cell = ("[%s]" if i == state.cursor else " %s ") % txt.center(8)
            rows.append(f"{t.label:>9} {cell}")

        rows += ["", f"world:  {mission.world.name}"
                 + (f", variant {state.variant} (v)" if state.value("imperfect") > 0 else ""),
                 f"grid:   {g.nx} x {g.ny} cells, {100 * g.occ.mean():.0f}% occupied"]
        r = mission.result
        if r is not None:
            # nothing searched at all when the start or goal is occupied
            what = (f"{len(r.expanded)} cells expanded" if r.expanded
                    else f"{r.iterations} steps" if (r.nodes is not None
                                                      and state.planner == "potential field")
                    else f"{len(r.nodes)} nodes, {r.iterations} iter" if r.nodes is not None
                    else r.message)
            plen = f"{r.cost:.2f} m" if r.path else "none"
            rows += [f"plan:   {plen}, {1000 * r.seconds:.0f} ms", f"        {what}"]
            if mission.shortened:
                k = mission.curvature
                rows.append(f"        {mission.path_mode} (s): {path_length(mission.path):.2f} m")
                if k:
                    rows.append(f"        max curvature {k:.1f} 1/m (r {1 / k:.2f} m)")
                if mission.path_note:
                    rows.append("        " + mission.path_note)
            elif r.path:
                rows.append("        smoothing: off (s)")
        elif state.path_mode != "as planned":
            rows.append(f"smooth: {state.path_mode} (s)")
        rows += [f"status: {status}"]
        if state.execute:
            f = mission.follower
            stops = f"  stops = {f.stops}" if (f is not None and state.law == "stop and turn") else ""
            rows += [f"t = {mission.t:5.1f} s   driven = {mission.driven:5.1f} m",
                     f"v = {mission.robot.v:+.2f} m/s  replans = {mission.replans}" + stops]
        rows += ["", "press 'h' for keys"]
        self.text.set_text("\n".join(rows))

    @property
    def artists(self):
        return [self.text]
