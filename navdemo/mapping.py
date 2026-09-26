"""Building the map as we go: a simulated lidar and a grid that fills in.

Instead of a map made from the real geometry up front (Grid.from_world),
the robot starts knowing nothing. Two layers, as in a real mapping system:

    the map       what the sensor has said about each cell, and nothing
    (obstacle,    else. Each scan: every cell a ray passes through becomes
     known)       known free, and the cell each hit lands in becomes
                  occupied. This is the only layer that is ever updated.

    the planning  derived from the map by growing its occupied cells by
    map (occ)     `inflate` (inflate_cells, as in run_grid.py), and simply
                  made again whenever the map changes. An inflated map
                  can't be updated in place -- once cells are merged, you
                  can't tell which obstacle made which one occupied, so
                  nothing could ever be removed again. Regenerating from
                  the map keeps that possible (it takes about a
                  millisecond at these sizes).

Cells no ray has reached stay unknown, and the planners treat them as
free: the optimistic "free space assumption". It is what lets the robot
plan at all before it has seen everything -- and what makes it head
confidently into a dead end it hasn't seen yet, and replan when it does.

Occupied cells are never cleared from the map yet. A real mapper would let
rays clear cells again, or keep a probability per cell (log-odds), so that
noise and moving objects can be forgotten -- a natural next step, and the
two-layer structure is what makes it possible.
"""

import numpy as np

from .grid import Grid
from .rasterize import inflate_cells


class Lidar:
    """A 2D range sensor: `n_rays` rays evenly spread over 360 degrees.

    Rays are cast by sphere tracing on the world's distance field: from
    the current point along the ray, the nearest obstacle is
    world.distance() away, so the ray can safely jump that far and look
    again. It converges onto the first surface the ray meets, and it
    works for any mix of circles and polygons without per-shape
    intersection code.
    """

    HIT_EPS = 1e-3     # closer than this to a surface counts as a hit
    MAX_STEPS = 100

    def __init__(self, max_range=3.0, n_rays=180, noise=0.0, rng=None):
        self.max_range = max_range
        self.n_rays = n_rays
        self.noise = noise
        self.rng = np.random.default_rng() if rng is None else rng

    def scan(self, world, x, y, a):
        """Returns (angles, ranges, hit): world-frame ray angles, measured
        ranges, and whether each ray hit something within range."""
        ang = a + np.linspace(-np.pi, np.pi, self.n_rays, endpoint=False)
        d = np.column_stack([np.cos(ang), np.sin(ang)])
        o = np.array([x, y])
        t = np.zeros(self.n_rays)
        hit = np.zeros(self.n_rays, dtype=bool)
        active = np.ones(self.n_rays, dtype=bool)
        for _ in range(self.MAX_STEPS):
            if not active.any():
                break
            dist = world.distance(o + t[active, None] * d[active])
            idx = np.flatnonzero(active)
            h = dist < self.HIT_EPS
            hit[idx[h]] = True
            t[idx] += dist
            active[idx[h]] = False
            active &= t < self.max_range
        hit &= t < self.max_range
        t = np.minimum(t, self.max_range)
        if self.noise > 0:
            t = np.where(hit, np.maximum(t + self.rng.normal(0, self.noise, len(t)), 0.0), t)
        return ang, t, hit


class MappedGrid(Grid):
    """A grid that starts unknown and is filled in scan by scan.

    `obstacle` and `known` are the map; `occ`, what the planners use, is
    made from them (see the module docstring)."""

    def __init__(self, bounds, resolution, inflate):
        super().__init__(bounds, resolution, inflate)
        self.known[:] = False

    def integrate(self, x, y, angles, ranges, hit):
        """Add one scan taken from (x, y). Returns True if any cell of the
        map became occupied that wasn't before (the trigger for checking
        the plan)."""
        ends = np.column_stack([x + ranges * np.cos(angles), y + ranges * np.sin(angles)])

        # Free space: every cell a ray passed through before its end.
        for (ex, ey), h in zip(ends, hit):
            cells = self.cells_on_segment(x, y, ex, ey)
            for ix, iy in (cells[:-1] if h else cells):
                if self.inside(ix, iy):
                    self.known[ix, iy] = True

        # Obstacles: the cell each hit lands in.
        c = np.floor((ends[hit] - self.origin) / self.res).astype(int)
        c = c[(c[:, 0] >= 0) & (c[:, 0] < self.nx) & (c[:, 1] >= 0) & (c[:, 1] < self.ny)]
        new = ~self.obstacle[c[:, 0], c[:, 1]]
        if not new.any():
            return False
        self.obstacle[c[:, 0], c[:, 1]] = True
        self.known[c[:, 0], c[:, 1]] = True
        self.update_planning_map()
        return True

    def update_planning_map(self):
        """Make the planning map again from the map."""
        self.occ = self.obstacle.copy()
        if self.inflate > 0:
            self.occ |= inflate_cells(self.obstacle, self.res, self.inflate)
