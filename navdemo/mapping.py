"""Building the map as we go: a simulated lidar and a grid that fills in.

Instead of a map made from the real geometry up front (Grid.from_world),
the robot starts knowing nothing. Each scan:

    - every cell a ray passes through becomes known free,
    - every cell near a ray's end point (a hit) becomes occupied -- the
      hit's own cell as the obstacle, and every cell within `inflate` of
      the hit point as inflation, exactly as for the pre-built grid.

Cells no ray has reached stay unknown, and the planners treat them as
free: the optimistic "free space assumption". It is what lets the robot
plan at all before it has seen everything -- and what makes it head
confidently into a dead end it hasn't seen yet, and replan when it does.

Occupied cells are never cleared again. A real mapper would keep a
probability per cell (log-odds) so that noise and moving objects can be
forgotten; that is a natural next step, not done here.
"""

import numpy as np

from .grid import Grid


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
    """A grid that starts unknown and is filled in scan by scan."""

    def __init__(self, bounds, resolution, inflate, mode="center"):
        super().__init__(bounds, resolution, inflate, mode)
        self.known[:] = False
        # Offsets of all cells whose centre can be within inflate + slack
        # of a point in the centre cell -- the stencil stamped around
        # every hit.
        k = int(np.ceil((self.inflate + self.slack) / self.res)) + 1
        self._offsets = np.array([(i, j) for i in range(-k, k + 1) for j in range(-k, k + 1)])

    def integrate(self, x, y, angles, ranges, hit):
        """Add one scan taken from (x, y). Returns True if any cell became
        occupied that wasn't before (the trigger for checking the plan)."""
        ends = np.column_stack([x + ranges * np.cos(angles), y + ranges * np.sin(angles)])

        # Free space: every cell a ray passed through before its end.
        for (ex, ey), h in zip(ends, hit):
            cells = self.cells_on_segment(x, y, ex, ey)
            for ix, iy in (cells[:-1] if h else cells):
                if self.inside(ix, iy):
                    self.known[ix, iy] = True

        # Obstacles: the cells around each hit point.
        before = self.occ.sum()
        pts = ends[hit]
        if len(pts):
            c = np.floor((pts - self.origin) / self.res).astype(int)
            cells = (c[:, None, :] + self._offsets[None, :, :]).reshape(-1, 2)
            pts_rep = np.repeat(pts, len(self._offsets), axis=0)
            ok = (cells[:, 0] >= 0) & (cells[:, 0] < self.nx) & (cells[:, 1] >= 0) & (cells[:, 1] < self.ny)
            cells, pts_rep = cells[ok], pts_rep[ok]
            centres = self.origin + (cells + 0.5) * self.res
            d = np.hypot(*(centres - pts_rep).T)
            own = (np.floor((pts - self.origin) / self.res).astype(int))
            own = own[(own[:, 0] >= 0) & (own[:, 0] < self.nx) & (own[:, 1] >= 0) & (own[:, 1] < self.ny)]
            self.obstacle[own[:, 0], own[:, 1]] = True
            infl = cells[d <= self.inflate + self.slack + 1e-12]
            self.occ[infl[:, 0], infl[:, 1]] = True
            self.occ[own[:, 0], own[:, 1]] = True
            self.known[self.occ] = True
        return self.occ.sum() > before
