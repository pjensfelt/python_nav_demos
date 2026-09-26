"""Occupancy grid made from the real world, and the collision checkers the
planners use.

Making the grid is where the world stops looking like itself: every cell
is either free or occupied. A cell is occupied if any part of it overlaps
an obstacle grown by the inflation radius, tested exactly (see
rasterize.py, shared with run_grid.py). Two choices matter, and both are
live parameters in run_planning.py:

    resolution  the cell size. Coarse cells close narrow passages.

    inflation   the obstacles are grown by this radius before gridding,
                so the planners can treat the robot as a point. Set it
                below the robot radius and the plan grazes obstacles that
                the real robot then hits.

Grid indexing is occ[ix, iy], x first, as in the WASP assignment 3 GridMap.
"""

from math import ceil, floor, hypot

import numpy as np

class Grid:
    """An occupancy grid over the world's bounds.

    `occ` is what the planners use: obstacles grown by `inflate`.
    `obstacle` is just the obstacles themselves -- drawn in a darker shade
    so you can see which occupied cells are only there because of the
    inflation. `known` is all True here; see MappedGrid for a grid that
    starts out unknown.
    """

    def __init__(self, bounds, resolution, inflate):
        self.bounds = bounds
        self.res = float(resolution)
        self.inflate = float(inflate)
        xmin, xmax, ymin, ymax = bounds
        self.origin = np.array([xmin, ymin])
        self.nx = int(ceil((xmax - xmin) / self.res - 1e-9))
        self.ny = int(ceil((ymax - ymin) / self.res - 1e-9))
        self.occ = np.zeros((self.nx, self.ny), dtype=bool)
        self.obstacle = np.zeros((self.nx, self.ny), dtype=bool)
        self.known = np.ones((self.nx, self.ny), dtype=bool)

    def centres(self):
        ix, iy = np.meshgrid(np.arange(self.nx), np.arange(self.ny), indexing="ij")
        return self.origin + (np.column_stack([ix.ravel(), iy.ravel()]) + 0.5) * self.res

    @classmethod
    def from_world(cls, world, resolution, inflate):
        """The pre-built map, from the real geometry: a cell is occupied if
        it overlaps an obstacle (or the room's walls) grown by `inflate`."""
        from .rasterize import rasterize, room_walls     # (rasterize imports Grid)
        # one ring of cells round the room, so its walls show up as cells too
        x0, x1, y0, y1 = world.bounds
        r_ = resolution
        bounds = (x0 - r_, x1 + r_, y0 - r_, y1 + r_)
        r = rasterize(world.obstacles + room_walls(world.room), bounds, resolution, inflate)
        g = cls(bounds, resolution, inflate)
        g.obstacle, g.occ = r.obstacle, r.occ
        return g

    # ---- cells and coordinates ------------------------------------------

    def cell(self, x, y):
        return (floor((x - self.origin[0]) / self.res), floor((y - self.origin[1]) / self.res))

    def coord(self, ix, iy):
        return (self.origin[0] + (ix + 0.5) * self.res, self.origin[1] + (iy + 0.5) * self.res)

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def free(self, ix, iy):
        return self.inside(ix, iy) and not self.occ[ix, iy]

    @property
    def extent(self):
        """For imshow."""
        return (self.origin[0], self.origin[0] + self.nx * self.res,
                self.origin[1], self.origin[1] + self.ny * self.res)

    # ---- line traversal -------------------------------------------------

    def cells_on_segment(self, x1, y1, x2, y2):
        """Every cell the segment passes through, in order.

        The grid traversal of Amanatides & Woo (1987), as in the WASP
        GridMap: walk from cell to cell, each time stepping in whichever
        of x or y reaches its next cell border first.
        """
        cx, cy = self.cell(x1, y1)
        gx, gy = self.cell(x2, y2)
        cells = [(cx, cy)]
        dx, dy = x2 - x1, y2 - y1
        L = hypot(dx, dy)
        if (cx, cy) == (gx, gy) or L == 0:
            return cells
        dx, dy = dx / L, dy / L
        sx, sy = int(np.sign(dx)), int(np.sign(dy))
        # distance along the ray to the first x / y border, and between borders
        bx = self.origin[0] + (cx + (sx > 0)) * self.res
        by = self.origin[1] + (cy + (sy > 0)) * self.res
        tx = (bx - x1) / dx if sx else np.inf
        ty = (by - y1) / dy if sy else np.inf
        dtx = self.res / abs(dx) if sx else np.inf
        dty = self.res / abs(dy) if sy else np.inf
        n = abs(gx - cx) + abs(gy - cy)
        for _ in range(n):
            if tx <= ty:
                cx += sx
                tx += dtx
            else:
                cy += sy
                ty += dty
            cells.append((cx, cy))
        return cells


class GridChecker:
    """Collision checks against the (inflated) grid: what the planners use.

    Cells in `ignore` count as free. When replanning from where the robot
    is, its own cell may have become occupied -- it's inside the inflation
    of an obstacle it just saw -- and the planner must still be able to
    leave it.
    """

    def __init__(self, grid, ignore=()):
        self.grid = grid
        self.ignore = set(ignore)

    def free(self, ix, iy):
        return (ix, iy) in self.ignore or self.grid.free(ix, iy)

    def point_free(self, p):
        return self.free(*self.grid.cell(*p))

    def segment_free(self, p, q):
        return all(self.free(ix, iy) for ix, iy in self.grid.cells_on_segment(*p, *q))


class GeometryChecker:
    """Collision checks against the real geometry, grown by `inflate`.

    Only the sampling planners (RRT, RRT*) can use this -- they only ever
    ask "is this point / this straight line free?", which the geometry can
    answer exactly. A* needs cells to search over, so it can't.
    """

    def __init__(self, world, inflate):
        self.world = world
        self.inflate = inflate

    def point_free(self, p):
        return self.world.distance(np.asarray(p, float))[0] > self.inflate

    def segment_free(self, p, q):
        return self.world.segment_distance(p, q) > self.inflate
