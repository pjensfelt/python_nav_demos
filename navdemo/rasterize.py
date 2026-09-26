"""From the real world to cells, exactly -- for run_grid.py.

Two rules for when a cell counts as occupied:

    any overlap   anything of the (grown) obstacle is inside the cell. Uses
                  the geometric model. The natural rule for a binary map,
                  and the safe one: it never loses an obstacle, but it eats
                  free space.

The other doesn't use the geometry at all, only points on it -- like a map
built from a range sensor, which only ever returns points on surfaces:

    samples       points spaced evenly along every obstacle's outline (and
                  the room's walls), scattered by 2D Gaussian noise; a cell
                  is occupied if at least `min_hits` points land in it.
                  Obstacles come out hollow (a sensor never sees inside
                  one), too sparse samples leave holes in walls, and noise
                  thickens walls and puts stray points in free space.

"Any overlap" is tested exactly here -- the cell square itself against the
obstacle -- unlike the quick "conservative" rule in grid.py, which adds half
the cell diagonal to a centre test and so marks a few extra cells near
corners. Only cells in the band where a centre test can't decide are
tested exactly, which keeps it fast enough to redo on every key press.

The world's outer boundary counts as a wall (see room_walls): the world is
a room. Without that, a grid rotated or shifted past the world's edge has
free cells outside it, and a connection could go round the end of a wall.

The world can also be *moved* relative to the grid (shifted and rotated),
to show that a coarse grid isn't one picture of the world but one of many,
depending on where the cell borders happen to fall.
"""

from collections import deque

import numpy as np

from .grid import Grid
from .world import Circle, Outside, Polygon, transform_obstacles  # noqa: F401 (re-exported)

RULES = ("any overlap", "samples")

# In which order the inflation happens:
#   world first  grow the real obstacles by `inflate`, then make cells of the
#                grown shapes. Only possible with a geometric model of the
#                world.
#   grid first   make cells of the obstacles, then grow the *occupied cells*
#                by `inflate`, in the grid. What a robot that builds its map
#                from sensor data has to do (a costmap's inflation layer):
#                the discretization error comes first and the inflation is
#                added on top of it, rounded to whole cells again -- so the
#                errors compound and gaps close sooner.
INFLATE_ORDERS = ("world first", "grid first")



# --------------------------------------------------------------------------
# Moving the world
# --------------------------------------------------------------------------

def sample_outlines(obstacles, room, spacing, sigma=0.0, rng=None):
    """Points every `spacing` metres along each obstacle's outline and the
    room's walls (`room`: its corners, or a bounds tuple for a rectangle),
    each moved by 2D Gaussian noise of std `sigma`.

    The first point on each outline is placed at a random distance along
    it (not always at a corner), so that the samples don't line up with
    anything by accident. Returned in the world file's coordinates, so they
    can be moved with the world like the obstacles themselves."""
    rng = np.random.default_rng() if rng is None else rng
    # Each point is nudged a hair into its obstacle, so that a point exactly
    # on a cell border counts for the cell on the obstacle's side (floor()
    # would otherwise always pick the cell above/right, and walls lying on
    # cell borders would get an extra column on one side only).
    nudge = 1e-9
    outlines = [(_room_corners(room), -1.0)]          # the room: nudged outwards
    pts = []
    for ob in obstacles:
        if isinstance(ob, Circle):
            n = max(int(np.ceil(2 * np.pi * ob.r / spacing)), 3)
            t = rng.uniform(0, 2 * np.pi) + np.arange(n) * 2 * np.pi / n
            pts.append(ob.c + (ob.r - nudge) * np.column_stack([np.cos(t), np.sin(t)]))
        else:
            outlines.append((ob.v, 1.0))
    for v, inwards in outlines:
        seg = np.roll(v, -1, axis=0) - v
        L = np.hypot(seg[:, 0], seg[:, 1])
        s_at = np.r_[0.0, np.cumsum(L)]                  # arc length at each vertex
        s = rng.uniform(0, spacing) + np.arange(0.0, s_at[-1], spacing)
        i = np.clip(np.searchsorted(s_at, s, side="right") - 1, 0, len(L) - 1)
        f = ((s - s_at[i]) / np.maximum(L[i], 1e-12))[:, None]
        # the polygon's inside is to the left of its edges if it runs
        # counter-clockwise (positive area), to the right otherwise
        area = 0.5 * np.sum(v[:, 0] * np.roll(v[:, 1], -1) - np.roll(v[:, 0], -1) * v[:, 1])
        left = np.column_stack([-seg[i, 1], seg[i, 0]]) / np.maximum(L[i], 1e-12)[:, None]
        pts.append(v[i] + f * seg[i] + inwards * np.sign(area) * nudge * left)
    P = np.vstack(pts)
    if sigma > 0:
        P = P + rng.normal(0.0, sigma, P.shape)
    return P


def count_hits(points, bounds, res):
    """How many points land in each cell of a Grid over `bounds`."""
    g = Grid(bounds, res, 0.0)
    ij = np.floor((points - g.origin) / res).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 0] < g.nx) & (ij[:, 1] >= 0) & (ij[:, 1] < g.ny)
    counts = np.zeros((g.nx, g.ny), dtype=int)
    np.add.at(counts, (ij[ok, 0], ij[ok, 1]), 1)
    return counts


def rasterize_samples(points, bounds, res, inflate, min_hits=1):
    """The "samples" rule: a Grid over `bounds` whose obstacle cells are
    those with at least `min_hits` points, inflated in the grid (there is
    no geometric model to inflate first). Also returns the hit counts."""
    g = Grid(bounds, res, inflate)
    pad = int(np.ceil(inflate / res)) + 1 if inflate > 0 else 0
    x0, y0 = g.origin
    big = (x0 - pad * res, x0 + (g.nx + pad) * res, y0 - pad * res, y0 + (g.ny + pad) * res)
    counts = count_hits(points, big, res)
    hit = counts >= min_hits
    g.obstacle = hit[pad:pad + g.nx, pad:pad + g.ny]
    g.occ = (inflate_cells(hit, res, inflate)[pad:pad + g.nx, pad:pad + g.ny]
             if inflate > 0 else g.obstacle.copy())
    g.counts = counts[pad:pad + g.nx, pad:pad + g.ny]
    return g


def _room_corners(room):
    if isinstance(room, tuple) and len(room) == 4 and np.ndim(room[0]) == 0:
        x0, x1, y0, y1 = room
        return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)
    return np.asarray(room, float)


def room_walls(room):
    """The room's walls as an obstacle (everything outside the room), so that
    they move (and get gridded) with the world like any other obstacle.
    `room` is a World's corners, or a bounds tuple for a rectangle."""
    return [Outside(_room_corners(room))]


# --------------------------------------------------------------------------
# Distance from cell squares to obstacles
# --------------------------------------------------------------------------

def _point_box_distance(P, C, h):
    """Distance from point(s) P to the axis-aligned squares with centres C
    (K x 2) and half-size h. Broadcasts P against C."""
    q = np.abs(P - C) - h
    return np.hypot(np.maximum(q[..., 0], 0.0), np.maximum(q[..., 1], 0.0))


def _point_segment_distance(P, a, b):
    ab = b - a
    t = np.clip(((P - a) * ab).sum(-1) / max((ab * ab).sum(), 1e-18), 0.0, 1.0)
    return np.hypot(*np.moveaxis(P - (a + t[..., None] * ab), -1, 0))


def _segment_box_distance(a, b, C, h):
    """Distance from the segment a-b to each square (centres C, half-size h):
    0 if the segment passes through the square (Liang-Barsky clipping),
    otherwise the smallest endpoint-to-square or corner-to-segment distance
    -- for two shapes that don't touch, the closest pair always involves a
    vertex of one of them."""
    d = b - a
    lo, hi = C - h, C + h
    t0 = np.zeros(len(C))
    t1 = np.ones(len(C))
    for k in (0, 1):
        if abs(d[k]) < 1e-15:
            outside = (a[k] < lo[:, k]) | (a[k] > hi[:, k])
            t0 = np.where(outside, 2.0, t0)       # never inside along this axis
        else:
            ta, tb = (lo[:, k] - a[k]) / d[k], (hi[:, k] - a[k]) / d[k]
            t0 = np.maximum(t0, np.minimum(ta, tb))
            t1 = np.minimum(t1, np.maximum(ta, tb))
    crosses = t0 <= t1
    dist = np.minimum(_point_box_distance(a, C, h), _point_box_distance(b, C, h))
    for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corner = C + np.array([sx, sy]) * h
        dist = np.minimum(dist, _point_segment_distance(corner, a, b))
    return np.where(crosses, 0.0, dist)


def square_distance(ob, C, h):
    """Distance from each cell square (centres C, half-size h) to the
    obstacle; 0 where they overlap."""
    if isinstance(ob, Circle):
        return np.maximum(_point_box_distance(ob.c, C, h) - ob.r, 0.0)
    if isinstance(ob, Outside):
        # The square reaches outside the (convex) room if any corner does;
        # otherwise its distance to the walls is smallest at a corner.
        d = np.full(len(C), np.inf)
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            d = np.minimum(d, ob.distance(C + np.array([sx, sy]) * h))
        return d
    d = np.full(len(C), np.inf)
    for a, b in zip(ob.v, np.roll(ob.v, -1, axis=0)):
        d = np.minimum(d, _segment_box_distance(a, b, C, h))
    # A square entirely inside the polygon crosses none of its edges.
    return np.where(ob.contains(C), 0.0, d)


# --------------------------------------------------------------------------
# Rasterizing
# --------------------------------------------------------------------------

def rasterize(obstacles, bounds, res, inflate, order="world first"):
    """The "any overlap" rule: a Grid over `bounds` with `occ` (obstacles
    grown by `inflate`) and `obstacle` (the obstacles themselves), inflating
    in the given `order` (see INFLATE_ORDERS)."""
    g = Grid(bounds, res, inflate)
    C = g.centres()
    d = np.full(len(C), np.inf)
    for ob in obstacles:
        d = np.minimum(d, ob.distance(C))

    # A cell that only *touches* an obstacle -- shares an edge with it, no
    # area in common -- doesn't count: round-numbered walls often lie
    # exactly on cell borders, and counting touching would make every such
    # wall a cell thicker on each side. So squares are tested a hair smaller
    # than they are.
    eps = 1e-9 * max(res, 1.0)

    def cells_within(r):
        occ = d < r - eps if r > 0 else d == 0     # centre inside: certainly overlaps
        maybe = ~occ & (d <= r + res * np.sqrt(2) / 2)
        if maybe.any():
            Cm = C[maybe]
            dm = np.full(len(Cm), np.inf)
            for ob in obstacles:
                dm = np.minimum(dm, square_distance(ob, Cm, res / 2 - eps))
            occ[np.flatnonzero(maybe)[dm <= r]] = True
        return occ

    g.obstacle = cells_within(0.0).reshape(g.nx, g.ny)
    if inflate <= 0:
        g.occ = g.obstacle.copy()
    elif order == "grid first":
        # Obstacle cells just outside the grid (the room's walls, say) must
        # be inflated into it too, so grid a margin of extra cells first,
        # inflate, and crop back.
        pad = int(np.ceil(inflate / res)) + 1
        x0, y0 = g.origin
        big = rasterize(obstacles, (x0 - pad * res, x0 + (g.nx + pad) * res,
                                    y0 - pad * res, y0 + (g.ny + pad) * res), res, 0.0)
        g.occ = inflate_cells(big.obstacle, res, inflate)[pad:pad + g.nx, pad:pad + g.ny]
        g.occ |= g.obstacle
    else:
        g.occ = cells_within(inflate).reshape(g.nx, g.ny)
    return g


def inflate_cells(occupied, res, radius):
    """Grow the occupied cells by `radius`, in the grid, as a map built from
    sensor data must: a cell becomes occupied if its square comes within
    `radius` of an occupied cell's square -- i.e. it overlaps the occupied
    cells grown by the radius, so this still never misses anything. Done as
    a dilation with a fixed stencil of cell offsets."""
    k = int(np.ceil(radius / res)) + 1
    out = occupied.copy()
    nx, ny = occupied.shape
    for i in range(-k, k + 1):
        for j in range(-k, k + 1):
            d = res * np.hypot(max(abs(i) - 1, 0), max(abs(j) - 1, 0))
            inside = d < radius - 1e-9 * res          # touching at exactly `radius` doesn't count
            if (i, j) == (0, 0) or not inside:
                continue
            # shift the occupied mask by (i, j) and OR it in
            src = occupied[max(0, -i):nx - max(0, i), max(0, -j):ny - max(0, j)]
            out[max(0, i):nx - max(0, -i), max(0, j):ny - max(0, -j)] |= src
    return out


def connected(grid, a, b):
    """Can you get from point a to point b through free cells? A flood fill
    with the moves A* makes: 8-connected, but never squeezing diagonally
    between two occupied cells that touch at a corner. (The demo itself
    plans with A*; this is the simpler check the tests use.)

    Returns (ok, reason, reached, path): `reached` is the boolean array of
    cells the fill got to, and `path` the way it found from a to b -- a, the
    centres of the cells in between, b -- or [] if there is none. The fill is
    breadth first, so that is a path with the fewest cell steps; remembering
    where each cell was reached from is all it takes to get it."""
    reached = np.zeros((grid.nx, grid.ny), dtype=bool)
    ca, cb = grid.cell(*a), grid.cell(*b)
    for name, c in (("start", ca), ("goal", cb)):
        if not grid.inside(*c):
            return False, f"{name} outside grid", reached, []
        if grid.occ[c]:
            return False, f"{name} in occupied cell", reached, []
    free = grid.free
    came_from = {ca: None}
    reached[ca] = True
    steps = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    todo = deque([ca])
    while todo:
        x, y = todo.popleft()
        for dx, dy in steps:
            nx, ny = x + dx, y + dy
            if not free(nx, ny) or reached[nx, ny]:
                continue
            if dx and dy and not (free(x + dx, y) and free(x, y + dy)):
                continue
            reached[nx, ny] = True
            came_from[(nx, ny)] = (x, y)
            todo.append((nx, ny))
    if not reached[cb]:
        return False, "no way through", reached, []
    cells = [cb]
    while came_from[cells[-1]] is not None:
        cells.append(came_from[cells[-1]])
    cells.reverse()
    path = [tuple(a)] + [grid.coord(*c) for c in cells[1:-1]] + [tuple(b)]
    return True, "connected", reached, path
