"""The real world: obstacles described by geometric primitives.

This is the ground truth the robot actually drives in. The planners never
see it directly (unless asked to -- see GeometryChecker): they plan on an
occupancy grid made from it (grid.py), which is what a real robot would
have too, built from its sensors rather than from a CAD drawing.

Only two primitive types are needed: circles and polygons. Boxes and thick
walls are just polygons with a convenient constructor.

World files are JSON:

    {
      "name": "gap",
      "bounds": [0, 10, 0, 10],               xmin, xmax, ymin, ymax  [m]
      "start": [1, 1, 0],                     x, y, heading [deg]
      "goal": [9, 9],
      "probes": [                             optional, for run_grid.py
        {"name": "door", "from": [1, 3], "to": [5, 3]}
      ],
      "obstacles": [
        {"type": "box", "center": [5, 3], "size": [0.3, 6], "angle": 0},
        {"type": "wall", "from": [0, 5], "to": [4, 5], "thickness": 0.1},
        {"type": "circle", "center": [7, 7], "radius": 0.8},
        {"type": "polygon", "points": [[1, 1], [2, 1], [1.5, 2]]}
      ]
    }
"""

import json
from pathlib import Path as _FsPath

import numpy as np

WORLD_DIR = _FsPath(__file__).resolve().parent.parent / "worlds"


# --------------------------------------------------------------------------
# Distance helpers, vectorised over many points at once
# --------------------------------------------------------------------------

def point_segment_distance(P, a, b):
    """Distance from each row of P (N x 2) to the segment a-b."""
    P = np.atleast_2d(P)
    ab = b - a
    # sums rather than @: numpy 2 on macOS (Accelerate) emits spurious
    # divide-by-zero warnings from matmul on some inputs
    t = np.clip(((P - a) * ab).sum(axis=1) / max((ab * ab).sum(), 1e-18), 0.0, 1.0)
    return np.hypot(*(P - (a + t[:, None] * ab)).T)


def _segments_intersect(p1, p2, q1, q2):
    """Do the segments p1-p2 and q1-q2 cross (or touch)?"""
    def orient(a, b, c):
        return np.sign((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
    o1, o2 = orient(p1, p2, q1), orient(p1, p2, q2)
    o3, o4 = orient(q1, q2, p1), orient(q1, q2, p2)
    return o1 * o2 <= 0 and o3 * o4 <= 0


def segment_segment_distance(p1, p2, q1, q2):
    if _segments_intersect(p1, p2, q1, q2):
        return 0.0
    return min(point_segment_distance(p1, q1, q2)[0], point_segment_distance(p2, q1, q2)[0],
               point_segment_distance(q1, p1, p2)[0], point_segment_distance(q2, p1, p2)[0])


# --------------------------------------------------------------------------
# Primitives
# --------------------------------------------------------------------------

class Circle:
    def __init__(self, center, radius):
        self.c = np.asarray(center, dtype=float)
        self.r = float(radius)

    def distance(self, P):
        """Distance from each point to the circle, 0 inside it."""
        return np.maximum(np.hypot(*(np.atleast_2d(P) - self.c).T) - self.r, 0.0)

    def segment_distance(self, p, q):
        return max(point_segment_distance(self.c, p, q)[0] - self.r, 0.0)


class Polygon:
    def __init__(self, points):
        self.v = np.asarray(points, dtype=float)   # vertices, not closed

    def contains(self, P):
        """Point-in-polygon by ray crossing: count how many edges a ray
        going +x from each point crosses; odd = inside."""
        P = np.atleast_2d(P)
        x, y = P[:, 0], P[:, 1]
        inside = np.zeros(len(P), dtype=bool)
        for a, b in zip(self.v, np.roll(self.v, -1, axis=0)):
            crosses = (a[1] > y) != (b[1] > y)
            with np.errstate(divide="ignore", invalid="ignore"):
                x_cross = a[0] + (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            inside ^= crosses & (x < x_cross)
        return inside

    def distance(self, P):
        P = np.atleast_2d(P)
        d = np.min([point_segment_distance(P, a, b)
                    for a, b in zip(self.v, np.roll(self.v, -1, axis=0))], axis=0)
        return np.where(self.contains(P), 0.0, d)

    def segment_distance(self, p, q):
        if self.contains(p)[0] or self.contains(q)[0]:
            return 0.0
        return min(segment_segment_distance(p, q, a, b)
                   for a, b in zip(self.v, np.roll(self.v, -1, axis=0)))


class Outside:
    """Everything outside a convex polygon: the room's walls as one obstacle.

    Its distance is 0 outside the room and, inside it, the distance to the
    nearest wall. The room may be any convex quadrilateral -- real rooms are
    never quite square."""

    def __init__(self, points):
        self.v = np.asarray(points, dtype=float)
        self.room = Polygon(self.v)

    def distance(self, P):
        P = np.atleast_2d(P)
        d = np.min([point_segment_distance(P, a, b)
                    for a, b in zip(self.v, np.roll(self.v, -1, axis=0))], axis=0)
        return np.where(self.room.contains(P), d, 0.0)

    def segment_distance(self, p, q):
        # Inside a convex room the distance to the walls is a concave
        # function, so along a segment it is smallest at one of the ends.
        return float(np.min(self.distance(np.array([p, q]))))


def box(center, size, angle=0.0):
    """A rectangle, `angle` in degrees."""
    c, (w, h), a = np.asarray(center, float), size, np.deg2rad(angle)
    corners = np.array([[-w, -h], [w, -h], [w, h], [-w, h]]) / 2
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    return Polygon(np.einsum('ij,kj->ki', R, corners) + c)


def wall(p, q, thickness):
    """A straight wall of the given thickness from p to q."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = q - p
    n = np.array([-d[1], d[0]]) / np.hypot(*d) * thickness / 2
    return Polygon([p - n, q - n, q + n, p + n])


def _primitive(spec):
    kind = spec["type"]
    if kind == "circle":
        return Circle(spec["center"], spec["radius"])
    if kind == "box":
        return box(spec["center"], spec["size"], spec.get("angle", 0.0))
    if kind == "wall":
        return wall(spec["from"], spec["to"], spec.get("thickness", 0.1))
    if kind == "polygon":
        return Polygon(spec["points"])
    raise ValueError(f"unknown obstacle type {kind!r}")


# --------------------------------------------------------------------------
# The world
# --------------------------------------------------------------------------

class World:
    """Obstacles inside a room. `room` is the room's four corners
    (counter-clockwise); by default the rectangle `bounds`, but a perturbed
    world (see `imperfect`) has a slightly irregular one, and `bounds` is then
    the box around it."""

    def __init__(self, obstacles, bounds, start, goal, name="world", probes=None, room=None):
        self.obstacles = list(obstacles)
        if room is None:
            x0, x1, y0, y1 = bounds
            room = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        self.room = np.asarray(room, dtype=float)
        self.walls = Outside(self.room)
        lo, hi = self.room.min(axis=0), self.room.max(axis=0)
        self.bounds = (float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]))
        self.start = (float(start[0]), float(start[1]), np.deg2rad(start[2]) if len(start) > 2 else 0.0)
        self.goal = (float(goal[0]), float(goal[1]))
        self.name = name
        # Pairs of points run_grid.py checks for a free connection through
        # the grid: [(name, (x, y), (x, y)), ...]. Start to goal by default.
        self.probes = probes or [("start-goal", self.start[:2], self.goal)]

    @classmethod
    def load(cls, filename):
        f = _FsPath(filename)
        if not f.exists():
            f = WORLD_DIR / f
        spec = json.loads(f.read_text())
        probes = [(p["name"], tuple(p["from"]), tuple(p["to"])) for p in spec.get("probes", [])]
        return cls([_primitive(o) for o in spec["obstacles"]], spec["bounds"],
                   spec["start"], spec["goal"], name=spec.get("name", f.stem), probes=probes)

    def distance(self, P):
        """Distance from each point to the nearest obstacle (0 inside one).
        The room's walls count as an obstacle too."""
        P = np.atleast_2d(P)
        d = self.walls.distance(P)
        for ob in self.obstacles:
            d = np.minimum(d, ob.distance(P))
        return d

    def segment_distance(self, p, q):
        p, q = np.asarray(p, float), np.asarray(q, float)
        d = float(np.min(self.distance(np.array([p, q]))))  # includes the boundary
        for ob in self.obstacles:
            if d <= 0.0:
                break
            d = min(d, ob.segment_distance(p, q))
        return d


def imperfect(world, sigma, rng):
    """The world as built rather than as drawn: every polygon corner (walls,
    boxes, the room's own corners) moved independently by 2D Gaussian noise
    of std `sigma`, and every circle moved and resized a little. Walls come
    out slightly tapered and rooms not quite square, and nothing lines up
    with the cell lattice by accident any more. Start, goal and probe points
    stay where they were designed."""
    if sigma <= 0:
        return world
    obs = []
    for ob in world.obstacles:
        if isinstance(ob, Circle):
            obs.append(Circle(ob.c + rng.normal(0, sigma, 2),
                              max(ob.r + rng.normal(0, sigma / 2), 0.02)))
        else:
            obs.append(Polygon(ob.v + rng.normal(0, sigma, ob.v.shape)))
    room = world.room + rng.normal(0, sigma, world.room.shape)
    w = World(obs, world.bounds, (*world.start[:2], np.rad2deg(world.start[2])), world.goal,
              name=world.name, probes=world.probes, room=room)
    return w


def builtin_worlds():
    """The worlds in worlds/, in key order 1..9 (sorted by file name)."""
    return [World.load(f) for f in sorted(WORLD_DIR.glob("*.json"))][:9]


def grid_worlds():
    """For run_grid.py: the worlds made for it (worlds/grid/) first, then
    the planning worlds, 9 in all."""
    made = [World.load(f) for f in sorted((WORLD_DIR / "grid").glob("*.json"))]
    return (made + builtin_worlds())[:9]
