"""A path as a polyline of waypoints, parameterised by arc length.

This replaces find_closest_point.m and get_lookahead_point.m from the MATLAB
demo. Both of those walked the waypoints one by one, keeping track of which
edge the point was on and a signed offset along it. With the cumulative arc
length `s` along the path precomputed, both questions become simpler:

    closest point   -> project onto every segment in a window, keep the best
    lookahead point -> the point at arc length s_closest + L
"""

from pathlib import Path as _FsPath

import numpy as np

PATH_DIR = _FsPath(__file__).resolve().parent.parent / "paths"


class Path:
    def __init__(self, xy, name="path"):
        xy = np.asarray(xy, dtype=float)
        # Drop repeated points: a zero-length segment has no direction.
        keep = np.r_[True, np.hypot(*np.diff(xy, axis=0).T) > 1e-9]
        self.xy = xy[keep]
        if len(self.xy) < 2:
            raise ValueError("a path needs at least 2 distinct waypoints")
        self.name = name
        d = np.diff(self.xy, axis=0)
        self.seg_len = np.hypot(d[:, 0], d[:, 1])
        self.seg_dir = d / self.seg_len[:, None]          # unit vector per segment
        self.s = np.r_[0.0, np.cumsum(self.seg_len)]      # arc length at each waypoint

    @classmethod
    def load(cls, filename):
        f = _FsPath(filename)
        if not f.exists():
            f = PATH_DIR / f
        return cls(np.loadtxt(f, delimiter=",", comments="#"), name=f.name)

    def save(self, filename):
        np.savetxt(filename, self.xy, fmt="%.6f", delimiter=",", header="x,y")

    @property
    def length(self):
        return self.s[-1]

    @property
    def start(self):
        return self.xy[0]

    def closest(self, x, y, s_min=0.0, window=np.inf):
        """Closest point on the path to (x, y), among the part of the path
        with arc length in [s_min, s_min + window].

        Returns (xc, yc, s, e): the point, its arc length, and the signed
        cross-track error e (positive when the robot is to the LEFT of the
        path, looking along it).

        Restricting the search to a window ahead of the previous closest
        point is what keeps the robot from "short-cutting" where a path
        crosses itself or doubles back: the globally closest point might be
        on a later part of the path that just happens to pass nearby. The
        MATLAB version did the same thing with its minIndex argument.
        """
        lo = int(np.clip(np.searchsorted(self.s, s_min, side="right") - 1,
                         0, len(self.seg_len) - 1))
        hi = np.searchsorted(self.s, s_min + window, side="right")
        hi = int(np.clip(hi, lo + 1, len(self.seg_len)))
        p0 = self.xy[lo:hi]
        u = self.seg_dir[lo:hi]
        rel = np.array([x, y]) - p0
        # Scalar product with the segment direction = how far along the
        # segment the robot projects; clamp to the segment's own extent.
        t = np.clip(np.einsum("ij,ij->i", rel, u), 0.0, self.seg_len[lo:hi])
        # Never step back behind s_min on the first segment.
        t[0] = max(t[0], s_min - self.s[lo])
        t = np.minimum(t, self.seg_len[lo:hi])
        foot = p0 + t[:, None] * u
        dist = np.hypot(*(np.array([x, y]) - foot).T)
        k = int(np.argmin(dist))
        s = self.s[lo + k] + t[k]
        # Cross product with the segment direction gives the side: the
        # scalar product with u rotated +90 degrees.
        r = rel[k]
        cross = u[k, 0] * r[1] - u[k, 1] * r[0]
        if s >= self.length - 1e-9:
            # Past the end: only the sideways offset counts as cross-track
            # error, not how far the robot has overshot the goal.
            return foot[k, 0], foot[k, 1], s, float(cross)
        return foot[k, 0], foot[k, 1], s, float(np.sign(cross) * dist[k])

    def point_at(self, s):
        """The point at arc length s. Past the end, the last segment is
        extended in a straight line -- the same thing get_lookahead_point.m
        does, so the target keeps pulling the robot forward over the goal
        instead of collapsing onto it."""
        if s <= 0.0:
            return self.xy[0].copy()
        i = min(np.searchsorted(self.s, s, side="right") - 1, len(self.seg_len) - 1)
        return self.xy[i] + (s - self.s[i]) * self.seg_dir[i]


def builtin_paths():
    """The four paths from the MATLAB demo, in key order 1..4 (path.mat,
    path1.mat, path2.mat, path3.mat there)."""
    return [Path.load(PATH_DIR / f"path{i}.csv") for i in (1, 2, 3, 4)]
