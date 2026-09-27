"""Obstacle avoidance: local, reactive methods that only see the lidar.

The world has three kinds of obstacles:

    static     walls and furniture that are in the map as well
    unmapped   things the map doesn't know about: a box someone left, a
               moved chair. Real, and seen by the lidar, but not in the map
               the global planner (A*) uses.
    movers     people: circles walking back and forth along a path, with
               an optional pause at each end.

A local method never sees the map or the geometry, only the latest scan.
Each control step it turns the scan and a (local) goal into speed
commands (v, w) -- which the robot then follows as far as its
acceleration limits allow. The goal is either the one you click (you are
the global planner) or a "carrot" some way ahead along an A* path planned
on the map, which knows nothing about the unmapped obstacles or the people.

The methods:

    potential field   the goal pulls, every lidar hit pushes; steer along
                      the sum. No idea of the robot's dynamics.
    DWA               the dynamic window approach: try the (v, w) the robot
                      can reach within one control period, predict each arc,
                      drop those that hit something or couldn't stop in
                      time, and score the rest (towards the goal, clearance,
                      speed). Built around the dynamics.
    VFH               the vector field histogram: bin the hits by direction
                      into a polar histogram, and head for the free direction
                      closest to the goal.

World files are the planning demo's format (see world.py) plus two optional
lists:

    "unmapped": [ ...obstacles, as in "obstacles"... ],
    "movers": [ {"radius": 0.25, "path": [[x, y], [x, y], ...],
                 "speed": 0.8, "pause": 2.0, "phase": 0.0}, ... ]
"""

import json
from pathlib import Path as _FsPath

import numpy as np

from .grid import Grid
from .mapping import Lidar
from .path import Path
from .planners import astar
from .sim import Robot, PHYSICS_DT, wrap_angle
from .world import World, WORLD_DIR, Circle, _primitive

AVOID_DIR = WORLD_DIR / "avoid"

METHODS = ("VFH", "potential field", "DWA")      # the first is the default

# Extra room DWA and VFH keep between the robot's edge and an obstacle: the
# robot doesn't follow a prediction exactly (it drives on for a control
# period, and its speeds lag the commands).
SAFETY = 0.1   # m
GOAL_MODES = ("clicked goal", "A* carrot")


# ==========================================================================
# The world: static + unmapped + moving
# ==========================================================================

class Mover:
    """A person: a circle walking back and forth along a polyline at
    `speed`, pausing `pause` seconds at each end. `phase` [s] staggers
    several movers on the same kind of path."""

    def __init__(self, radius, path, speed=0.8, pause=0.0, phase=0.0):
        self.r = float(radius)
        self.path = Path(np.asarray(path, dtype=float), name="mover")
        self.speed = float(speed)
        self.pause = float(pause)
        self.phase = float(phase)
        self.t = 0.0
        self.waited = 0.0
        self.c = self.position(0.0)

    def _cycle(self):
        leg = self.path.length / max(self.speed, 1e-9)
        return leg, 2 * (leg + self.pause)

    def turn_back(self):
        """Walk back the way this person came, from where they are: the
        same place on the other leg of the back-and-forth."""
        leg, cycle = self._cycle()
        u = (self.t + self.phase) % cycle
        if u < leg or leg + self.pause <= u < 2 * leg + self.pause:
            self.t += (2 * leg + self.pause - u) - u

    def position(self, t):
        L = self.path.length
        leg, cycle = self._cycle()
        u = (t + self.phase) % cycle
        if u < leg:
            s = u * self.speed
        elif u < leg + self.pause:
            s = L
        elif u < 2 * leg + self.pause:
            s = L - (u - leg - self.pause) * self.speed
        else:
            s = 0.0
        return self.path.point_at(min(max(s, 0.0), L))

    YIELD_AFTER = 2.0    # s of waiting for the robot before turning back

    def advance(self, dt, robot=None, robot_radius=0.0, gap=0.3):
        """Walk on for dt -- unless `robot` (x, y) is given and the next
        step would bring this person closer than `gap` to it: then wait,
        as people do when a robot is in their way, and after YIELD_AFTER
        seconds give way and walk back (otherwise a robot and a person
        meeting in a doorway would wait for each other for ever)."""
        c = self.position(self.t + dt)
        if robot is not None:
            d_now = np.hypot(*(self.c - robot)) - self.r - robot_radius
            d_new = np.hypot(*(c - robot)) - self.r - robot_radius
            if d_new < gap and d_new < d_now:
                self.waited += dt
                if self.waited >= self.YIELD_AFTER:
                    self.turn_back()
                    self.waited = 0.0
                return
        self.waited = 0.0
        self.t += dt
        self.c = c

    def reset(self):
        self.t = 0.0
        self.waited = 0.0
        self.c = self.position(0.0)

    def distance(self, P):
        return np.maximum(np.hypot(P[:, 0] - self.c[0], P[:, 1] - self.c[1]) - self.r, 0.0)


class AvoidWorld:
    """The real world for the avoidance demo: the map's world (`mapped`),
    plus what the map doesn't have. distance() -- used by the lidar and
    the collision check -- sees all of it."""

    def __init__(self, mapped, unmapped=(), movers=(), name=None, comment=""):
        self.mapped = mapped
        self.unmapped = list(unmapped)
        self.movers = list(movers)
        self.people_on = True          # 'p': the people can be taken out of the world
        self.name = name or mapped.name
        self.comment = comment
        self.bounds = mapped.bounds
        self.start = mapped.start
        self.goal = mapped.goal

    @classmethod
    def load(cls, filename):
        f = _FsPath(filename)
        if not f.exists():
            f = AVOID_DIR / f
        spec = json.loads(f.read_text())
        mapped = World.load(f)
        unmapped = [_primitive(o) for o in spec.get("unmapped", [])]
        movers = [Mover(m["radius"], m["path"], m.get("speed", 0.8), m.get("pause", 0.0),
                        m.get("phase", 0.0)) for m in spec.get("movers", [])]
        return cls(mapped, unmapped, movers, spec.get("name", f.stem), spec.get("comment", ""))

    def distance(self, P, static_only=False):
        P = np.atleast_2d(P)
        d = self.mapped.distance(P)
        for ob in self.unmapped:
            d = np.minimum(d, ob.distance(P))
        if not static_only and self.people_on:
            for m in self.movers:
                d = np.minimum(d, m.distance(P))
        return d

    def advance(self, dt, robot=None, robot_radius=0.0):
        """People walk on; with `robot` given, they wait rather than walk
        into it (Mover.advance). Without, they walk blindly."""
        if not self.people_on:
            return
        for m in self.movers:
            m.advance(dt, robot, robot_radius)

    def reset(self):
        for m in self.movers:
            m.reset()


def avoid_worlds():
    return [AvoidWorld.load(f) for f in sorted(AVOID_DIR.glob("*.json"))]


# ==========================================================================
# The local methods: scan + local goal -> (v, w)
# ==========================================================================

def steer(pose, heading, cfg):
    """Turn a desired heading into (v, w): turn towards it (P on the
    heading error, capped at w_max) and slow down the further off it is
    -- stopping to turn when it is more than 90 degrees off."""
    err = wrap_angle(heading - pose[2])
    w = float(np.clip(2.0 * err, -cfg["wmax"], cfg["wmax"]))
    v = cfg["vmax"] * max(np.cos(err), 0.0) ** 2
    return v, w, err


def potential_field_cmd(pose, hits, goal, cfg, robot_radius, weights=None):
    """The goal pulls (strength 1, less within 1 m of it); each obstacle
    point within d0 pushes, k_rep * (1/rho - 1/d0) / rho^2 with rho the gap
    between the robot's edge and the point, weighted by the stretch of
    surface the point stands for (`weights` [m]: for a lidar hit its angular
    step times its range, for a local map cell its size times its
    certainty), so neither the ray count nor the cell size changes the
    push. Steer along the sum."""
    x = np.array(pose[:2])
    g = np.asarray(goal) - x
    f_att = g / max(np.hypot(*g), 1.0)
    f_rep = np.zeros(2)
    if len(hits):
        dv = x - hits
        d = np.hypot(dv[:, 0], dv[:, 1])
        rho = np.maximum(d - robot_radius, 0.01)
        near = rho < cfg["d0"]
        if near.any():
            weight = (weights[near] if weights is not None
                      else (2 * np.pi / cfg["rays"]) * d[near])
            mag = cfg["k_rep"] * weight * (1 / rho[near] - 1 / cfg["d0"]) / rho[near] ** 2
            f_rep = (mag / np.maximum(d[near], 1e-9)) @ dv[near]
    f = f_att + f_rep
    v, w, _ = steer(pose, np.arctan2(f[1], f[0]), cfg)
    return v, w, {"f_att": f_att, "f_rep": f_rep, "f": f}


def predict_arc(pose, v, w, horizon, dt=0.1):
    """Poses along the arc of constant (v, w) from `pose`, every dt."""
    n = max(int(round(horizon / dt)), 1)
    t = dt * np.arange(1, n + 1)
    x, y, a = pose
    if abs(w) < 1e-6:
        xs, ys, As = x + v * t * np.cos(a), y + v * t * np.sin(a), np.full(n, a)
    else:
        As = a + w * t
        xs = x + v / w * (np.sin(As) - np.sin(a))
        ys = y - v / w * (np.cos(As) - np.cos(a))
    return np.column_stack([xs, ys, As])


def free_distance(pose, v, w, hits, robot_radius, cap=3.0, ds=0.05):
    """How far the robot can drive along the curve of (v, w) before it
    comes within `robot_radius` of a hit: Fox et al.'s dist(v, w). `cap` if
    nothing is in the way. For v = 0 (turning on the spot) it is what lies
    straight ahead -- otherwise standing still would always look safest."""
    if not len(hits):
        return cap
    s = np.arange(ds, cap + 1e-9, ds)
    k = w / v if v > 1e-6 else 0.0      # standing still: what is straight ahead
    x, y, a = pose
    if abs(k) < 1e-6:
        px, py = x + s * np.cos(a), y + s * np.sin(a)
    else:
        px = x + (np.sin(a + k * s) - np.sin(a)) / k
        py = y - (np.cos(a + k * s) - np.cos(a)) / k
    d = np.hypot(px[:, None] - hits[None, :, 0], py[:, None] - hits[None, :, 1]).min(axis=1)
    # Already closer than that (someone walked up to the robot)? Then only
    # getting closer still counts -- or nothing, not even backing away,
    # would be allowed, and the robot would freeze.
    now = float(np.hypot(hits[:, 0] - x, hits[:, 1] - y).min())
    bad = np.flatnonzero(d < min(robot_radius, now - 1e-3))
    return cap if not len(bad) else float(s[bad[0]] - ds)


def dwa_cmd(pose, vel, hits, goal, cfg, robot_radius, ctrl_dt, nv=7, nw=15, cap=3.0):
    """The dynamic window approach (Fox, Burgard & Thrun 1997).

    The window: the (v, w) reachable from the current speeds within one
    control period under the acceleration limits. For each candidate,
    dist = how far along its curve the robot can go before hitting
    something (free_distance). Admissible: the robot can still stop in
    that distance, counting the control period it drives on before it can
    react: v dt + v^2 / (2 acc_v) <= dist. The admissible ones are scored

        w_head  * (1 - |angle to goal after `horizon` s| / pi)
      + w_clear * dist / 3 m
      + w_vel   * v / v_max

    (an arc that passes the goal counts as heading straight for it) and
    the best is sent. If none is admissible, stop and turn on the spot
    towards the goal. The arcs drawn are the `horizon` s predictions."""
    v0, w0 = vel
    dv, dw = cfg["accv"] * ctrl_dt, cfg["accw"] * ctrl_dt
    vs = np.linspace(max(0.0, v0 - dv), min(cfg["vmax"], v0 + dv), nv)
    ws = np.linspace(max(-cfg["wmax"], w0 - dw), min(cfg["wmax"], w0 + dw), nw)
    arcs, info = [], []
    best, best_i, best_score = None, None, -np.inf
    for v in vs:
        for w in ws:
            arc = predict_arc(pose, v, w, cfg["horizon"])
            dist = free_distance(pose, v, w, hits, robot_radius + SAFETY, cap)
            # it keeps going for one control period before it can start
            # braking, and then needs v^2 / (2 acc_v) to stop
            ok = v <= 1e-6 or v * ctrl_dt + v ** 2 / (2 * cfg["accv"]) <= dist + 1e-9
            end = arc[-1]
            to_goal = np.arctan2(goal[1] - end[1], goal[0] - end[0])
            head = 1 - abs(wrap_angle(to_goal - end[2])) / np.pi
            if np.hypot(*(arc[:, :2] - goal).T).min() < 0.15:
                head = 1.0               # this arc gets to the goal on the way
            arcs.append(np.vstack([np.array(pose[:2]), arc[:, :2]]))
            info.append(bool(ok))
            if ok:
                score = (cfg["w_head"] * head + cfg["w_clear"] * dist / cap
                         + cfg["w_vel"] * v / cfg["vmax"])
                if score > best_score:
                    best, best_i, best_score = (float(v), float(w)), len(arcs) - 1, score
    if best is None:
        # nothing admissible: stop and turn on the spot towards the goal
        err = wrap_angle(np.arctan2(goal[1] - pose[1], goal[0] - pose[0]) - pose[2])
        best = (0.0, float(np.sign(err) * cfg["wmax"]))
    return best[0], best[1], {"arcs": arcs, "ok": info, "best": best_i}


def vfh_cmd(pose, hits, goal, cfg, robot_radius, sector_deg=5.0, weights=None):
    """The vector field histogram (Borenstein & Koren 1991), simplified.

    Every hit within the window (`vfh_win`, or less near the goal: nothing
    beyond the goal matters) adds to the sector of its direction -- more
    the closer it is (1 - d / window) -- spread over the sectors the robot
    would sweep passing it at a safe distance. Sectors
    below the threshold are free, and runs of free sectors are valleys:
    head for the goal through a wide valley (keeping clear of its edges), or
    through the middle of a narrow one (_vfh_direction); if nothing is free,
    stop and turn towards the goal.

    Each point counts by the stretch of surface it stands for (`weights`,
    as for the potential field) relative to a hit of a 180-ray scan at the
    same range -- so a scan and a local map give the same histogram, and
    a faded map cell counts less, as in the original certainty grid."""
    n = int(round(360 / sector_deg))
    width = 2 * np.pi / n
    hist = np.zeros(n)
    x = np.array(pose[:2])
    # near the goal, what lies beyond it doesn't matter
    dg = np.hypot(goal[0] - x[0], goal[1] - x[1])
    win = min(cfg["vfh_win"], dg + robot_radius + SAFETY)
    if len(hits):
        dv = hits - x
        d = np.hypot(dv[:, 0], dv[:, 1])
        if weights is None:
            weights = (2 * np.pi / cfg["rays"]) * d
        ref = (2 * np.pi / 180) * np.maximum(d, 0.05)    # a 180-ray hit at that range
        near = d < win
        for (dx, dy), dd, wt, rf in zip(dv[near], d[near], weights[near], ref[near]):
            m = wt / rf * (1 - dd / cfg["vfh_win"])
            ang = np.arctan2(dy, dx)
            half = np.arcsin(min((robot_radius + SAFETY) / max(dd, 1e-6), 1.0))
            k0 = int(np.floor((ang - half + np.pi) / width))
            k1 = int(np.floor((ang + half + np.pi) / width))
            for k in range(k0, k1 + 1):
                hist[k % n] += m
    free = hist < cfg["vfh_thr"]
    centres = -np.pi + (np.arange(n) + 0.5) * width
    to_goal = np.arctan2(goal[1] - x[1], goal[0] - x[0])
    choice = _vfh_direction(free, centres, width, to_goal)
    if choice is None:
        err = wrap_angle(to_goal - pose[2])
        return 0.0, float(np.sign(err) * cfg["wmax"]), {"hist": hist, "free": free,
                                                        "choice": None, "goal_dir": to_goal}
    v, w, _ = steer(pose, choice, cfg)
    return v, w, {"hist": hist, "free": free, "choice": choice, "goal_dir": to_goal}


def _vfh_direction(free, centres, width, to_goal, wide=8, margin=2):
    """VFH's choice of direction from the free sectors, by valleys (runs of
    free sectors). In a wide valley (at least `wide` sectors), the direction
    closest to the goal that is at least `margin` sectors from the valley's
    edges -- or the goal itself if it lies there. In a narrow valley (a
    door), its middle. Of all the valleys' candidates, the one closest to
    the goal. None if nothing is free."""
    n = len(free)
    if not free.any():
        return None
    if free.all():
        return to_goal
    goal_k = int(np.floor((wrap_angle(to_goal) + np.pi) / width)) % n
    # rotate so that index 0 is blocked, then valleys don't wrap round
    start = int(np.flatnonzero(~free)[0])
    order = (np.arange(n) + start) % n
    f = free[order]
    cands = []
    k = 0
    while k < n:
        if not f[k]:
            k += 1
            continue
        j = k
        while j < n and f[j]:
            j += 1
        idx = order[k:j]                     # one valley, in angular order
        if len(idx) >= wide:
            inner = idx[margin:len(idx) - margin]
            if goal_k in inner:
                cands.append(to_goal)        # the goal is in the valley's safe part
            else:
                lo, hi = centres[inner[0]], centres[inner[-1]]
                cands.append(lo if abs(wrap_angle(to_goal - lo)) < abs(wrap_angle(to_goal - hi))
                             else hi)
        else:
            mid = centres[idx[0]] + wrap_angle(centres[idx[-1]] - centres[idx[0]]) / 2
            cands.append(mid)
        k = j
    return min(cands, key=lambda c: abs(wrap_angle(c - to_goal)))




# ==========================================================================
# The local map
# ==========================================================================

class LocalMap:
    """A small grid that moves with the robot and remembers what the lidar
    has seen: `size` x `size` metres of `res` cells, centred on the robot
    but aligned with the world, so a remembered obstacle stays where it is
    as the robot drives (the pose is exact in this demo; on a real robot
    odometry drift would smear it).

    Each cell holds a certainty in [0, 1]. Every scan:
      - forget (optional): every cell fades by half every `half_life` s;
      - clear along rays (optional): the cells each ray passes through,
        up to its hit, are set to 0 -- what the robot can see through is
        free now;
      - the cells the hits land in are set to 1.
    With neither option the map only ever adds: people leave trails, and a
    doorway someone stood in stays blocked. Cells that leave the window as
    the robot moves on are dropped."""

    def __init__(self, size=6.0, res=0.1):
        self.res = res
        self.n = int(round(size / res))
        self.c = np.zeros((self.n, self.n))
        self.i0 = self.j0 = None          # world cell index of self.c[0, 0]

    def clear(self):
        self.c[:] = 0.0
        self.i0 = self.j0 = None

    def recenter(self, x, y):
        i0 = int(np.floor(x / self.res)) - self.n // 2
        j0 = int(np.floor(y / self.res)) - self.n // 2
        if self.i0 is not None and (i0, j0) != (self.i0, self.j0):
            di, dj = i0 - self.i0, j0 - self.j0
            new = np.zeros_like(self.c)
            n = self.n
            if abs(di) < n and abs(dj) < n:
                src = self.c[max(di, 0):n + min(di, 0), max(dj, 0):n + min(dj, 0)]
                new[max(-di, 0):n + min(-di, 0), max(-dj, 0):n + min(-dj, 0)] = src
            self.c = new
        self.i0, self.j0 = i0, j0

    def index(self, P):
        """Local cell indices of world points P (N x 2), and which are inside."""
        i = np.floor(P[:, 0] / self.res).astype(int) - self.i0
        j = np.floor(P[:, 1] / self.res).astype(int) - self.j0
        inside = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i, j, inside

    def update(self, pose, scan, cfg, dt):
        x, y, _ = pose
        self.recenter(x, y)
        if cfg.get("map_forget", False):
            self.c *= 0.5 ** (dt / cfg["half_life"])
        ang, rng_, hit = scan
        if cfg.get("map_clear", False):
            # points every half cell along each ray, stopping short of its hit
            t = np.arange(0.0, rng_.max() + 1e-9, self.res / 2)
            T = t[None, :]
            ok = T < (rng_[:, None] - np.where(hit, self.res, 0.0)[:, None])
            P = np.column_stack([(x + T * np.cos(ang)[:, None])[ok],
                                 (y + T * np.sin(ang)[:, None])[ok]])
            i, j, inside = self.index(P)
            self.c[i[inside], j[inside]] = 0.0
        H = np.column_stack([x + rng_[hit] * np.cos(ang[hit]), y + rng_[hit] * np.sin(ang[hit])])
        i, j, inside = self.index(H)
        self.c[i[inside], j[inside]] = 1.0

    def cells(self, threshold):
        """Centres and certainties of the cells above `threshold`."""
        i, j = np.nonzero(self.c > threshold)
        if self.i0 is None or not len(i):
            return np.empty((0, 2)), np.empty(0)
        P = np.column_stack([(i + self.i0 + 0.5) * self.res, (j + self.j0 + 0.5) * self.res])
        return P, self.c[i, j]

    @property
    def extent(self):
        x0, y0 = self.i0 * self.res, self.j0 * self.res
        return (x0, x0 + self.n * self.res, y0, y0 + self.n * self.res)


# ==========================================================================
# The simulation
# ==========================================================================

class AvoidSim:
    """The robot, its lidar and the local method, driving in an AvoidWorld
    towards a clicked goal or an A* carrot."""

    CTRL_DT = 0.1         # s: control and scan period (10 Hz, typical for DWA)
    CHECK_DT = 0.02       # s: collision check and people's motion
    AT_GOAL = 0.15        # m
    STUCK_TIME = 10.0     # s without getting closer: "stuck"

    def __init__(self, world, robot_radius, rng=None):
        self.world = world
        self.robot_radius = robot_radius
        self.rng = np.random.default_rng() if rng is None else rng
        self.goal = tuple(world.goal)
        self.start = tuple(world.start)
        self.map = None
        self.plan = None
        self.reset()

    # ---- setup ---------------------------------------------------------

    def reset(self):
        self.world.reset()
        self.robot = Robot(*self.start)
        self.t = 0.0
        self.trail = [(self.robot.x, self.robot.y)]
        self.driving = False
        self.done = self.collided = False
        self.hit_by = ""
        self.cmd = (0.0, 0.0)
        self.viz = {}
        self.scan = None
        self.hits = np.empty((0, 2))
        self.hit_weights = np.empty(0)
        self.local = LocalMap()
        self.min_clear = np.inf
        self._next_ctrl = 0.0
        self._best = (np.inf, 0.0)       # (closest to the goal so far, when)
        self.s = 0.0                     # progress along the A* path

    def build_map(self, cfg):
        """The map the global planner has: the static world only."""
        self.map = Grid.from_world(self.world.mapped, cfg["res"], cfg["inflate"])

    def plan_global(self, cfg):
        """A* on the map from the robot to the goal. It knows nothing of
        the unmapped obstacles or the people."""
        if self.map is None:
            self.build_map(cfg)
        res = astar(self.map, (self.robot.x, self.robot.y), self.goal, True, 1.0)
        self.plan = Path(np.asarray(res.path), name="A*") if res.path else None
        self.s = 0.0
        return res

    def local_goal(self, cfg):
        """Where the local method is heading: the clicked goal, or the
        carrot `carrot` metres ahead of the robot along the A* path."""
        if cfg["goal_mode"] == GOAL_MODES[0] or self.plan is None:
            return np.asarray(self.goal, dtype=float)
        _, _, s, _ = self.plan.closest(self.robot.x, self.robot.y, s_min=self.s,
                                       window=cfg["carrot"] + 2.0)
        self.s = s
        return self.plan.point_at(min(s + cfg["carrot"], self.plan.length))

    # ---- running -------------------------------------------------------

    def sense(self, cfg):
        self.world.people_on = cfg.get("people_on", True)
        fov = cfg.get("fov", 2 * np.pi)
        lidar = Lidar(cfg["sensor_range"], int(cfg["rays"]), 0.0, self.rng, fov)
        ang, rng_, hit = lidar.scan(self.world, self.robot.x, self.robot.y, self.robot.a)
        self.scan = (ang, rng_, hit)
        x, y = self.robot.x, self.robot.y
        self.hits = np.column_stack([x + rng_[hit] * np.cos(ang[hit]),
                                     y + rng_[hit] * np.sin(ang[hit])])
        # the stretch of surface each hit stands for: angular step x range
        step = fov / (int(cfg["rays"]) - (0 if fov >= 2 * np.pi - 1e-9 else 1))
        self.hit_weights = step * rng_[hit]

    def obstacles(self, cfg):
        """What the method gets: the latest scan's hits, or -- with the local
        map on -- its cells. Returns (points, weights [m], points for DWA):
        DWA needs yes/no obstacles, so it gets the cells that are more
        likely occupied than not."""
        if not cfg.get("local_map", False):
            self.local.clear()          # switched on again, it starts empty
            return self.hits, self.hit_weights, self.hits
        self.local.update(self.robot.pose, self.scan, cfg, self.CTRL_DT)
        pts, c = self.local.cells(0.05)
        return pts, self.local.res * c, pts[c >= 0.5]

    def control(self, cfg):
        self.sense(cfg)
        pts, wts, binary = self.obstacles(cfg)
        pose = self.robot.pose
        self.carrot = self.local_goal(cfg)
        m = cfg["method"]
        if m == "potential field":
            v, w, self.viz = potential_field_cmd(pose, pts, self.carrot, cfg,
                                                 self.robot_radius, wts)
        elif m == "DWA":
            v, w, self.viz = dwa_cmd(pose, (self.robot.v, self.robot.w), binary,
                                     self.carrot, cfg, self.robot_radius, self.CTRL_DT)
        else:
            v, w, self.viz = vfh_cmd(pose, pts, self.carrot, cfg, self.robot_radius,
                                     weights=wts)
        # brake into the goal itself, as the other demos do, and stop on it
        dg = np.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
        v = min(v, np.sqrt(2 * 0.5 * cfg["accv"] * max(dg - 0.05, 0.0)))
        if dg < self.AT_GOAL:
            v, w = 0.0, 0.0
        self.cmd = (v, w)

    def advance(self, duration, cfg):
        if not self.driving or self.done or self.collided:
            return
        self.world.people_on = cfg.get("people_on", True)
        n = max(int(round(duration / PHYSICS_DT)), 1)
        per_check = int(round(self.CHECK_DT / PHYSICS_DT))
        for i in range(n):
            if self.t >= self._next_ctrl - 1e-9:
                self._next_ctrl += self.CTRL_DT
                self.control(cfg)
            self.robot.physics_step(self.cmd[0], self.cmd[1], cfg["accv"], cfg["accw"])
            self.t += PHYSICS_DT
            if i % per_check:
                continue
            self.world.advance(self.CHECK_DT * cfg["people"],
                               np.array([self.robot.x, self.robot.y]) if cfg["polite"] else None,
                               self.robot_radius)
            p = np.array([[self.robot.x, self.robot.y]])
            self.trail.append((self.robot.x, self.robot.y))
            clear = float(self.world.distance(p)[0]) - self.robot_radius
            self.min_clear = min(self.min_clear, clear)
            if clear <= 0:
                self.collided = True
                self.driving = False
                self.hit_by = ("a person" if float(self.world.distance(p, static_only=True)[0])
                               > self.robot_radius else "an obstacle")
                return
            dg = np.hypot(self.goal[0] - self.robot.x, self.goal[1] - self.robot.y)
            if dg < self._best[0] - 0.05:
                self._best = (dg, self.t)
            if dg < self.AT_GOAL and abs(self.robot.v) < 0.05:
                self.done = True
                self.driving = False
                return

    @property
    def stuck(self):
        """No closer to the goal for STUCK_TIME seconds of driving."""
        return self.driving and self.t - self._best[1] > self.STUCK_TIME
