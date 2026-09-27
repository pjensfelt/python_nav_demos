"""One navigation mission: map, plan, drive, and (when mapping as we go)
sense and replan.

    known map    the grid is made from the real geometry up front, the
                 plan is made once, the robot drives it.
    mapped       the robot starts with an empty map and a lidar. It plans
                 on what it has seen so far (unknown = free), drives, and
                 every scan checks whether something new blocks the rest
                 of its path. If so it replans from where it is.

In both cases the robot drives in the *real* world, so whatever the grid
got wrong -- a vanished thin wall, too little inflation -- shows up as a
collision with the real geometry.

`cfg` is a plain dict of settings (PlanState.mission_config() in the demo),
passed in on every call so parameter changes take effect immediately.
"""

import numpy as np

from .grid import GridChecker, GeometryChecker, known_map
from .rasterize import sample_outlines
from .mapping import Lidar, MappedGrid
from .path import Path
from .planners import (astar, rrt, rrtstar, potential_field, PlanResult, shortcut, spline, path_length, smoothed_cost,
                       max_curvature, PATH_MODES)
from .sim import Robot, Follower


class Mission:
    SENSE_DT = 0.1     # s between lidar scans while driving
    CHECK_DT = 0.02    # s between checks for a collision with the real world

    def __init__(self, world, robot_radius, rng=None):
        self.world = world
        self.robot_radius = robot_radius
        self.rng = np.random.default_rng() if rng is None else rng
        self.start = world.start
        self.goal = world.goal
        self.grid = None
        self.lidar = None
        self.scan = None

    # ---- the map ------------------------------------------------------------

    def build_map(self, cfg, samples=None):
        """(Re)make the grid for the current settings and reset the mission.

        The known map is made from `samples`, points on the obstacles'
        outlines as an earlier survey with a sensor would have left them
        (see grid.known_map); the demo passes them in so they stay fixed to
        the world. Without them, a fresh set is sampled here."""
        if cfg["mapped"]:
            self.grid = MappedGrid(self.world.bounds, cfg["res"], cfg["inflate"])
            self.lidar = Lidar(cfg["sensor_range"], cfg["rays"], cfg["noise"], self.rng)
        else:
            if samples is None:
                samples = sample_outlines(self.world.obstacles, self.world.room, cfg["spacing"],
                                          cfg["sample_sigma"], self.rng)
            self.grid = known_map(self.world, samples, cfg["res"], cfg["inflate"],
                                  int(cfg["min_hits"]))
            self.lidar = None
        # the survey's points, which the potential field is pushed away from
        self.samples = None if cfg["mapped"] else np.asarray(samples)
        self.reset()

    def reset(self):
        """Robot back to the start, plan and history cleared; a mapped
        grid keeps nothing (a fresh start means a fresh map)."""
        if self.lidar is not None:
            self.grid = MappedGrid(self.world.bounds, self.grid.res, self.grid.inflate)
        self.robot = Robot(*self.start)
        self.follower = None
        self.result = None           # the latest PlanResult
        self.path = []               # the path to drive: the plan, shortcut and/or splined
        self.path_mode = PATH_MODES[0]
        self.spline_spacing = None   # the control-point spacing the spline ended up with
        self.path_note = ""          # e.g. why there is no spline
        self.cost = None             # A*'s cost for being near obstacles, if used
        self._cost_key = None
        self.old_paths = []          # earlier plans, for showing the replans
        self.trail = [(self.robot.x, self.robot.y)]
        self.t = 0.0
        self.driving = False
        self.done = False
        self.collided = False
        self.failed = ""
        self.replans = 0
        self._next_sense = 0.0
        self.scan = None
        if self.lidar is not None:
            self.sense()

    def sense(self):
        """One lidar scan from the robot's pose, folded into the map.
        Returns True if new obstacles appeared."""
        x, y, a = self.robot.pose
        self.scan = (x, y) + self.lidar.scan(self.world, x, y, a)
        return self.grid.integrate(*self.scan)

    # ---- planning -----------------------------------------------------------

    def checker(self, cfg):
        # (cfg["exact_geometry"] is already False for A* and when mapping,
        # see PlanState.uses_exact_geometry)
        if cfg["exact_geometry"] and cfg["planner"] in ("RRT", "RRT*") and self.lidar is None:
            return GeometryChecker(self.world, cfg["inflate"])
        return GridChecker(self.grid, ignore={self.grid.cell(self.robot.x, self.robot.y)})

    def plan(self, cfg):
        """Plan from the robot's current position to the goal."""
        start = (self.robot.x, self.robot.y)
        checker = self.checker(cfg)
        p = cfg["planner"]
        if p == "A*":
            self.update_cost(cfg)
            res = astar(self.grid, start, self.goal, cfg["eight"], cfg["h_weight"],
                        free=checker.free, cost=self.cost,
                        cost_weight=cfg.get("cost_weight", 0))
        elif p == "potential field":
            if self.samples is None:
                res = PlanResult([], message="the potential field needs the known map's "
                                             "sample points (m)")
            else:
                res = potential_field(self.samples, start, self.goal, self.world.bounds,
                                      cfg["k_rep"], cfg["d0"], cfg["inflate"], cfg["spacing"])
        elif p == "RRT":
            res = rrt(checker, self.world.bounds, start, self.goal, cfg["iterations"],
                      cfg["step"], cfg["goal_bias"], cfg["stop_at_goal"], self.rng)
        else:
            res = rrtstar(checker, self.world.bounds, start, self.goal, cfg["iterations"],
                          cfg["step"], cfg["goal_bias"], cfg["radius"], self.rng)
        self.result = res
        if self.path:
            self.old_paths.append(self.path)
        # the path to drive: the plan, smoothed as selected with 's' --
        # every plan, replans while driving included
        self.path, self.path_mode, self.path_note, self.spline_spacing = res.path, PATH_MODES[0], "", None
        self.failed = "" if res.path else res.message
        if self.path:
            self.set_path_mode(cfg.get("path_mode", PATH_MODES[0]), cfg)
        return res

    def update_cost(self, cfg):
        """A*'s cost for being near obstacles (planners.smoothed_cost), or
        None when it is off or the planner isn't A*. Recomputed only when
        the planning map or cost_sigma has changed -- so it can be shown
        before planning, and follows the map while mapping as we go."""
        if cfg["planner"] != "A*" or cfg.get("cost_weight", 0) <= 0:
            self.cost = None
            return None
        key = (self.grid.occ.shape, self.grid.occ.tobytes(), self.grid.res, cfg["cost_sigma"])
        if self.cost is None or key != self._cost_key:
            self.cost = smoothed_cost(self.grid.occ, self.grid.res, cfg["cost_sigma"])
            self._cost_key = key
        return self.cost

    @property
    def shortened(self):
        """Is the path driven different from the plan as planned?"""
        return self.path_mode != PATH_MODES[0]

    def set_path_mode(self, mode, cfg):
        """Make the path to drive from the plan: shortcut it and/or spline
        it, each checked against the map. If the spline collides even with
        its control points close together, the unsplined path is kept.

        Once the robot is on its way, only the part of the plan still ahead
        of it is smoothed, starting from where it is -- otherwise the new
        path would begin behind it and the follower would drive back."""
        if self.result is None or not self.result.path:
            return False
        chk = self.checker(cfg)
        path = self.result.path
        if self.t > 0 and len(path) > 1:
            plan = Path(path, name="plan")
            s = plan.closest(self.robot.x, self.robot.y)[2]
            ahead = [tuple(p) for p, sp in zip(plan.xy, plan.s) if sp > s + 1e-6]
            path = [(self.robot.x, self.robot.y)] + (ahead or [tuple(plan.xy[-1])])
        if "shortcut" in mode:
            path = shortcut(chk, path)
        self.path_note, self.spline_spacing = "", None
        if "spline" in mode:
            pts, used = spline(chk, path, cfg["spline_spacing"])
            if pts is None:
                self.path_note = "spline collides: kept the path without it"
            else:
                path, self.spline_spacing = pts, used
        self.path, self.path_mode = path, mode
        self._make_follower()
        return True

    @property
    def curvature(self):
        """Max curvature of a splined path [1/m] (a polyline's corners have
        infinite curvature, so None for those)."""
        return max_curvature(self.path) if self.spline_spacing else None

    def _make_follower(self):
        """A fresh Follower on the new path, starting from wherever the
        robot is now and keeping its current speed -- a replan must not
        stop the robot or send it back to the start."""
        r = self.robot
        v, w = r.v, r.w
        robot = Robot(*r.pose)
        self.follower = Follower(Path(self.path, name="plan"), robot, self.rng)
        robot.v, robot.w = v, w
        self.robot = robot

    # ---- driving ------------------------------------------------------------

    def path_blocked(self):
        """Does the rest of the path cross an occupied cell? Checked from
        the robot's position along the remaining waypoints."""
        f = self.follower
        pts = [(self.robot.x, self.robot.y)] + \
              [tuple(p) for p, s in zip(f.path.xy, f.path.s) if s > f.s]
        chk = GridChecker(self.grid, ignore={self.grid.cell(self.robot.x, self.robot.y)})
        return not all(chk.segment_free(a, b) for a, b in zip(pts, pts[1:]))

    def advance(self, duration, cfg):
        if not self.driving or self.done or self.collided or self.follower is None:
            return
        n = max(int(round(duration / self.CHECK_DT)), 1)
        for _ in range(n):
            self.follower.advance(self.CHECK_DT, cfg)
            self.t += self.CHECK_DT
            self.trail.append((self.robot.x, self.robot.y))

            if self.world.distance(np.array([self.robot.x, self.robot.y]))[0] < self.robot_radius:
                self.collided = True
                self.driving = False
                return
            if self.follower.done and abs(self.robot.v) < 1e-3:
                self.done = True
                self.driving = False
                return

            if self.lidar is not None and self.t >= self._next_sense - 1e-9:
                self._next_sense = self.t + self.SENSE_DT
                if self.sense() and self.path_blocked():
                    self.replans += 1
                    if not self.plan(cfg).path:
                        self.driving = False
                        self.failed = "no path in the map so far: " + self.result.message
                        return

    @property
    def driven(self):
        """Distance actually driven [m]."""
        return path_length(self.trail)
