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

from .grid import Grid, GridChecker, GeometryChecker
from .mapping import Lidar, MappedGrid
from .path import Path
from .planners import astar, rrt, rrtstar, shortcut, path_length
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

    def build_map(self, cfg):
        """(Re)make the grid for the current settings and reset the mission."""
        if cfg["mapped"]:
            self.grid = MappedGrid(self.world.bounds, cfg["res"], cfg["inflate"], cfg["raster"])
            self.lidar = Lidar(cfg["sensor_range"], cfg["rays"], cfg["noise"], self.rng)
        else:
            self.grid = Grid.from_world(self.world, cfg["res"], cfg["inflate"], cfg["raster"])
            self.lidar = None
        self.reset()

    def reset(self):
        """Robot back to the start, plan and history cleared; a mapped
        grid keeps nothing (a fresh start means a fresh map)."""
        if self.lidar is not None:
            self.grid = MappedGrid(self.world.bounds, self.grid.res, self.grid.inflate, self.grid.mode)
        self.robot = Robot(*self.start)
        self.follower = None
        self.result = None           # the latest PlanResult
        self.path = []               # the path being driven (after shortcutting)
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
        if cfg["exact_geometry"] and cfg["planner"] != "A*" and self.lidar is None:
            return GeometryChecker(self.world, cfg["inflate"])
        return GridChecker(self.grid, ignore={self.grid.cell(self.robot.x, self.robot.y)})

    def plan(self, cfg):
        """Plan from the robot's current position to the goal."""
        start = (self.robot.x, self.robot.y)
        checker = self.checker(cfg)
        p = cfg["planner"]
        if p == "A*":
            res = astar(self.grid, start, self.goal, cfg["eight"], cfg["h_weight"],
                        free=checker.free)
        elif p == "RRT":
            res = rrt(checker, self.world.bounds, start, self.goal, cfg["iterations"],
                      cfg["step"], cfg["goal_bias"], cfg["stop_at_goal"], self.rng)
        else:
            res = rrtstar(checker, self.world.bounds, start, self.goal, cfg["iterations"],
                          cfg["step"], cfg["goal_bias"], cfg["radius"], self.rng)
        self.result = res
        if self.path:
            self.old_paths.append(self.path)
        self.path = shortcut(checker, res.path) if (cfg["shortcut"] and res.path) else res.path
        self.failed = "" if res.path else res.message
        if self.path:
            self._make_follower()
        return res

    def _make_follower(self):
        """A fresh Follower on the new path, starting from wherever the
        robot is now and keeping its current speed -- a replan must not
        stop the robot or send it back to the start."""
        r = self.robot
        v, w = r.v, r.w
        robot = Robot(*r.pose)
        self.follower = Follower(Path(self.path, name="plan"), robot)
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
