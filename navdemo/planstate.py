"""Tunables and demo state for run_planning.py.

Kept separate from params.py (which belongs to run_pure_pursuit.py) so the
two demos can't affect each other; only the Tunable class is shared.
"""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from .params import Tunable
from .planners import PLANNERS
from .sim import CONTROL_LAWS

ROBOT_RADIUS = 0.2   # m, as in run_pure_pursuit.py / display_robot.m

_RES = [0.05, 0.1, 0.2, 0.25, 0.5, 1.0]
_INFLATE = [round(0.02 * i, 2) for i in range(26)]      # 0 to 0.5 m in 2 cm steps
_IMPERFECT = [0.0, 0.005, 0.01, 0.02, 0.05]
_HWEIGHT = [0.0, 0.5, 1.0, 2.0, 5.0]
_ITER = [100, 250, 500, 1000, 2000, 5000, 10000]
_STEP = [0.1, 0.25, 0.5, 1.0, 2.0]
_BIAS = [0.0, 0.05, 0.1, 0.2, 0.5]
_RADIUS = [0.5, 1.0, 1.5, 2.0, 3.0]
_LOOKAHEAD = [0.1, 0.2, 0.3, 0.5, 0.75, 1.0]
_VMAX = [0.25, 0.5, 1.0, 1.5, 2.0]
_KP = [1.0, 2.0, 5.0, 10.0, 20.0]
_TIME_SCALE = [0.25, 0.5, 1.0, 2.0, 4.0]
_ANIM = [0.0, 1.0, 3.0, 10.0]
_SENSOR_RANGE = [1.0, 1.5, 2.0, 3.0, 5.0, 10.0]
_RAYS = [8, 16, 36, 90, 180, 360]
_NOISE = [0.0, 0.01, 0.02, 0.05, 0.1, 0.2]

class CountTunable(Tunable):
    """A whole-number tunable: 2000, not 2e+03."""

    def format(self, value):
        return f"{int(value)}"


TUNABLES: List[Tunable] = [
    # the grid. Inflation defaults to the robot radius plus a 0.1 m margin:
    # with exactly the radius there is no room left for tracking error
    # (pure pursuit cuts corners), and the robot clips obstacles.
    Tunable("res", "cell", _RES, _RES.index(0.1), "m"),
    Tunable("inflate", "inflate", _INFLATE, _INFLATE.index(0.3), "m"),
    # the world as built, not as drawn (see world.imperfect); 0 = as drawn
    Tunable("imperfect", "imperfect", _IMPERFECT, _IMPERFECT.index(0.02), "m"),
    # the sensor, when mapping as we go
    Tunable("sensor_range", "lidar_rng", _SENSOR_RANGE, _SENSOR_RANGE.index(3.0), "m"),
    CountTunable("rays", "rays", _RAYS, _RAYS.index(180)),
    Tunable("noise", "sig_rng", _NOISE, 0, "m"),
    # the planners
    Tunable("h_weight", "h_weight", _HWEIGHT, _HWEIGHT.index(1.0)),
    CountTunable("iterations", "iters", _ITER, _ITER.index(2000)),
    Tunable("step", "step", _STEP, _STEP.index(0.5), "m"),
    Tunable("goal_bias", "goal_bias", _BIAS, _BIAS.index(0.05)),
    Tunable("radius", "radius", _RADIUS, _RADIUS.index(1.0), "m"),
    Tunable("anim", "replay", _ANIM, _ANIM.index(3.0), "s"),
    # execution (pure pursuit)
    Tunable("lookahead", "lookahd", _LOOKAHEAD, _LOOKAHEAD.index(0.3), "m"),
    Tunable("vmax", "v_max", _VMAX, _VMAX.index(1.0), "m/s"),
    Tunable("kp", "kP", _KP, _KP.index(10.0), "/s"),
    Tunable("time_scale", "speed", _TIME_SCALE, _TIME_SCALE.index(1.0), "x"),
]
TUNABLE_BY_NAME = {t.name: t for t in TUNABLES}

# Which planner each planner parameter belongs to; anything not listed is
# always shown. Rows for other planners are hidden and skipped by tab.
_ONLY_FOR = {
    "h_weight": {"A*"},
    "iterations": {"RRT", "RRT*"},
    "step": {"RRT", "RRT*"},
    "goal_bias": {"RRT", "RRT*"},
    "radius": {"RRT*"},
}

# Fixed pure pursuit settings (the defaults of run_pure_pursuit.py) that
# aren't worth a row here.
FIXED_CONTROL = dict(sigma=np.deg2rad(60), accv=2.0, accw=np.deg2rad(3600), ctrl_dt=0.01)


@dataclass
class PlanState:
    running: bool = True
    mapped: bool = False               # build the map as we go (else: known map)
    planner: str = PLANNERS[0]
    eight: bool = True                 # A*: 8- (else 4-) connectivity
    exact_geometry: bool = False       # RRT/RRT*: check the real geometry, not the grid
    stop_at_goal: bool = True          # RRT: stop at the first path found
    shortcut: bool = False
    law: str = CONTROL_LAWS[0]
    turn_in_place: bool = True         # pure pursuit law: turn on the spot when the target is behind

    show_grid: bool = True
    show_geometry: bool = True
    show_search: bool = True
    show_lookahead: bool = True
    show_trail: bool = True

    # one-shot requests
    world_idx: int = 0
    variant: int = 0                   # bumped by 'v' for another imperfect building
    newWorld: object = None            # an int (1..9 key) or a World
    plan: bool = False
    drive: bool = False                # space: start / pause execution
    reset: bool = False
    newStart: object = None
    newGoal: object = None

    idx: dict = field(default_factory=lambda: {t.name: t.default for t in TUNABLES})
    cursor: int = 0

    def value(self, name):
        return TUNABLE_BY_NAME[name].ladder[self.idx[name]]

    def step(self, name, delta):
        t = TUNABLE_BY_NAME[name]
        self.idx[name] = int(np.clip(self.idx[name] + delta, 0, len(t.ladder) - 1))

    def set_value(self, name, value):
        ladder = np.array(TUNABLE_BY_NAME[name].ladder, dtype=float)
        self.idx[name] = int(np.argmin(np.abs(ladder - value)))

    def relevant(self, t):
        if t.name == "kp":
            return self.law == "heading-P"
        if t.name in ("sensor_range", "rays", "noise"):
            return self.mapped
        return self.planner in _ONLY_FOR.get(t.name, {self.planner})

    def visible(self):
        return [i for i, t in enumerate(TUNABLES) if self.relevant(t)]

    @property
    def selected(self):
        return TUNABLES[self.cursor]

    @property
    def uses_exact_geometry(self):
        """Is the planner checking the real geometry rather than the grid?
        Only RRT/RRT* can (A* needs cells), and only with the known map (when
        mapping, the grid is all the robot has)."""
        return self.exact_geometry and self.planner != "A*" and not self.mapped

    def checks_text(self):
        if self.uses_exact_geometry:
            return "exact geometry"
        if self.planner == "A*":
            return "grid (A* needs cells)"
        if self.mapped:
            return "grid (the map is all we have)"
        return "grid"

    def world_key(self):
        """Everything the building depends on (besides which world it is)."""
        return (self.value("imperfect"), self.variant)

    def grid_key(self):
        """Everything the map depends on: a change means starting over."""
        key = (self.mapped, self.value("res"), self.value("inflate"))
        if self.mapped:
            key += (self.value("sensor_range"), self.value("rays"), self.value("noise"))
        return key

    def plan_key(self):
        """Everything the current plan depends on: a change marks it stale."""
        names = [t.name for t in TUNABLES if t.name in _ONLY_FOR and self.relevant(t)]
        return (self.planner, self.eight, self.uses_exact_geometry, self.stop_at_goal,
                self.shortcut) + tuple(self.value(n) for n in names)

    def mission_config(self):
        """Everything Mission and Follower need, as one dict."""
        cfg = dict(FIXED_CONTROL, law=self.law, mapped=self.mapped,
                   planner=self.planner, eight=self.eight,
                   exact_geometry=self.uses_exact_geometry,
                   stop_at_goal=self.stop_at_goal, shortcut=self.shortcut,
                   turn_in_place=self.turn_in_place)
        cfg.update({t.name: self.value(t.name) for t in TUNABLES})
        cfg["rays"] = int(cfg["rays"])
        cfg["iterations"] = int(cfg["iterations"])
        return cfg
