"""Tunables and demo state for run_avoid.py (the obstacle avoidance demo).

Separate from the other demos' state, as planstate.py is; only the Tunable
class is shared.
"""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from .avoid import GOAL_MODES, METHODS
from .params import Tunable
from .planstate import CountTunable

ROBOT_RADIUS = 0.2   # m, as in the other demos


def _deg(values):
    return [np.deg2rad(v) for v in values]


_VMAX = [0.25, 0.5, 0.75, 1.0, 1.5]
_ACCV = [0.25, 0.5, 1.0, 2.0]
_ACCW = [45, 90, 180, 360, 720]
_WMAX = [30, 60, 90, 180]
_RANGE = [1.0, 2.0, 3.0, 5.0]
_RAYS = [36, 90, 180, 360]
_PEOPLE = [0.0, 0.5, 1.0, 1.5, 2.0]
_CARROT = [0.5, 1.0, 1.5, 2.0, 3.0]
_K_REP = [0.01, 0.03, 0.1, 0.3, 1.0]
_D0 = [0.3, 0.5, 0.75, 1.0, 1.5]
_HORIZON = [0.5, 1.0, 2.0, 3.0, 5.0]
_W = [0.0, 0.25, 0.5, 1.0, 2.0, 5.0]
_VFH_WIN = [1.0, 1.5, 2.0, 3.0]
_VFH_THR = [0.25, 0.5, 1.0, 2.0, 4.0]
_TIME_SCALE = [0.25, 0.5, 1.0, 2.0, 4.0]
_FOV = [60, 90, 180, 270, 360]
_HALF_LIFE = [0.5, 1.0, 2.0, 5.0, 10.0, 30.0]

TUNABLES: List[Tunable] = [
    # the robot: the same limits whatever the method -- DWA plans with
    # them, the others just meet them
    Tunable("vmax", "v_max", _VMAX, _VMAX.index(0.75), "m/s"),
    Tunable("wmax", "w_max", _deg(_WMAX), _WMAX.index(90), "deg/s"),
    Tunable("accv", "acc_v", _ACCV, _ACCV.index(1.0), "m/s²"),
    Tunable("accw", "acc_w", _deg(_ACCW), _ACCW.index(180), "deg/s2"),
    # the sensor
    Tunable("sensor_range", "lidar_rng", _RANGE, _RANGE.index(3.0), "m"),
    CountTunable("rays", "rays", _RAYS, _RAYS.index(180)),
    # the sensor's field of view, centred on the heading (360 = all round)
    Tunable("fov", "fov", _deg(_FOV), _FOV.index(360), "deg"),
    # the local map ('l'), when it forgets ('f'): the half-life of a cell
    Tunable("half_life", "forget", _HALF_LIFE, _HALF_LIFE.index(5.0), "s"),
    # the world: how fast the people walk (0 = they stand still)
    Tunable("people", "people", _PEOPLE, _PEOPLE.index(1.0), "x"),
    # the A* carrot: how far ahead along the global path
    Tunable("carrot", "carrot", _CARROT, _CARROT.index(1.0), "m"),
    # potential field
    Tunable("k_rep", "k_rep", _K_REP, _K_REP.index(0.1)),
    Tunable("d0", "d0", _D0, _D0.index(0.75), "m"),
    # DWA: how far ahead each arc is predicted, and the score's weights
    Tunable("horizon", "horizon", _HORIZON, _HORIZON.index(2.0), "s"),
    Tunable("w_head", "w_head", _W, _W.index(1.0)),
    Tunable("w_clear", "w_clear", _W, _W.index(0.5)),
    Tunable("w_vel", "w_vel", _W, _W.index(0.5)),
    # VFH: the window it builds its histogram in, and the free threshold
    Tunable("vfh_win", "vfh_win", _VFH_WIN, _VFH_WIN.index(2.0), "m"),
    Tunable("vfh_thr", "vfh_thr", _VFH_THR, _VFH_THR.index(2.0)),
    Tunable("time_scale", "speed", _TIME_SCALE, _TIME_SCALE.index(1.0), "x"),
]
TUNABLE_BY_NAME = {t.name: t for t in TUNABLES}

_ONLY_FOR = {
    "k_rep": {"potential field"}, "d0": {"potential field"},
    "horizon": {"DWA"}, "w_head": {"DWA"}, "w_clear": {"DWA"}, "w_vel": {"DWA"},
    "vfh_win": {"VFH"}, "vfh_thr": {"VFH"},
}

# The global planner's map (A* carrot mode), fixed: 0.1 m cells, inflated
# by the robot radius plus 0.1 m.
FIXED = dict(res=0.1, inflate=ROBOT_RADIUS + 0.1)


@dataclass
class AvoidState:
    running: bool = True
    method: str = METHODS[0]
    goal_mode: str = GOAL_MODES[0]
    polite: bool = True             # people wait for the robot (else they walk blindly)
    people_on: bool = True          # 'p': people in the world at all
    # the local map: on/off ('l'), clear along rays or only add hits ('u'),
    # forget (fade) or not ('f')
    local_map: bool = False
    map_clear: bool = False
    map_forget: bool = False

    show_geometry: bool = True
    show_scan: bool = True
    show_method: bool = True        # the method's own picture (forces, arcs, histogram)
    show_trail: bool = True
    show_map: bool = True           # the global planner's grid (A* carrot mode)

    # one-shot requests
    newWorld: object = None
    newGoal: object = None
    newStart: object = None
    drive: bool = False
    reset: bool = False
    replan: bool = False

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
        if t.name == "half_life":
            return self.local_map and self.map_forget
        if t.name == "carrot":
            return self.goal_mode == GOAL_MODES[1]
        return self.method in _ONLY_FOR.get(t.name, {self.method})

    def visible(self):
        return [i for i, t in enumerate(TUNABLES) if self.relevant(t)]

    @property
    def selected(self):
        return TUNABLES[self.cursor]

    def config(self):
        cfg = dict(FIXED, method=self.method, goal_mode=self.goal_mode, polite=self.polite,
                   people_on=self.people_on, local_map=self.local_map,
                   map_clear=self.map_clear, map_forget=self.map_forget)
        cfg.update({t.name: self.value(t.name) for t in TUNABLES})
        cfg["rays"] = int(cfg["rays"])
        return cfg
