"""Tunable parameters and mutable demo state.

As in the localization demos, every tunable steps along an explicit ladder
of values rather than a slider: explicit numbers are easier to talk about
in a lecture and make an experiment reproducible. There is only one column
here -- the controller has no model of the world to get wrong, it just has
settings.
"""

from dataclasses import dataclass, field
from typing import List

import numpy as np

from .sim import CONTROL_LAWS


@dataclass(frozen=True)
class Tunable:
    name: str
    label: str
    ladder: List[float]
    default: int
    unit: str = ""    # shown after the number; 'deg' and 'deg/s2' are stored in rad
    law: str = None   # only relevant for this control law (None = always)

    def format(self, value):
        if np.isinf(value):
            return "∞" if self.name != "sigma" else "off"
        if self.unit == "deg":
            return f"{np.rad2deg(value):.4g}°"
        if self.unit == "deg/s2":
            return f"{np.rad2deg(value):.4g}°/s²"
        return f"{value:.3g}{self.unit}"


def _deg(values):
    return [np.deg2rad(v) for v in values]


_LOOKAHEAD = [0.02, 0.05, 0.1, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5]
_VMAX = [0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
_KP = [0.5, 1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0]
_SIGMA_DEG = [10, 20, 30, 60, 90]
_ACCV = [0.25, 0.5, 1.0, 2.0, 5.0, np.inf]
_ACCW_DEG = [90, 180, 360, 720, 1800, 3600]
_CTRL_DT = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5]
_TIME_SCALE = [0.1, 0.25, 0.5, 1.0, 2.0, 4.0]

# Display order is also the order tab walks through.
TUNABLES: List[Tunable] = [
    Tunable("lookahead", "lookahd", _LOOKAHEAD, _LOOKAHEAD.index(0.1), "m"),
    Tunable("vmax", "v_max", _VMAX, _VMAX.index(1.0), "m/s"),
    Tunable("kp", "kP", _KP, _KP.index(10.0), "/s", law="heading-P"),
    # Speed reduction with heading error; 'off' = always drive at v_max.
    Tunable("sigma", "slowdn", _deg(_SIGMA_DEG) + [np.inf], _SIGMA_DEG.index(60), "deg"),
    Tunable("accv", "acc_v", _ACCV, _ACCV.index(2.0), "m/s²"),
    Tunable("accw", "acc_w", _deg(_ACCW_DEG) + [np.inf], _ACCW_DEG.index(3600), "deg/s2"),
    Tunable("ctrl_dt", "ctrl_dt", _CTRL_DT, _CTRL_DT.index(0.01), "s"),
    Tunable("time_scale", "speed", _TIME_SCALE, _TIME_SCALE.index(1.0), "x"),
]
TUNABLE_BY_NAME = {t.name: t for t in TUNABLES}


@dataclass
class DemoState:
    """Everything the keyboard can change, plus one-shot request flags that
    the main loop polls and clears."""

    running: bool = True
    paused: bool = False
    law: str = CONTROL_LAWS[0]
    turnInPlace: bool = False   # pure pursuit law: turn on the spot when the target is behind
    showGeometry: bool = True
    showTrail: bool = True
    path_idx: int = 0

    # one-shot requests
    reset: bool = False
    disturb: bool = False
    newPath: object = None   # a Path from the mouse, or an int for 1..4
    savePath: bool = False

    idx: dict = field(default_factory=lambda: {t.name: t.default for t in TUNABLES})
    cursor: int = 0

    def value(self, name):
        t = TUNABLE_BY_NAME[name]
        return t.ladder[self.idx[name]]

    def step(self, name, delta):
        t = TUNABLE_BY_NAME[name]
        self.idx[name] = int(np.clip(self.idx[name] + delta, 0, len(t.ladder) - 1))

    def set_value(self, name, value):
        """Snap to the ladder entry closest to `value` (radians for angles)."""
        t = TUNABLE_BY_NAME[name]
        ladder = np.array(t.ladder)
        if np.isinf(value):
            self.idx[name] = int(np.argmax(np.isinf(ladder))) if np.isinf(ladder).any() \
                else len(ladder) - 1
            return
        finite = np.where(np.isinf(ladder), np.nan, ladder)
        self.idx[name] = int(np.nanargmin(np.abs(finite - value)))

    @property
    def selected(self):
        return TUNABLES[self.cursor]

    def controller_params(self):
        """The dict Follower.advance/control expect."""
        p = {t.name: self.value(t.name) for t in TUNABLES}
        p["law"] = self.law
        p["turn_in_place"] = self.turnInPlace
        return p
