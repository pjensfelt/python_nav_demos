"""Tunables, state and keys for run_grid.py.

Both the grid and the world have a pose (a shift and a rotation about the
world's centre). Only the pose of one relative to the other decides the
cells, but moving the *grid* over a fixed world is the more honest
picture -- the building is what it is, the map frame is what you happen to
choose -- so that is what the keys move by default ('w' switches).
"""

import time
from dataclasses import dataclass, field
from typing import List

import numpy as np

from . import app
from .keys import clear_default_keymap, _MIN_REPEAT_INTERVAL
from .params import Tunable
from .rasterize import INFLATE_ORDERS, RULES

# Down to 1 cm, so the cost of resolution shows: at 1 cm a 10 x 10 m world
# is a million cells and every key press takes a couple of seconds.
_CELL = [0.01, 0.02, 0.05, 0.1, 0.2, 0.25, 0.3, 0.4, 0.5, 0.75, 1.0]
_INFLATE = [round(0.02 * i, 2) for i in range(26)]      # 0 to 0.5 m in 2 cm steps
_SPACING = [0.01, 0.02, 0.05, 0.1, 0.2, 0.3, 0.5]
_IMPERFECT = [0.0, 0.005, 0.01, 0.02, 0.05]
_SIGMA = [0.0, 0.01, 0.02, 0.05, 0.1, 0.2]
_MIN_HITS = [1, 2, 3, 5, 10]


class CountTunable(Tunable):
    def format(self, value):
        return f"{int(value)}"

TUNABLES: List[Tunable] = [
    # Starts fine, so the grid first looks like the world; coarsen it with
    # '>' to watch it fall apart.
    Tunable("res", "cell", _CELL, _CELL.index(0.1), "m"),
    # 0 by default: the demo starts as pure world -> cells; inflation (and
    # the much stronger door-closing effect) is one '>' press away.
    Tunable("inflate", "inflate", _INFLATE, 0, "m"),
    # the world as built, not as drawn: corners moved by this much (std),
    # so nothing lines up with the cell lattice by accident; 0 = as drawn
    Tunable("imperfect", "imperfect", _IMPERFECT, _IMPERFECT.index(0.02), "m"),
    # the "samples" rule only: points along the outlines
    Tunable("spacing", "spacing", _SPACING, _SPACING.index(0.05), "m"),
    Tunable("sigma", "noise", _SIGMA, 0, "m"),
    CountTunable("min_hits", "min_hits", _MIN_HITS, 0),
]
SAMPLE_ONLY = {"spacing", "sigma", "min_hits"}
TUNABLE_BY_NAME = {t.name: t for t in TUNABLES}

SWEEP_STEPS = 40                   # frames for each half of a sweep
SWEEP_ANGLE = np.deg2rad(45)       # a rotation sweep goes 0 -> 45 deg and back
TARGETS = ("grid", "world")


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


@dataclass
class Pose:
    offset: np.ndarray = field(default_factory=lambda: np.zeros(2))   # m
    angle: float = 0.0                                                  # rad, about the world centre

    def copy(self):
        return Pose(self.offset.copy(), self.angle)


@dataclass
class GridState:
    running: bool = True
    rule: str = RULES[0]
    # Grid first by default: what a real map (from sensor data, a costmap)
    # has to do, and the safe choice -- any inflation adds at least a ring of
    # cells. World first (needs a model) is one 'i' away, to compare.
    inflate_order: str = INFLATE_ORDERS[1]
    grid: Pose = field(default_factory=Pose)
    world: Pose = field(default_factory=Pose)
    moving: str = TARGETS[0]                     # what the keys and the mouse move

    show_grid: bool = True
    show_geometry: bool = True
    show_reach: bool = False
    show_samples: bool = True
    # Planning between the probes: off at the start -- the demo is about the
    # grid; planning on it is something to turn on once the grid is understood
    planning: bool = False
    sample_draw: int = 0          # bumped by 'u' for a fresh set of samples
    variant: int = 0              # bumped by 'v' for another imperfect building
    # Clicked start / goal for the first probe, in the world file's own
    # coordinates (so they move with the world); None = as designed.
    start: object = None
    goal: object = None
    newStart: object = None       # clicks, in the drawing frame, not yet converted
    newGoal: object = None

    # one-shot requests
    newWorld: object = None
    sweep_kind: object = None     # "x", "y" or "r" while sweeping
    sweep_k: int = 0
    sweep_base: Pose = None
    sweep_log: list = field(default_factory=list)   # (u, occupied %, [open per probe], forward?)
    last_sweep_kind: str = "x"

    idx: dict = field(default_factory=lambda: {t.name: t.default for t in TUNABLES})
    cursor: int = 0

    # ---- tunables ---------------------------------------------------------

    def value(self, name):
        return TUNABLE_BY_NAME[name].ladder[self.idx[name]]

    def step(self, name, delta):
        t = TUNABLE_BY_NAME[name]
        self.idx[name] = int(np.clip(self.idx[name] + delta, 0, len(t.ladder) - 1))

    def set_value(self, name, value):
        ladder = np.array(TUNABLE_BY_NAME[name].ladder, dtype=float)
        self.idx[name] = int(np.argmin(np.abs(ladder - value)))

    @property
    def selected(self):
        return TUNABLES[self.cursor]

    def visible(self):
        """Rows that apply to the current rule (tab only visits these)."""
        return [i for i, t in enumerate(TUNABLES)
                if t.name not in SAMPLE_ONLY or self.rule == "samples"]

    @property
    def effective_inflate_order(self):
        """With samples there is no geometric model: inflate in the grid."""
        return "grid first" if self.rule == "samples" else self.inflate_order

    # ---- poses ------------------------------------------------------------

    @property
    def target(self):
        return self.grid if self.moving == "grid" else self.world

    def relative(self):
        """The world's pose in the grid's frame, as (dx, dy, angle) for
        transform_obstacles: the only thing the cells depend on.

        A point p of the world file sits at  R_w (p - c) + c + t_w  in the
        world frame, and a world point x at  R_g^T (x - c - t_g) + c  in the
        grid frame; composed, that is  R(a_w - a_g) (p - c) + c + R_g^T (t_w - t_g).
        """
        d = rot(self.grid.angle).T @ (self.world.offset - self.grid.offset)
        return (float(d[0]), float(d[1]), float(self.world.angle - self.grid.angle))

    def reset_poses(self):
        self.grid, self.world = Pose(), Pose()

    def reset(self):
        """'0': back to the designed setup -- the poses, and the start and
        goal the world was built to demo something with."""
        self.reset_poses()
        self.start = self.goal = None

    # ---- sweeps -----------------------------------------------------------

    def start_sweep(self, kind):
        self.sweep_kind = kind
        self.last_sweep_kind = kind
        self.sweep_k = 0
        self.sweep_base = self.target.copy()
        self.sweep_log = []

    def sweep_u(self):
        """Where in the sweep we are: 0 -> 1 and back to 0."""
        k = self.sweep_k
        return k / SWEEP_STEPS if k <= SWEEP_STEPS else 2 - k / SWEEP_STEPS

    def apply_sweep(self, u):
        """Set the moving thing's pose for sweep position u. Shifts go one
        cell along the *grid's* axes (so the grid really does come back to
        itself after one cell), rotations 0 -> 45 degrees."""
        t = self.target
        if self.sweep_kind == "r":
            t.angle = self.sweep_base.angle + u * SWEEP_ANGLE
        else:
            e = np.array([1.0, 0.0]) if self.sweep_kind == "x" else np.array([0.0, 1.0])
            t.offset = self.sweep_base.offset + u * self.value("res") * (rot(self.grid.angle) @ e)


HELP = """
 moving                           cells                         display
 ------                           -----                         -------
 w            move: grid / world  m  rule: any overlap /        g  grid
 arrows       shift 1/10 cell        samples                    o  real obstacles
 shift+arrows shift 1/50 cell     i  inflate: world first /     d  sample points
 , / .        rotate -1 / +1 deg     grid first                 f  cells the planner
 k / l        rotate -5 / +5 deg                                 expanded
 mouse drag   shift                  corners                    S  screenshot
 x / y        sweep one cell      u  samples: a fresh set       h  help, q quit
              along the grid's    tab / S-tab  select
 r            sweep the rotation  > / <        change it
              0 -> 45 deg
 0            back to the start (poses, and the designed start and goal)
 1..9         world               p  planning between the probe points on / off
 v            another variant of the (imperfect) building
"""


def make_handler(state: GridState, fig=None, ax=None, demo="grid"):
    last_press = {}
    held = set()
    continuous = {"left", "right", "up", "down", "shift+left", "shift+right",
                  "shift+up", "shift+down", ",", ".", "k", "l", "[", "]", "{", "}", ">", "<"}

    def release(event):
        held.discard(event.key)

    def handler(event):
        k = event.key
        if k is None:
            return
        if k not in continuous:
            if k in held:
                return
            held.add(k)
            now = time.monotonic()
            if now - last_press.get(k, -1.0) < _MIN_REPEAT_INTERVAL:
                return
            last_press[k] = now

        res = state.value("res")
        moves = {"left": (-1, 0), "right": (1, 0), "up": (0, 1), "down": (0, -1)}
        t = state.target
        if k in moves:
            t.offset = t.offset + np.array(moves[k]) * res / 10
        elif k.startswith("shift+") and k[6:] in moves:
            t.offset = t.offset + np.array(moves[k[6:]]) * res / 50
        elif k in (",", ".", "k", "l", "[", "]", "{", "}"):
            # , . (1 deg) and k l right above them (5 deg) are single
            # unshifted keys on most layouts -- brackets need Option on a
            # Swedish Mac. Hold one down to keep rotating.
            t.angle += np.deg2rad({",": -1, ".": 1, "k": -5, "l": 5,
                                   "[": -1, "]": 1, "{": -5, "}": 5}[k])
        elif k == "w":
            state.moving = TARGETS[(TARGETS.index(state.moving) + 1) % len(TARGETS)]
        elif k == "0":
            state.reset()
        elif k in ("x", "y", "r"):
            state.start_sweep(k)
        elif k == "m":
            state.rule = RULES[(RULES.index(state.rule) + 1) % len(RULES)]
            if state.cursor not in state.visible():     # its row just got hidden
                state.cursor = state.visible()[0]
        elif k == "i":
            state.inflate_order = INFLATE_ORDERS[
                (INFLATE_ORDERS.index(state.inflate_order) + 1) % len(INFLATE_ORDERS)]
        elif k in "123456789" and len(k) == 1:
            state.newWorld = int(k) - 1
        elif k == "g":
            state.show_grid = not state.show_grid
        elif k == "o":
            state.show_geometry = not state.show_geometry
        elif k == "f":
            state.show_reach = not state.show_reach
        elif k == "p":
            state.planning = not state.planning
        elif k == "v":
            state.variant += 1
        elif k == "u":
            state.sample_draw += 1
        elif k == "d":
            state.show_samples = not state.show_samples
        elif k == "tab" or "tab" in k.lower():
            vis = state.visible()
            pos = vis.index(state.cursor) if state.cursor in vis else -1
            state.cursor = vis[(pos + (1 if k == "tab" else -1)) % len(vis)]
        elif k == ">":
            state.step(state.selected.name, +1)
        elif k == "<":
            state.step(state.selected.name, -1)
        elif k == "h":
            print(HELP)
        elif k == "S" and ax is not None:
            app.save_screenshot(fig, ax, demo)
        elif k == "q":
            state.running = False
            if fig is not None:
                import matplotlib.pyplot as plt
                plt.close(fig)

    return handler, release


def connect(fig, state, ax, demo="grid"):
    clear_default_keymap()
    handler, release = make_handler(state, fig, ax, demo)
    fig.canvas.mpl_connect("key_press_event", handler)
    fig.canvas.mpl_connect("key_release_event", release)

    # Left drag moves the grid (or the world). With planning on, a left
    # *click* -- released within a few pixels of where it was pressed --
    # sets the goal instead, and a right click the start; with planning off
    # there is nothing to plan between, so clicks do nothing.
    CLICK_PIXELS = 4
    drag = {}

    def press(event):
        if event.inaxes is not ax or event.xdata is None:
            return
        if event.button == 1:
            drag["from"] = np.array([event.xdata, event.ydata])
            drag["pixels"] = np.array([event.x, event.y])
            drag["offset"] = state.target.offset.copy()
        elif event.button == 3 and state.planning:
            state.newStart = (event.xdata, event.ydata)

    def move(event):
        if "from" in drag and event.inaxes is ax and event.xdata is not None:
            state.target.offset = drag["offset"] + np.array([event.xdata, event.ydata]) - drag["from"]

    def up(event):
        if "from" in drag and event.x is not None and \
                np.hypot(*(np.array([event.x, event.y]) - drag["pixels"])) < CLICK_PIXELS:
            state.target.offset = drag["offset"]          # it was a click, not a drag
            if state.planning:
                state.newGoal = tuple(drag["from"])
        drag.clear()

    fig.canvas.mpl_connect("button_press_event", press)
    fig.canvas.mpl_connect("motion_notify_event", move)
    fig.canvas.mpl_connect("button_release_event", up)
