"""Keyboard and mouse control.

The same conventions as the localization demos: tab / shift-tab select a
parameter, > / < step it along its ladder, h prints the key list, S saves
a screenshot, q quits. The mouse draws a new path (samplepath.m in the
MATLAB demo).
"""

import time

import matplotlib as mpl
import numpy as np

from . import app
from .params import DemoState, TUNABLES
from .path import Path
from .sim import CONTROL_LAWS

# See locdemo/keys.py: drop OS key auto-repeat for everything except the
# keys where repeat-while-held is the point.
_MIN_REPEAT_INTERVAL = 0.15
_CONTINUOUS_KEYS = {">", "<"}

HELP = """
 simulation             display                 parameters
 ----------             -------                 ----------
 space  pause / run     g  lookahead geometry   tab   select next parameter
 r      reset (restart) t  driven trail         S-tab select previous
 d      disturb robot   S  screenshot (2 pngs)  >     increase selected
 c      control law:                            <     decrease selected
        heading-P / pure pursuit
 1..4   built-in path   mouse: drag in the plot to draw a new path
 w      save the path   (the robot restarts at its first point)
        to paths/
 h      this help
 q      quit
"""


def clear_default_keymap():
    """Stop matplotlib's own shortcuts ('s' save, 'q' close, 'g' grid, ...)
    from stealing our keys."""
    for k in list(mpl.rcParams):
        if k.startswith("keymap."):
            mpl.rcParams[k] = []


def make_handler(state: DemoState, fig=None, ax=None, demo="purepursuit"):
    last_press = {}
    held = set()

    def release(event):
        held.discard(event.key)

    def handler(event):
        k = event.key
        if k is None:
            return
        if k not in _CONTINUOUS_KEYS:
            if k in held:
                return
            held.add(k)
        now = time.monotonic()
        if now - last_press.get(k, -1.0) < _MIN_REPEAT_INTERVAL:
            return
        last_press[k] = now

        if k == " ":
            state.paused = not state.paused
        elif k == "r":
            state.reset = True
        elif k == "d":
            state.disturb = True
        elif k == "c":
            i = (CONTROL_LAWS.index(state.law) + 1) % len(CONTROL_LAWS)
            state.law = CONTROL_LAWS[i]
        elif k in "1234":
            state.newPath = int(k) - 1
        elif k == "w":
            state.savePath = True
        elif k == "g":
            state.showGeometry = not state.showGeometry
        elif k == "t":
            state.showTrail = not state.showTrail
        elif k == "tab":
            state.cursor = (state.cursor + 1) % len(TUNABLES)
        elif "tab" in k.lower():
            # shift-tab's name is backend dependent ("shift+tab", "backtab")
            state.cursor = (state.cursor - 1) % len(TUNABLES)
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


class PathDrawer:
    """Left-drag in the plot to draw a new path.

    Points closer than MIN_STEP to the previous one are dropped, as in
    samplepath.m, so a slow drag doesn't give a pile of tiny segments.
    """

    MIN_STEP = 0.02  # m

    def __init__(self, fig, ax, state: DemoState):
        self.ax, self.state = ax, state
        self.pts = None
        (self.preview,) = ax.plot([], [], color="0.5", lw=1.5, zorder=8)
        fig.canvas.mpl_connect("button_press_event", self.press)
        fig.canvas.mpl_connect("motion_notify_event", self.move)
        fig.canvas.mpl_connect("button_release_event", self.release)

    def press(self, event):
        if event.inaxes is self.ax and event.button == 1 and event.xdata is not None:
            self.pts = [(event.xdata, event.ydata)]

    def move(self, event):
        if self.pts is None or event.inaxes is not self.ax or event.xdata is None:
            return
        if np.hypot(event.xdata - self.pts[-1][0], event.ydata - self.pts[-1][1]) >= self.MIN_STEP:
            self.pts.append((event.xdata, event.ydata))
            self.preview.set_data(*zip(*self.pts))

    def release(self, event):
        if self.pts is None:
            return
        pts, self.pts = self.pts, None
        self.preview.set_data([], [])
        if len(pts) >= 3:
            self.state.newPath = Path(pts, name="drawn")

    @property
    def artists(self):
        return [self.preview]


def connect(fig, state, ax=None, demo="purepursuit"):
    clear_default_keymap()
    handler, release = make_handler(state, fig, ax, demo)
    fig.canvas.mpl_connect("key_press_event", handler)
    fig.canvas.mpl_connect("key_release_event", release)
    return PathDrawer(fig, ax, state) if ax is not None else None
