"""Keyboard and mouse control for run_avoid.py.

Same conventions as the other demos (tab / > / <, h, S, q, space, r,
number keys for worlds, c for the method as for the control law).
"""

import time

from . import app
from .avoid import GOAL_MODES, METHODS
from .avoidstate import AvoidState, TUNABLES
from .keys import clear_default_keymap, _MIN_REPEAT_INTERVAL

_CONTINUOUS_KEYS = {">", "<"}

HELP = """
 driving                   the goal                 display
 -------                   --------                 -------
 space  drive / pause      m  clicked goal / A*     o  obstacles
 r      reset                 carrot along a path   d  lidar hits
 c      method: VFH /      mouse  left: goal        a  the method's picture
        potential f. / DWA        right: start      t  driven trail
 p      people on / off    1..4  world              g  the map (A* carrot)
 w      people wait for    tab / S-tab  select,     S  screenshot (2 pngs)
        the robot / walk   > / <  change it         h  this help
        blindly                                     q  quit

 local map
 ---------
 l      on / off           u  add hits only /       f  forget (fade) on / off
                              also clear along rays
"""


def _cycle(options, current):
    return options[(options.index(current) + 1) % len(options)]


def make_handler(state: AvoidState, fig=None, ax=None, demo="avoid"):
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
            state.drive = True
        elif k == "r":
            state.reset = True
        elif k in "123456789":
            state.newWorld = int(k) - 1
        elif k == "c":
            state.method = _cycle(METHODS, state.method)
            if not state.relevant(state.selected):
                state.cursor = state.visible()[0]
        elif k == "m":
            state.goal_mode = _cycle(GOAL_MODES, state.goal_mode)
            state.replan = True
            if not state.relevant(state.selected):
                state.cursor = state.visible()[0]
        elif k == "w":
            state.polite = not state.polite
        elif k == "p":
            state.people_on = not state.people_on
        elif k == "l":
            state.local_map = not state.local_map
        elif k == "u":
            state.map_clear = not state.map_clear
        elif k == "f":
            state.map_forget = not state.map_forget
            if not state.relevant(state.selected):
                state.cursor = state.visible()[0]
        elif k == "o":
            state.show_geometry = not state.show_geometry
        elif k == "d":
            state.show_scan = not state.show_scan
        elif k == "a":
            state.show_method = not state.show_method
        elif k == "t":
            state.show_trail = not state.show_trail
        elif k == "g":
            state.show_map = not state.show_map
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


def connect(fig, state, ax, demo="avoid"):
    clear_default_keymap()
    handler, release = make_handler(state, fig, ax, demo)
    fig.canvas.mpl_connect("key_press_event", handler)
    fig.canvas.mpl_connect("key_release_event", release)

    def click(event):
        if event.inaxes is not ax or event.xdata is None:
            return
        if event.button == 1:
            state.newGoal = (event.xdata, event.ydata)
        elif event.button == 3:
            state.newStart = (event.xdata, event.ydata)

    fig.canvas.mpl_connect("button_press_event", click)
