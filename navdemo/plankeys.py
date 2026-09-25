"""Keyboard and mouse control for run_planning.py.

Same conventions as the other demos (tab / > / <, h, S, q); the keys
themselves are listed in HELP.
"""

import time

from . import app
from .grid import RASTER_MODES
from .keys import clear_default_keymap, _MIN_REPEAT_INTERVAL
from .planners import PLANNERS
from .planstate import PlanState
from .sim import CONTROL_LAWS

_CONTINUOUS_KEYS = {">", "<"}

HELP = """
 mission                  map / planner               display
 -------                  -------------               -------
 enter  plan              k  known map / map as we go g  grid
 space  drive / pause     m  cells: center / conserv. o  real obstacles
 r      reset             p  planner: A* / RRT / RRT* e  search (A* cells, tree)
 1..8   world             n  A*: 4 / 8 connectivity   l  lookahead geometry
 mouse  left: goal        x  RRT: grid / exact geom.  t  driven trail
        right: start      f  RRT: stop at 1st path    S  screenshot (2 pngs)
                          s  shortcut the path
 tab / S-tab  select      c  control law              h  this help
                          b  pure pursuit: turn in place
 > / <        change it                               q  quit
"""


def _cycle(options, current):
    return options[(options.index(current) + 1) % len(options)]


def make_handler(state: PlanState, fig=None, ax=None, demo="planning"):
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

        if k == "enter":
            state.plan = True
        elif k == " ":
            state.drive = True
        elif k == "r":
            state.reset = True
        elif k in "12345678":
            state.newWorld = int(k) - 1
        elif k == "k":
            state.mapped = not state.mapped
        elif k == "m":
            state.raster = _cycle(RASTER_MODES, state.raster)
        elif k == "p":
            state.planner = _cycle(PLANNERS, state.planner)
        elif k == "n":
            state.eight = not state.eight
        elif k == "x":
            state.exact_geometry = not state.exact_geometry
            if state.planner == "A*":
                print("x: only RRT/RRT* can check the exact geometry -- A* needs grid "
                      "cells to search (press p to switch planner)")
            elif state.mapped:
                print("x: not while mapping as we go -- the grid is all the robot knows")
            else:
                print(f"x: {state.planner} now checks collisions against the "
                      f"{state.checks_text()} (enter to replan)")
        elif k == "f":
            state.stop_at_goal = not state.stop_at_goal
        elif k == "s":
            state.shortcut = not state.shortcut
        elif k == "c":
            state.law = _cycle(CONTROL_LAWS, state.law)
        elif k == "b":
            state.turn_in_place = not state.turn_in_place
        elif k == "g":
            state.show_grid = not state.show_grid
        elif k == "o":
            state.show_geometry = not state.show_geometry
        elif k == "e":
            state.show_search = not state.show_search
        elif k == "l":
            state.show_lookahead = not state.show_lookahead
        elif k == "t":
            state.show_trail = not state.show_trail
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


def connect(fig, state, ax, demo="planning"):
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
