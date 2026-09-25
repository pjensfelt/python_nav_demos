"""Shared plumbing: argument parsing, the main loop and screenshots.

Same shape as locdemo/app.py in python_loc_demos, so the programs look and
run the same way -- but copied, not imported, so this repo stands alone.
"""

import argparse
import time
from pathlib import Path

import numpy as np

from .params import TUNABLE_BY_NAME

# Wall-clock time between animation frames. Each frame advances the
# simulation by FRAME_DT * time_scale seconds of simulated time.
FRAME_DT = 0.04

SNAPSHOT_DIR = Path("snapshots")


def save_screenshot(fig, ax, demo):
    """Two PNGs in SNAPSHOT_DIR: the whole window and just the plot axes."""
    SNAPSHOT_DIR.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d%H%M%S")
    win_path = SNAPSHOT_DIR / f"snap_{stamp}_{demo}_win.png"
    plot_path = SNAPSHOT_DIR / f"snap_{stamp}_{demo}_plot.png"
    fig.savefig(win_path, dpi=150)
    fig.canvas.draw()
    bbox = ax.get_tightbbox(fig.canvas.get_renderer()).transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(plot_path, dpi=150, bbox_inches=bbox)
    print(f"wrote {win_path}\nwrote {plot_path}")
    return win_path, plot_path


def common_args(description):
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("--seed", type=int, default=None,
                    help="seed the random generator ('d' disturbances) for a repeatable run")
    ap.add_argument("--headless", action="store_true",
                    help="run without a window until the goal is reached, for testing")
    ap.add_argument("--steps", type=int, default=2000,
                    help="max number of frames when headless, or frames before --snapshot")
    ap.add_argument("--snapshot", default=None, metavar="FILE.png",
                    help="run --steps frames (or to the goal) offscreen and save the final frame")
    ap.add_argument("--path", default="1", metavar="1..4|FILE.csv",
                    help="built-in path 1..4 (as the MATLAB path.mat, path1..3.mat), "
                         "or a CSV of x,y waypoints")
    ap.add_argument("--theta0", type=float, default=0.0,
                    help="robot's starting heading [deg]; it always starts at the path's first point")
    ap.add_argument("--law", choices=["heading-P", "pure-pursuit"], default="heading-P",
                    help="control law to start with ('c' toggles)")
    ap.add_argument("--turn-in-place", action="store_true",
                    help="pure pursuit law: turn on the spot when the target is behind ('b' toggles)")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="preset a tunable, e.g. --set lookahead=0.3 --set accw=360 "
                         "(angles in degrees, 'inf' for unlimited/off). Names: "
                         + ", ".join(TUNABLE_BY_NAME))
    return ap


def apply_common_args(state, args):
    state.law = {"heading-P": "heading-P", "pure-pursuit": "pure pursuit"}[args.law]
    for item in args.set:
        name, _, raw = item.partition("=")
        if name not in TUNABLE_BY_NAME:
            raise SystemExit(f"--set: unknown tunable {name!r}, expected one of "
                             + ", ".join(TUNABLE_BY_NAME))
        value = float(raw)
        if TUNABLE_BY_NAME[name].unit.startswith("deg"):
            value = np.deg2rad(value)
        state.set_value(name, value)


def pyplot(args):
    """Import pyplot, forcing a non-interactive backend when snapshotting."""
    import matplotlib
    if args.snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def run(fig, state, step, headless=False, steps=0, snapshot=None, done=lambda: False):
    """Drive `step` from a matplotlib timer, or as a plain loop that stops
    at the goal (`done()`) or after `steps` frames."""
    if snapshot or headless:
        for i in range(steps):
            step(i)
            if done() or not state.running:
                break
        if snapshot:
            fig.savefig(snapshot, dpi=110)
            print(f"wrote {snapshot}")
        return

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    ani = FuncAnimation(fig, step, interval=int(FRAME_DT * 1000),
                        blit=False, cache_frame_data=False)
    fig._nav_demo_animation = ani  # keep a reference so it isn't collected
    plt.show()
