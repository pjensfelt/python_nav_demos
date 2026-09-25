#!/usr/bin/env python3
"""Pure pursuit path following.

    python run_pure_pursuit.py                      open the window
    python run_pure_pursuit.py --path 3 --set lookahead=0.3
    python run_pure_pursuit.py --law pure-pursuit --set lookahead=0.5
    python run_pure_pursuit.py --headless --path 2  run to the goal, print a summary

Press 'h' in the window (or see README.md) for the key bindings.
"""

import time

import numpy as np

from navdemo import app, draw, keys
from navdemo.params import DemoState
from navdemo.path import Path, PATH_DIR, builtin_paths
from navdemo.sim import Robot, Follower


def load_path(spec, paths):
    if spec in ("1", "2", "3", "4"):
        return paths[int(spec) - 1]
    return Path.load(spec)


def main():
    args = app.common_args(__doc__.splitlines()[0]).parse_args()
    rng = np.random.default_rng(args.seed)

    state = DemoState()
    app.apply_common_args(state, args)
    state.turnInPlace = args.turn_in_place
    paths = builtin_paths()
    theta0 = np.deg2rad(args.theta0)

    def make_robot(path):
        return Robot(*path.start, theta0)

    path = load_path(args.path, paths)
    follower = Follower(path, make_robot(path))

    fig = None
    if not args.headless:
        plt = app.pyplot(args)
        keys.clear_default_keymap()
        fig = plt.figure("Pure pursuit", figsize=(11, 7))
        ax = draw.setup_axes(fig, "Pure pursuit path following")
        (path_line,) = ax.plot([], [], color="C0", lw=1.5, zorder=3)
        (trail,) = ax.plot([], [], color="r", lw=1, alpha=0.8, zorder=3)
        (goal,) = ax.plot([], [], "s", color="C0", ms=6, mfc="none", zorder=3)
        robot = draw.RobotArtist(ax)
        geometry = draw.GeometryArtist(ax)
        charts = draw.StripCharts(fig)
        panel = draw.Panel(fig)
        drawer = keys.connect(fig, state, ax=ax, demo="purepursuit")
        if not args.snapshot:
            print(keys.HELP)

    def step(_frame=0):
        nonlocal path
        # ---- one-shot requests ----------------------------------------
        if state.newPath is not None:
            new = state.newPath
            state.newPath = None
            path = paths[new] if isinstance(new, int) else new
            follower.robot = make_robot(path)
            follower.set_path(path)
            print(f"path: {path.name}, {len(path.xy)} waypoints, {path.length:.2f} m")
        if state.reset:
            follower.reset()
            state.reset = False
        if state.disturb:
            follower.disturb(rng)
            state.disturb = False
        if state.savePath:
            PATH_DIR.mkdir(exist_ok=True)
            fname = PATH_DIR / f"drawn_{time.strftime('%Y%m%d%H%M%S')}.csv"
            path.save(fname)
            print(f"wrote {fname}  (run again with --path {fname})")
            state.savePath = False

        # ---- simulation ------------------------------------------------
        p = state.controller_params()
        if not state.paused:
            follower.advance(app.FRAME_DT * p["time_scale"], p)

        if fig is None:
            return []

        # ---- drawing ---------------------------------------------------
        path_line.set_data(path.xy[:, 0], path.xy[:, 1])
        goal.set_data([path.xy[-1, 0]], [path.xy[-1, 1]])
        trail.set_visible(state.showTrail)
        trail.set_data(*zip(*follower.trail))
        robot.set_pose(*follower.robot.pose)
        geometry.set(follower, p["lookahead"], state.showGeometry)
        charts.set(follower)
        panel.update(state, follower)
        return ([path_line, trail, goal] + robot.artists + geometry.artists
                + charts.artists + panel.artists + drawer.artists)

    app.run(fig, state, step, args.headless, args.steps, args.snapshot,
            done=lambda: follower.done and abs(follower.robot.v) < 1e-3)

    if args.headless or args.snapshot:
        if follower.done:
            miss = np.hypot(*(np.array(follower.robot.pose[:2]) - path.xy[-1]))
            print(f"Reached the end of {path.name} at t={follower.t_done:.2f}s "
                  f"with max cross-track error {follower.max_abs_e:.3f}m, "
                  f"stopped {miss:.3f}m from the goal")
        else:
            print(f"Did not reach the end of {path.name} in t={follower.t:.2f}s "
                  f"(max cross-track error {follower.max_abs_e:.3f}m)")


if __name__ == "__main__":
    main()
