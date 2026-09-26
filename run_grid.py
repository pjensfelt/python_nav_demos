#!/usr/bin/env python3
"""The world and its grid -- and how much the grid depends on where the
cell borders happen to fall.

    python run_grid.py                                   open the window
    python run_grid.py --world 1 --set res=0.5 --sweep y door: open or closed depending on a few cm
    python run_grid.py --world 3 --set res=0.5 --sweep r rotate the grid 0 -> 45 deg
    python run_grid.py --headless --world 1 --set res=0.5 --sweep y   print the sweep as text

Press 'h' in the window (or see README.md) for the key bindings.
"""

import argparse
import time

import numpy as np
from matplotlib.transforms import Affine2D

from navdemo import app, griddraw, gridstate
from navdemo.gridstate import GridState, Pose, SWEEP_ANGLE, SWEEP_STEPS, TUNABLE_BY_NAME, rot
from navdemo.rasterize import (connected, rasterize, rasterize_samples, room_walls,
                               sample_outlines, square_distance, transform_obstacles)
from navdemo.planners import astar, path_length
from navdemo.world import Polygon
from navdemo.world import World, grid_worlds, imperfect


def occupied_percent(grid):
    """Of the cells inside the world."""
    return 100 * grid.occ[grid.known].mean()


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="1", metavar="1..9|FILE.json",
                    help="1-4: worlds/grid/, 5-9: the first planning worlds; or a world file")
    ap.add_argument("--rule", choices=["any-overlap", "samples"], default="any-overlap")
    ap.add_argument("--seed", type=int, default=0, help="seed for the sample points")
    ap.add_argument("--plan", action="store_true",
                    help="start with planning between the probe points on ('p' toggles)")
    ap.add_argument("--inflate", choices=["grid", "world"], default="grid",
                    help="inflation order: 'grid' grows the occupied cells, in the grid "
                         "(as from sensor data); 'world' grows the real geometry, then makes "
                         "cells (needs a model). The amount is --set inflate=METRES")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="preset res or inflate, e.g. --set res=0.25")
    ap.add_argument("--grid-offset", type=float, nargs=2, default=[0.0, 0.0], metavar=("DX", "DY"))
    ap.add_argument("--grid-rotate", type=float, default=0.0, metavar="DEG")
    ap.add_argument("--world-offset", type=float, nargs=2, default=[0.0, 0.0], metavar=("DX", "DY"))
    ap.add_argument("--world-rotate", type=float, default=0.0, metavar="DEG")
    ap.add_argument("--move", choices=["grid", "world"], default="grid",
                    help="what the keys (and --sweep) move")
    ap.add_argument("--sweep", choices=["x", "y", "r"], default=None,
                    help="sweep one cell along the grid's x or y, or the rotation 0 -> 45 deg")
    ap.add_argument("--headless", action="store_true",
                    help="no window: run the --sweep and print it as text")
    ap.add_argument("--snapshot", default=None, metavar="FILE.png",
                    help="save the window (after the --sweep, if any) and exit")
    ap.add_argument("--steps", type=int, default=2 * SWEEP_STEPS + 1,
                    help="frames to run when headless or snapshotting")
    return ap.parse_args()


def main():
    args = parse_args()
    worlds = grid_worlds()
    state = GridState(rule=args.rule.replace("-", " "),
                      moving=args.move,
                      inflate_order=f"{args.inflate} first",
                      planning=args.plan)
    for item in args.set:
        name, _, raw = item.partition("=")
        if name not in TUNABLE_BY_NAME:
            raise SystemExit(f"--set: unknown tunable {name!r}, expected one of "
                             + ", ".join(TUNABLE_BY_NAME))
        state.set_value(name, float(raw))
    state.grid = Pose(np.array(args.grid_offset, float), np.deg2rad(args.grid_rotate))
    state.world = Pose(np.array(args.world_offset, float), np.deg2rad(args.world_rotate))
    # `base` is the world as drawn in its file; `world` the building as
    # built -- the base with its corners moved a little (see imperfect),
    # made again when the world, the imperfection or the variant changes.
    base = worlds[int(args.world) - 1] if args.world in [str(i + 1) for i in range(len(worlds))] \
        else World.load(args.world)
    world, wkey = None, None

    def update_world():
        nonlocal world, wkey
        key = (id(base), state.value("imperfect"), state.variant)
        if key != wkey:
            rng = np.random.default_rng([args.seed, 7, state.variant])
            world, wkey = imperfect(base, state.value("imperfect"), rng), key
            return True
        return False

    update_world()
    if args.sweep:
        state.start_sweep(args.sweep)

    cache = {}

    def centre():
        xmin, xmax, ymin, ymax = base.bounds       # as drawn: stays put across variants
        return np.array([(xmin + xmax) / 2, (ymin + ymax) / 2])

    def samples():
        """The sample points along the outlines, in the world file's
        coordinates: fixed to the world, so that moving the grid shows
        discretization only. 'u' draws a fresh set."""
        key = ("samples", wkey, state.value("spacing"), state.value("sigma"),
               state.sample_draw)
        if key not in cache:
            rng = np.random.default_rng([args.seed, state.sample_draw])
            cache[key] = sample_outlines(world.obstacles, world.room, state.value("spacing"),
                                         state.value("sigma"), rng)
        return cache[key]

    def sample_key():
        return ((state.value("spacing"), state.value("sigma"), state.value("min_hits"),
                 state.sample_draw) if state.rule == "samples" else ())

    def probes_file():
        """The probe points in the world file's coordinates: as designed,
        with the first pair's start / goal replaced by clicked ones."""
        probes = list(world.probes)
        if state.start is not None or state.goal is not None:
            name, a, b = probes[0]
            probes[0] = (name, state.start if state.start is not None else a,
                         state.goal if state.goal is not None else b)
        return probes

    def to_file(p):
        """A point in the drawing frame -> the world file's coordinates (undo
        the world's own pose), so a clicked point moves with the world."""
        c = centre()
        return tuple(rot(state.world.angle).T @ (np.asarray(p) - c - state.world.offset) + c)

    def compute(rel):
        """The grid for the world at relative pose `rel` (see
        GridState.relative), with the probe results in grid coordinates.
        Cached, since poses only change on key presses."""
        key = (wkey, state.rule, state.effective_inflate_order,
               state.value("res"), state.value("inflate"), state.planning,
               state.start, state.goal) + sample_key() + tuple(rel)
        if key in cache:
            return cache[key]
        obs, move = transform_obstacles(world.obstacles + room_walls(world.room), centre(),
                                        rel[:2], rel[2])
        # The grid covers the whole world, whatever its pose: the world's
        # bounding box in the grid's frame, on a fixed lattice of cells, plus
        # one ring of cells for the room's walls -- the walls are part of the
        # building, so the map shows them too (unmoved and as drawn, they lie
        # exactly on the box's edge and would otherwise fill no cell at all).
        # k0 is where the grid starts on the lattice, for lining two up.
        res = state.value("res")
        room = move(world.room)
        anchor = np.array([base.bounds[0], base.bounds[2]])   # the lattice, fixed by the design
        k0 = np.floor((room.min(axis=0) - anchor) / res + 1e-9).astype(int) - 1
        k1 = np.ceil((room.max(axis=0) - anchor) / res - 1e-9).astype(int) + 1
        extent = (anchor[0] + k0[0] * res, anchor[0] + k1[0] * res,
                  anchor[1] + k0[1] * res, anchor[1] + k1[1] * res)
        # Timed, to show what resolution costs: halve the cell size and there
        # are four times the cells, and roughly four times the time.
        t0 = time.perf_counter()
        if state.rule == "samples":
            grid = rasterize_samples(move(samples()), extent, res, state.value("inflate"),
                                     int(state.value("min_hits")))
        else:
            grid = rasterize(obs, extent, res, state.value("inflate"), state.inflate_order)
        grid.t_build = time.perf_counter() - t0
        grid.k0 = k0
        # Cells more than a cell away from the room (the far corners of a
        # rotated grid): drawn apart, left out of the stats. The ring of wall
        # cells right round the room counts as part of the world.
        grid.known = (square_distance(Polygon(room), grid.centres(), res / 2) < res).reshape(
            grid.nx, grid.ny)
        # A* between each pair of probe points (8-connected, no corner
        # cutting): the
        # same planner as run_planning.py, so the path is the shortest on
        # this grid (in metres) and the time is what planning really costs.
        probes, reach = [], None
        t0 = time.perf_counter()
        for i, (name, a, b) in enumerate(probes_file() if state.planning else []):
            pa, pb = move(a)[0], move(b)[0]
            if not (grid.inside(*grid.cell(*pa)) and grid.inside(*grid.cell(*pb))):
                result, why = None, "outside grid"
            else:
                result = astar(grid, pa, pb)
                why = ("start in occupied cell" if result.message.startswith("start") else
                       "goal in occupied cell" if result.message.startswith("goal") else
                       "no way through" if not result.path else "")
            path = result.path if result else []
            probes.append((name, bool(path), why, path,
                           path_length(path) if path else None, np.hypot(*(pb - pa))))
            if i == 0 and result is not None:
                reach = np.zeros((grid.nx, grid.ny), dtype=bool)
                if result.expanded:
                    cells = np.array(result.expanded)
                    reach[cells[:, 0], cells[:, 1]] = True
        grid.t_search = time.perf_counter() - t0 if state.planning else None
        if len(cache) > 300:
            cache.clear()
        cache[key] = (grid, probes, reach)
        return cache[key]

    def world_frame():
        """The obstacles and probe points where they are in the (fixed)
        drawing frame, i.e. moved by the world's own pose."""
        key = ("world", wkey, float(state.world.offset[0]), float(state.world.offset[1]),
               state.world.angle, state.start, state.goal) + sample_key()
        if key in cache:
            return cache[key]
        obs, move = transform_obstacles(world.obstacles, centre(), state.world.offset,
                                        state.world.angle)
        probes = [(name, move(a)[0], move(b)[0]) for name, a, b in probes_file()]
        room = move(world.room)
        pts = move(samples()) if state.rule == "samples" else None
        cache[key] = (obs, probes, room, pts)
        return cache[key]

    fig = None
    if not args.headless:
        plt = app.pyplot(args)
        fig = plt.figure("Grid", figsize=(11, 7))
        ax = fig.add_axes([0.30, 0.08, 0.68, 0.86])
        ax.set_aspect("equal")
        ax.set_title("The real world and its grid")
        fig.text(0.99, 0.01, "P. Jensfelt, KTH 2026", ha="right", va="bottom",
                 fontsize=7, color="0.6")
        view = griddraw.GridView(ax)
        griddraw.add_legend(fig)
        panel = griddraw.Panel(fig)
        gridstate.connect(fig, state, ax)
        if not args.snapshot:
            print(gridstate.HELP)

    limits = {}

    def update_limits(grid):
        """Keep the world and the (possibly rotated) grid in view. The view
        only grows, so it doesn't jump around while you rotate; it is reset
        with the world or with '0'."""
        xmin, xmax, ymin, ymax = world.bounds
        c = centre()
        corners = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
        gx0, gx1, gy0, gy1 = grid.extent
        grid_corners = np.array([[gx0, gy0], [gx1, gy0], [gx1, gy1], [gx0, gy1]])
        pts = [corners, (grid_corners - c) @ rot(state.grid.angle).T + c + state.grid.offset,
               (corners - c) @ rot(state.world.angle).T + c + state.world.offset]
        lo = np.min(np.vstack(pts), axis=0) - 0.3
        hi = np.max(np.vstack(pts), axis=0) + 0.3
        if limits:
            lo, hi = np.minimum(lo, limits["lo"]), np.maximum(hi, limits["hi"])
        if not limits or (lo != limits["lo"]).any() or (hi != limits["hi"]).any():
            limits["lo"], limits["hi"] = lo, hi
            ax.set_xlim(lo[0], hi[0])
            ax.set_ylim(lo[1], hi[1])

    def step(_frame=0):
        nonlocal base
        if state.newWorld is not None:
            if state.newWorld < len(worlds):
                base = worlds[state.newWorld]
                state.reset()
                state.sweep_kind = None
                state.sweep_log = []
                limits.clear()
                print(f"world: {base.name}")
            state.newWorld = None
        if update_world():
            state.sweep_log = []
        if state.newStart is not None:
            state.start, state.newStart = to_file(state.newStart), None
        if state.newGoal is not None:
            state.goal, state.newGoal = to_file(state.newGoal), None
        # ---- sweep: one cell along x or y, or the rotation, and back -------
        if state.sweep_kind is not None:
            u = state.sweep_u()
            state.apply_sweep(u)
            grid, probes, _ = compute(state.relative())
            state.sweep_log.append((u, occupied_percent(grid), [p[1] for p in probes],
                                    state.sweep_k <= SWEEP_STEPS))
            state.sweep_k += 1
            if state.sweep_k > 2 * SWEEP_STEPS:
                state.apply_sweep(0.0)
                state.sweep_kind = None

        if fig is None:
            return []

        if not (state.grid.offset.any() or state.grid.angle
                or state.world.offset.any() or state.world.angle):
            limits.clear()              # back at the start pose: default view
        grid, probes, reach = compute(state.relative())
        update_limits(grid)
        obs, probe_pts, room, pts = world_frame()
        c = centre()
        grid_to_axes = (Affine2D().translate(-c[0], -c[1]).rotate(state.grid.angle)
                        .translate(c[0] + state.grid.offset[0], c[1] + state.grid.offset[1])
                        + ax.transData)
        view.set(state, grid, obs, room, grid_to_axes,
                 [(name, a, b, p[1], p[3]) for (name, a, b), p in zip(probe_pts, probes)],
                 reach, pts if state.show_samples else None, robot_at=probe_pts[0][1])
        panel.update(state, world, worlds.index(base) + 1 if base in worlds else None,
                     grid, probes)
        return []

    app.run(fig, state, step, args.headless, args.steps, args.snapshot,
            done=lambda: state.sweep_kind is None)

    if args.headless:
        log = [r for r in state.sweep_log if r[3]]
        if not log:
            grid, probes, _ = compute(state.relative())
            print(f"{world.name}: {grid.nx}x{grid.ny} = {grid.nx * grid.ny} cells, "
                  f"{occupied_percent(grid):.1f}% occupied"
                  + "".join(f", {p[0]} {'open' if p[1] else 'blocked'}" for p in probes))
            return
        names = [p[0] for p in world.probes] if state.planning else []
        rotating = args.sweep == "r"
        what = "rotation 0 -> 45 deg" if rotating else f"one cell along the grid's {args.sweep}"
        print(f"{world.name}, {state.rule}, cell {state.value('res')} m, inflate "
              f"{state.value('inflate')} m ({state.effective_inflate_order}), moving the {state.moving}, "
              f"sweep: {what}")
        print(("  angle    " if rotating else "  offset   ") + "occupied  "
              + "  ".join(f"{n:>10}" for n in names))
        for u, occ, oks, _ in log[::4]:
            x = f"{np.rad2deg(u * SWEEP_ANGLE):5.1f}" if rotating else f"{u:5.2f}"
            print(f"  {x}    {occ:5.1f}%   " + "  ".join(
                f"{'open' if ok else '----':>10}" for ok in oks))
        for i, n in enumerate(names):
            frac = np.mean([r[2][i] for r in log])
            print(f"  {n}: open at {100 * frac:.0f}% of the {'angles' if rotating else 'offsets'}")


if __name__ == "__main__":
    main()
