#!/usr/bin/env python3
"""Planning and executing in a world you only know through a grid.

    python run_planning.py                         open the window
    python run_planning.py --world 3 --set res=0.5 thin walls vanish from a coarse grid
    python run_planning.py --world 8 --mapped      map as we go: into the dead end and out
    python run_planning.py --headless --world 8 --mapped   plan, drive, print a summary

Press 'h' in the window (or see README.md) for the key bindings.
"""

import argparse

import numpy as np

from navdemo import app, draw, plandraw, plankeys
from navdemo.mission import Mission
from navdemo.planners import PLANNERS
from navdemo.planstate import PlanState, ROBOT_RADIUS, TUNABLE_BY_NAME
from navdemo.world import World, builtin_worlds, imperfect


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="1", metavar="1..8|FILE.json",
                    help="built-in world 1..8 (worlds/*.json) or a world file")
    ap.add_argument("--planner", choices=PLANNERS, default="A*")
    ap.add_argument("--mapped", action="store_true",
                    help="start mapping as we go (lidar) instead of with the known map")
    ap.add_argument("--exact", action="store_true",
                    help="RRT/RRT*: check collisions against the exact geometry, not the grid")
    ap.add_argument("--law", choices=["heading-P", "pure-pursuit"], default="heading-P")
    ap.add_argument("--no-turn-in-place", action="store_true",
                    help="pure pursuit law: don't turn on the spot when the target is behind")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="preset a tunable, e.g. --set res=0.25 --set inflate=0.2. Names: "
                         + ", ".join(TUNABLE_BY_NAME))
    ap.add_argument("--seed", type=int, default=None, help="seed for RRT/RRT* and sensor noise")
    ap.add_argument("--headless", action="store_true",
                    help="no window: plan, drive to the goal (or a collision), print a summary")
    ap.add_argument("--steps", type=int, default=5000,
                    help="max frames when headless or before --snapshot")
    ap.add_argument("--snapshot", default=None, metavar="FILE.png",
                    help="plan, drive --steps frames offscreen, save the final frame")
    return ap.parse_args()


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    worlds = builtin_worlds()

    state = PlanState(planner=args.planner, mapped=args.mapped, exact_geometry=args.exact,
                      turn_in_place=not args.no_turn_in_place,
                      law={"heading-P": "heading-P", "pure-pursuit": "pure pursuit"}[args.law])
    for item in args.set:
        name, _, raw = item.partition("=")
        if name not in TUNABLE_BY_NAME:
            raise SystemExit(f"--set: unknown tunable {name!r}, expected one of "
                             + ", ".join(TUNABLE_BY_NAME))
        state.set_value(name, float(raw))
    if args.world in [str(i + 1) for i in range(len(worlds))]:
        base = worlds[int(args.world) - 1]
    else:
        base = World.load(args.world)

    def build_world():
        """The building as built: `base` (as drawn in its file) with its
        corners moved a little, the same for the same seed and variant."""
        rng_b = np.random.default_rng([args.seed or 0, 7, state.variant])
        return imperfect(base, state.value("imperfect"), rng_b)

    world = build_world()
    world_key = state.world_key()
    mission = Mission(world, ROBOT_RADIUS, rng)
    mission.build_map(state.mission_config())
    grid_key = state.grid_key()
    plan_key = None                   # settings the current plan was made with
    replay = {"k": 0, "per_frame": 1, "result": None}

    fig = None
    if not args.headless:
        plt = app.pyplot(args)
        fig = plt.figure("Planning", figsize=(11, 7))
        ax = plandraw.setup_axes(fig, "Planning on a grid, driving in the real world")
        plandraw.set_world_limits(ax, world)
        grid_art = plandraw.GridArtist(ax)
        obstacles = plandraw.ObstacleArtist(ax)
        obstacles.set_world(world)
        search = plandraw.SearchArtist(ax)
        scan_art = plandraw.ScanArtist(ax)
        cspace = plandraw.CSpaceArtist(ax)
        old_lines = []
        (raw_line,) = ax.plot([], [], ":", color="C0", lw=1, zorder=6)
        (plan_line,) = ax.plot([], [], color="C0", lw=2.5, alpha=0.8, zorder=6)
        (trail,) = ax.plot([], [], color="r", lw=1, zorder=7)
        (start_mark,) = ax.plot([], [], "o", color="k", mfc="none", ms=8, zorder=7)
        (goal_mark,) = ax.plot([], [], "*", color="gold", mec="k", ms=16, zorder=9)
        robot = draw.RobotArtist(ax)
        lookahead = draw.GeometryArtist(ax)
        plandraw.add_legend(fig)
        panel = plandraw.Panel(fig)
        plankeys.connect(fig, state, ax)
        if not args.snapshot:
            print(plankeys.HELP)

    def start_replay(result, animate):
        replay["result"] = result
        total = search.total(result) if fig is not None else 0
        frames = state.value("anim") / app.FRAME_DT
        if animate and frames > 0 and total > 0:
            replay["k"] = 0
            replay["per_frame"] = max(1, int(np.ceil(total / frames)))
        else:
            replay["k"] = total

    def replaying():
        return fig is not None and replay["result"] is not None and \
            replay["k"] < search.total(replay["result"])

    def do_plan(animate):
        nonlocal plan_key
        res = mission.plan(cfg)
        plan_key = state.plan_key()
        start_replay(res, animate)
        print(f"{state.planner}: " + (f"path {res.cost:.2f} m" if res.path else res.message)
              + (f", {len(res.expanded)} cells expanded" if res.expanded else
                 f", {len(res.nodes)} nodes" if res.nodes is not None else "")
              + f", {1000 * res.seconds:.0f} ms")

    cfg = state.mission_config()

    def step(_frame=0):
        nonlocal base, world, world_key, mission, grid_key, plan_key, cfg
        cfg = state.mission_config()

        # ---- one-shot requests ----------------------------------------
        new_base = state.newWorld is not None and state.newWorld < len(worlds)
        if new_base:
            base = worlds[state.newWorld]
            print(f"world: {base.name}")
        state.newWorld = None
        if new_base or state.world_key() != world_key:
            world = build_world()
            world_key = state.world_key()
            mission = Mission(world, ROBOT_RADIUS, rng)
            grid_key = None
            if fig is not None:
                obstacles.set_world(world)
                plandraw.set_world_limits(ax, world)
        if state.grid_key() != grid_key:
            mission.build_map(cfg)
            grid_key = state.grid_key()
            plan_key = None
            start_replay(None, False)
        if state.newStart is not None or state.newGoal is not None:
            if state.newStart is not None:
                mission.start = (*state.newStart, mission.start[2])
            if state.newGoal is not None:
                mission.goal = state.newGoal
            state.newStart = state.newGoal = None
            mission.reset()
            plan_key = None
            start_replay(None, False)
        if state.reset:
            mission.reset()
            plan_key = None
            start_replay(None, False)
            state.reset = False
        if state.plan:
            state.plan = False
            if not mission.driving:
                do_plan(animate=True)
        if state.drive:
            state.drive = False
            if mission.done or mission.collided:
                print("press 'r' to reset first")
            elif mission.driving:
                mission.driving = False
            else:
                if not mission.path or plan_key != state.plan_key():
                    do_plan(animate=False)
                replay["k"] = search.total(replay["result"]) if fig is not None else 0
                mission.driving = bool(mission.path)

        # ---- simulation ------------------------------------------------
        if replaying():
            replay["k"] += replay["per_frame"]
        else:
            result_before = mission.result
            mission.advance(app.FRAME_DT * cfg["time_scale"], cfg)
            if mission.result is not result_before:     # replanned while driving
                start_replay(mission.result, animate=False)

        if fig is None:
            return []

        # ---- status ----------------------------------------------------
        if mission.collided:
            status = "COLLISION with a real obstacle"
        elif mission.done:
            status = "GOAL reached"
        elif mission.failed:
            status = mission.failed
        elif replaying():
            status = "planning (replay)"
        elif mission.driving:
            status = "driving"
        elif mission.path and plan_key == state.plan_key():
            status = "planned: space to drive"
        elif mission.path:
            status = "settings changed: enter to replan"
        else:
            status = "enter to plan, space to go"

        # ---- drawing ---------------------------------------------------
        grid_art.set(mission.grid, state.show_grid, faded=state.uses_exact_geometry)
        cspace.set(world, state.value("inflate"), state.uses_exact_geometry and state.show_geometry)
        obstacles.set_visible(state.show_geometry)
        search.set(replay["result"], mission.grid, replay["k"], state.show_search)
        scan_art.set(mission.scan if state.mapped else None)
        show_plan = not replaying()
        path = np.array(mission.path) if (mission.path and show_plan) else np.empty((0, 2))
        plan_line.set_data(path[:, 0], path[:, 1])
        raw = mission.result.path if (mission.result and state.shortcut and show_plan) else []
        raw = np.array(raw) if raw else np.empty((0, 2))
        raw_line.set_data(raw[:, 0], raw[:, 1])
        while len(old_lines) < len(mission.old_paths):
            (ln,) = ax.plot([], [], "--", color="0.4", lw=1, zorder=5)
            old_lines.append(ln)
        for i, ln in enumerate(old_lines):
            if i < len(mission.old_paths):
                p = np.array(mission.old_paths[i])
                ln.set_data(p[:, 0], p[:, 1])
            else:
                ln.set_data([], [])
        trail.set_data(*zip(*mission.trail))
        trail.set_visible(state.show_trail)
        start_mark.set_data([mission.start[0]], [mission.start[1]])
        goal_mark.set_data([mission.goal[0]], [mission.goal[1]])
        robot.set_pose(*mission.robot.pose)
        for h in robot.artists:
            h.set_color("r" if mission.collided else "k")
        if mission.follower is not None and state.show_lookahead and show_plan:
            lookahead.set(mission.follower, cfg["lookahead"], True)
        else:
            lookahead.set(None, 0, False)
        panel.update(state, mission, status)
        return []

    if args.headless or args.snapshot:
        do_plan(animate=False)
        mission.driving = bool(mission.path)
    app.run(fig, state, step, args.headless, args.steps, args.snapshot,
            done=lambda: not mission.driving)

    if args.headless or args.snapshot:
        outcome = ("reached the goal" if mission.done else
                   "COLLIDED with a real obstacle" if mission.collided else
                   mission.failed or "stopped")
        print(f"{world.name}: {outcome} at t={mission.t:.1f}s, driven {mission.driven:.2f} m, "
              f"{mission.replans} replans")


if __name__ == "__main__":
    main()
