#!/usr/bin/env python3
"""Obstacle avoidance: local methods that only see the lidar.

    python run_avoid.py                          open the window
    python run_avoid.py --world 2 --method DWA   the trap, with DWA
    python run_avoid.py --goal carrot            follow a carrot along an A* path
    python run_avoid.py --headless --world 3 --method VFH --goal carrot

Press 'h' in the window (or see README.md) for the key bindings.
"""

import argparse

import numpy as np

from navdemo import app, draw, plandraw, avoidkeys, avoiddraw
from navdemo.avoid import AvoidSim, AvoidWorld, GOAL_MODES, METHODS, avoid_worlds
from navdemo.avoidstate import AvoidState, ROBOT_RADIUS, TUNABLE_BY_NAME


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world", default="1", metavar="1..4|FILE.json",
                    help="world: a number (key 1..) or a JSON file")
    ap.add_argument("--method", choices=["VFH", "potential-field", "DWA"], default="VFH")
    ap.add_argument("--goal", choices=["clicked", "carrot"], default="clicked",
                    help="head straight for the goal, or for a carrot along an A* path")
    ap.add_argument("--no-people", action="store_true", help="start without the people")
    ap.add_argument("--blind", action="store_true",
                    help="people walk blindly instead of waiting for the robot")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="set a parameter (row name as in the panel); angles in degrees")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--headless", action="store_true",
                    help="no window: drive to the goal and print a summary")
    ap.add_argument("--steps", type=int, default=5000,
                    help="max frames (headless / snapshot)")
    ap.add_argument("--snapshot", default=None, metavar="FILE.png",
                    help="run --steps frames without a window and save the figure")
    return ap.parse_args()


def main():
    args = parse_args()
    rng = np.random.default_rng(args.seed)
    state = AvoidState(method=args.method.replace("-", " "),
                       goal_mode=GOAL_MODES[1] if args.goal == "carrot" else GOAL_MODES[0],
                       polite=not args.blind, people_on=not args.no_people)
    for item in args.set:
        name, _, raw = item.partition("=")
        if name not in TUNABLE_BY_NAME:
            raise SystemExit(f"--set: unknown tunable {name!r}, expected one of "
                             + ", ".join(TUNABLE_BY_NAME))
        value = float(raw)
        if TUNABLE_BY_NAME[name].unit.startswith("deg"):
            value = np.deg2rad(value)
        state.set_value(name, value)
    if not state.relevant(state.selected):
        state.cursor = state.visible()[0]

    worlds = avoid_worlds()
    if args.world in [str(i + 1) for i in range(len(worlds))]:
        world = worlds[int(args.world) - 1]
    else:
        world = AvoidWorld.load(args.world)

    def new_sim(w):
        s = AvoidSim(w, ROBOT_RADIUS, rng)
        s.build_map(state.config())
        if state.goal_mode == GOAL_MODES[1]:
            s.plan_global(state.config())
        return s

    sim = new_sim(world)

    fig = None
    if not args.headless:
        plt = app.pyplot(args)
        fig = plt.figure("Obstacle avoidance", figsize=(11, 7))
        ax = plandraw.setup_axes(fig, "Obstacle avoidance: only the lidar")
        grid_art = plandraw.GridArtist(ax)
        obstacles = plandraw.ObstacleArtist(ax)
        unmapped = avoiddraw.UnmappedArtist(ax)
        people = avoiddraw.PeopleArtist(ax)
        method_art = avoiddraw.MethodArtist(ax)
        (plan_line,) = ax.plot([], [], color="C0", lw=2, alpha=0.7, zorder=4)
        (carrot,) = ax.plot([], [], "o", color="C0", ms=8, zorder=9)
        (scan_dots,) = ax.plot([], [], ".", color="r", ms=3, zorder=7)
        (trail,) = ax.plot([], [], color="r", lw=1, zorder=7)
        (start_mark,) = ax.plot([], [], "o", color="k", mfc="none", ms=8, zorder=7)
        (goal_mark,) = ax.plot([], [], "*", color="gold", mec="k", ms=16, zorder=9)
        robot = draw.RobotArtist(ax)
        crash_text = ax.text(0, 0, "COLLISION", color="r", fontsize=14, fontweight="bold",
                             ha="center", va="bottom", zorder=10, visible=False)
        legend = avoiddraw.Legend(fig)
        panel = avoiddraw.Panel(fig)
        avoidkeys.connect(fig, state, ax)

        def show_world(w):
            plandraw.set_world_limits(ax, w.mapped)
            obstacles.set_world(w.mapped)
            unmapped.set_world(w.unmapped)
            people.set_world(w.movers)

        show_world(world)
        if not args.snapshot:
            print(avoidkeys.HELP)

    def step(_frame=0):
        nonlocal world, sim
        cfg = state.config()

        # ---- one-shot requests ----------------------------------------
        if state.newWorld is not None:
            if state.newWorld < len(worlds):
                world = worlds[state.newWorld]
                sim = new_sim(world)
                print(f"world: {world.name} -- {world.comment}")
                if fig is not None:
                    show_world(world)
            state.newWorld = None
        if state.newStart is not None:
            x, y = state.newStart
            a = np.arctan2(sim.goal[1] - y, sim.goal[0] - x)
            sim.start = (x, y, a)
            sim.reset()
            state.newStart = None
            state.replan = True
        if state.newGoal is not None:
            # a new goal: the robot carries on from where it is
            sim.goal = tuple(state.newGoal)
            sim.done = False
            sim._best = (np.inf, sim.t)
            state.newGoal = None
            state.replan = True
        if state.reset:
            sim.reset()
            state.reset = False
            state.replan = True
        if state.replan:
            state.replan = False
            if state.goal_mode == GOAL_MODES[1]:
                res = sim.plan_global(cfg)
                if not res.path:
                    print("A*: " + res.message)
        if state.drive:
            state.drive = False
            if sim.done or sim.collided:
                print("press 'r' to reset first")
            else:
                sim.driving = not sim.driving

        # ---- simulation ------------------------------------------------
        if sim.driving:
            sim.advance(app.FRAME_DT * cfg["time_scale"], cfg)
        elif sim.scan is None or not (sim.done or sim.collided):
            sim.sense(cfg)          # standing still: still show what it sees
            sim.carrot = sim.local_goal(cfg)

        if fig is None:
            return []

        # ---- status ----------------------------------------------------
        if sim.collided:
            status = f"COLLISION with {sim.hit_by}"
        elif sim.done:
            status = "GOAL reached"
        elif sim.stuck:
            status = "stuck? no closer to the goal for 10 s"
        elif sim.driving:
            status = "driving"
        else:
            status = "space to drive"

        # ---- drawing ---------------------------------------------------
        carrot_mode = state.goal_mode == GOAL_MODES[1] and sim.plan is not None
        grid_art.set(sim.map, state.show_map and carrot_mode, faded=True)
        obstacles.set_visible(state.show_geometry)
        unmapped.set_visible(state.show_geometry)
        people.update(world.movers, state.people_on)
        if carrot_mode:
            plan_line.set_data(sim.plan.xy[:, 0], sim.plan.xy[:, 1])
            c = getattr(sim, "carrot", None)
            carrot.set_data(*([[c[0]], [c[1]]] if c is not None else [[], []]))
        else:
            plan_line.set_data([], [])
            carrot.set_data([], [])
        h = sim.hits if state.show_scan else np.empty((0, 2))
        scan_dots.set_data(h[:, 0], h[:, 1])
        trail.set_data(*zip(*sim.trail))
        trail.set_visible(state.show_trail)
        start_mark.set_data([sim.start[0]], [sim.start[1]])
        goal_mark.set_data([sim.goal[0]], [sim.goal[1]])
        robot.set_pose(*sim.robot.pose)
        for hh in robot.artists:
            hh.set_color("r" if sim.collided else "k")
        crash_text.set_position((sim.robot.x, sim.robot.y + 2.5 * ROBOT_RADIUS))
        crash_text.set_visible(sim.collided)
        method_art.set(sim, state.method, state.show_method)
        panel.update(state, sim, status)
        legend.update(state, sim)
        return []

    if args.headless or args.snapshot:
        sim.driving = True
    app.run(fig, state, step, args.headless, args.steps, args.snapshot,
            done=lambda: not sim.driving)

    if args.headless or args.snapshot:
        outcome = ("reached the goal" if sim.done else
                   f"COLLIDED with {sim.hit_by}" if sim.collided else
                   "stuck (no closer to the goal for 10 s)" if sim.stuck else "stopped")
        print(f"{world.name}, {state.method}, {state.goal_mode}: {outcome} at t={sim.t:.1f}s, "
              f"closest {sim.min_clear:.2f} m")


if __name__ == "__main__":
    main()
