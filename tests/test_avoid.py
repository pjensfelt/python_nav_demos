#!/usr/bin/env python3
"""Checks for the obstacle avoidance demo: the people, the local methods
and what they are (and aren't) able to do.

    python tests/test_avoid.py

No test framework needed; it just asserts and prints.
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from types import SimpleNamespace as N

import numpy as np

from navdemo import avoidkeys
from navdemo.avoid import (AvoidSim, AvoidWorld, Mover, METHODS, GOAL_MODES, avoid_worlds,
                           free_distance, vfh_cmd)
from navdemo.avoidstate import AvoidState, ROBOT_RADIUS, TUNABLES

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


def world(name):
    return AvoidWorld.load(name)


def drive(w, method, goal_mode, t_max=60, **settings):
    s = AvoidState(method=method, goal_mode=goal_mode)
    for k, v in settings.items():
        s.set_value(k, v)
    cfg = s.config()
    sim = AvoidSim(w, ROBOT_RADIUS, np.random.default_rng(0))
    if goal_mode == GOAL_MODES[1]:
        sim.plan_global(cfg)
    sim.driving = True
    while sim.driving and sim.t < t_max:
        sim.advance(0.5, cfg)
    return sim


@test
def test_people_walk_back_and_forth_and_pause():
    m = Mover(0.25, [[0, 0], [4, 0]], speed=1.0, pause=1.0)
    for t, x in ((2.0, 2.0), (4.5, 4.0), (6.0, 3.0), (9.5, 0.0), (11.0, 1.0)):
        m.reset()
        m.advance(t)
        assert abs(m.c[0] - x) < 1e-9, (t, m.c)
    m.reset()
    m.advance(1.5)
    m.turn_back()
    m.advance(0.5)
    assert abs(m.c[0] - 1.0) < 1e-9          # walking back from where it was


@test
def test_polite_people_wait_then_give_way():
    m = Mover(0.25, [[0, 0], [4, 0]], speed=1.0)
    robot = np.array([2.0, 0.0])
    for _ in range(150):                     # 1.5 s: walks up to the robot and waits
        m.advance(0.01, robot, ROBOT_RADIUS)
    x_wait = m.c[0]
    assert 1.0 < x_wait < 2.0 - 0.45 - 0.29 + 0.02, x_wait   # stops 0.3 m short
    for _ in range(300):                     # after 2 s it turns back
        m.advance(0.01, robot, ROBOT_RADIUS)
    assert m.c[0] < x_wait - 0.3
    blind = Mover(0.25, [[0, 0], [4, 0]], speed=1.0)
    blind.advance(2.0)                       # no robot given: walks straight through
    assert abs(blind.c[0] - 2.0) < 1e-9


@test
def test_free_distance_along_the_arc():
    hits = np.array([[3.0, y] for y in np.linspace(-2, 2, 81)])        # a wall at x = 3
    d = free_distance((0, 0, 0), 0.5, 0.0, hits, 0.3)
    assert abs(d - 2.7) < 0.06
    assert free_distance((0, 0, np.pi), 0.5, 0.0, hits, 0.3) == 3.0    # driving away
    # already too close: backing away is still allowed
    assert free_distance((2.8, 0, np.pi), 0.5, 0.0, hits, 0.3) > 1.0


@test
def test_vfh_blocks_the_direction_of_a_wall():
    s = AvoidState(method="VFH")
    hits = np.array([[1.0, y] for y in np.linspace(-1, 1, 41)])
    v, w, viz = vfh_cmd((0, 0, 0), hits, (5, 0), s.config(), ROBOT_RADIUS)
    n = len(viz["hist"])
    ahead = n // 2                           # sector of angle 0
    assert not viz["free"][ahead] and viz["free"][0]
    assert abs(viz["choice"]) > np.deg2rad(30)     # it turns away from the wall


@test
def test_local_minima_need_a_global_plan():
    """Heading straight for a goal behind the trap, every method stays
    trapped (without crashing); with the A* carrot, every one gets there."""
    for m in METHODS:
        sim = drive(world("2_trap.json"), m, GOAL_MODES[0], t_max=40)
        assert not sim.done and not sim.collided, m
        sim = drive(world("2_trap.json"), m, GOAL_MODES[1], t_max=60)
        assert sim.done and not sim.collided, m


@test
def test_dwa_and_vfh_get_through_the_people():
    """With the A* carrot, DWA and VFH reach the goal in every world,
    people and unmapped obstacles included, without touching anything."""
    for w in avoid_worlds():
        for m in ("DWA", "VFH"):
            sim = drive(w, m, GOAL_MODES[1], t_max=120)
            assert sim.done and not sim.collided, (w.name, m, sim.t)
            assert sim.min_clear > 0.03, (w.name, m, sim.min_clear)


@test
def test_the_potential_field_ignores_the_dynamics():
    """Straight at a pillar in the hall, the potential field only turns
    when the push catches up with the pull -- too late to stop."""
    sim = drive(world("4_hall.json"), "potential field", GOAL_MODES[0], t_max=20, people=0)
    assert sim.collided and sim.hit_by == "an obstacle"
    sim = drive(world("4_hall.json"), "DWA", GOAL_MODES[0], t_max=40, people=0)
    assert sim.done and not sim.collided


@test
def test_keys():
    s = AvoidState()
    assert s.method == "VFH"                  # the default
    s.method = "potential field"
    avoidkeys._MIN_REPEAT_INTERVAL = 0.0
    h, r = avoidkeys.make_handler(s)

    def press(k):
        h(N(key=k))
        r(N(key=k))

    names = lambda: {TUNABLES[i].name for i in s.visible()}
    assert "k_rep" in names() and "horizon" not in names() and "carrot" not in names()
    s.cursor = [t.name for t in TUNABLES].index("k_rep")
    press("c")
    assert s.method == "DWA" and "horizon" in names() and "k_rep" not in names()
    assert s.selected.name in names()        # the cursor moved off the hidden row
    press("m")
    assert s.goal_mode == GOAL_MODES[1] and "carrot" in names() and s.replan
    press("w")
    assert not s.polite
    press("p")
    assert not s.people_on and not s.config()["people_on"]


@test
def test_people_off_takes_them_out_of_the_world():
    """Without people, the lidar doesn't see them and nobody bumps into
    anybody: in the corridor, blind people can't hit the robot."""
    w = world("1_corridor.json")
    s = AvoidState(method="DWA", polite=False, people_on=False)
    sim = AvoidSim(w, ROBOT_RADIUS, np.random.default_rng(0))
    p = np.array([w.movers[0].c])
    cfg = s.config()
    sim.sense(cfg)
    assert w.distance(p)[0] > 0.0              # no person there any more
    sim.driving = True
    while sim.driving and sim.t < 60:
        sim.advance(0.5, cfg)
    assert sim.done and not sim.collided
    assert all(m.t == 0 for m in w.movers)     # and they didn't walk either


if __name__ == "__main__":
    failed = 0
    for fn in TESTS:
        try:
            fn()
            print(f"ok    {fn.__name__}")
        except Exception as exc:  # noqa: BLE001 -- report and keep going
            failed += 1
            print(f"FAIL  {fn.__name__}: {exc!r}")
    print(f"\n{len(TESTS) - failed}/{len(TESTS)} passed")
    sys.exit(1 if failed else 0)
