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
from navdemo.avoid import (AvoidSim, AvoidWorld, LocalMap, Mover, METHODS, GOAL_MODES,
                           avoid_worlds, free_distance, vfh_cmd, _vfh_direction)
from navdemo.mapping import Lidar
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


@test
def test_local_map_keeps_obstacles_where_they_are():
    """The window moves with the robot, but a remembered cell stays at the
    same place in the world -- until it falls out of the window."""
    m = LocalMap(size=6.0, res=0.1)
    one_hit = (np.array([0.0]), np.array([1.0]), np.array([True]))   # 1 m ahead
    m.update((2.0, 2.0, 0.0), one_hit, {}, 0.1)
    p0, _ = m.cells(0.5)
    assert len(p0) == 1 and np.allclose(p0[0], (3.05, 2.05))
    none = (np.array([0.0]), np.array([3.0]), np.array([False]))
    m.update((3.7, 2.4, 0.0), none, {}, 0.1)            # the robot has moved on
    p1, _ = m.cells(0.5)
    assert len(p1) == 1 and np.allclose(p1[0], p0[0])
    m.update((8.0, 2.0, 0.0), none, {}, 0.1)            # far away: dropped
    assert not len(m.cells(0.5)[0])


@test
def test_local_map_modes():
    """Hits only: a person's cell stays occupied after they have gone.
    Clearing along rays: it is free again as soon as a ray passes through.
    Forgetting: it fades by half every half-life."""
    hit = (np.array([0.0]), np.array([1.0]), np.array([True]))
    gone = (np.array([0.0]), np.array([3.0]), np.array([False]))      # the ray goes through
    for clear, left in ((False, 1.0), (True, 0.0)):
        m = LocalMap()
        m.update((0, 0, 0), hit, {"map_clear": clear}, 0.1)
        m.update((0, 0, 0), gone, {"map_clear": clear}, 0.1)
        c = m.cells(0.0)[1]
        assert (c.max() if len(c) else 0.0) == left, clear
    m = LocalMap()
    m.update((0, 0, 0), hit, {}, 0.1)
    for _ in range(20):                                # 2 s with a 2 s half-life
        m.update((0, 0, 0), gone, {"map_forget": True, "half_life": 2.0}, 0.1)
    assert abs(m.cells(0.0)[1].max() - 0.5) < 1e-9


@test
def test_field_of_view():
    w = world("4_hall.json")
    ang, _, _ = Lidar(3.0, 90, fov=np.deg2rad(90)).scan(w, 5, 5, 0.3)
    assert np.allclose([ang.min(), ang.max()], [0.3 - np.pi / 4, 0.3 + np.pi / 4])


@test
def test_vfh_goes_through_the_middle_of_a_door():
    """A narrow valley (a door): VFH heads for its middle, rather than for
    some wide open direction further from the goal."""
    n = 72
    centres = -np.pi + (np.arange(n) + 0.5) * 2 * np.pi / n
    free = np.zeros(n, bool)
    free[(np.abs(centres - 0.1) < np.deg2rad(8))] = True          # a door just left of ahead
    free[(np.abs(centres + np.pi / 2) < np.deg2rad(40))] = True   # wide open to the right
    c = _vfh_direction(free, centres, 2 * np.pi / n, 0.0)
    assert abs(c - 0.1) < np.deg2rad(3)
    free[:] = True
    assert _vfh_direction(free, centres, 2 * np.pi / n, 0.7) == 0.7


@test
def test_hits_only_leaves_ghosts_and_clearing_removes_them():
    """In the corridor, a map that only adds hits fills with the people's
    trails and the robot gets stuck; clearing along the rays gets it
    through."""
    s = AvoidState(method="VFH", goal_mode=GOAL_MODES[1], local_map=True)
    for clear, ok in ((False, False), (True, True)):
        s.map_clear = clear
        cfg = s.config()
        sim = AvoidSim(world("1_corridor.json"), ROBOT_RADIUS, np.random.default_rng(0))
        sim.plan_global(cfg)
        sim.driving = True
        while sim.driving and sim.t < 60:
            sim.advance(0.5, cfg)
        assert sim.done == ok and not sim.collided, clear


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
