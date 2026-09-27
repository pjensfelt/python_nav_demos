#!/usr/bin/env python3
"""Checks for the world geometry, grids, planners and the mission loop.

    python tests/test_planning.py

No test framework needed; it just asserts and prints.
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

from navdemo.grid import Grid, GridChecker, GeometryChecker
from navdemo.mapping import Lidar, MappedGrid
from navdemo.mission import Mission
from navdemo.planners import astar, rrt, rrtstar, shortcut, path_length
from navdemo.planstate import PlanState, ROBOT_RADIUS
from navdemo.world import World, Circle, box, wall, builtin_worlds

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


EMPTY = World([], (0, 10, 0, 10), (1, 1, 0), (9, 9))
WALL = World([wall((5, 0), (5, 8), 0.2)], (0, 10, 0, 10), (1, 1, 0), (9, 1))


@test
def test_distances():
    c = Circle((0, 0), 1.0)
    assert np.allclose(c.distance(np.array([[0, 0], [2, 0], [0, -3]])), [0, 1, 2])
    b = box((0, 0), (2, 2), 45)          # a diamond reaching sqrt(2) along the axes
    assert np.isclose(b.distance(np.array([[2.0, 0.0]]))[0], 2 - np.sqrt(2))
    assert b.distance(np.array([[0.5, 0.5]]))[0] == 0.0
    assert np.isclose(WALL.segment_distance((4, 5), (6, 5)), 0.0)          # crosses the wall
    assert np.isclose(WALL.segment_distance((4, 9), (6, 9)), 1.0)          # passes above it
    assert np.isclose(EMPTY.distance(np.array([[1.0, 5.0]]))[0], 1.0)      # the boundary counts


@test
def test_grid_marks_obstacles_and_inflation():
    g = Grid.from_world(WALL, 0.1, 0.3)
    ix, iy = g.cell(5.0, 4.0)
    assert g.obstacle[ix, iy] and g.occ[ix, iy]
    ix, iy = g.cell(5.35, 4.0)     # 0.25 m from the wall face: inflation, not obstacle
    assert g.occ[ix, iy] and not g.obstacle[ix, iy]
    ix, iy = g.cell(5.55, 4.0)
    assert not g.occ[ix, iy]


@test
def test_grid_never_loses_a_thin_wall():
    """Cells are marked if they overlap an obstacle at all, so a 5 cm wall is
    kept at any cell size (it just gets as thick as the cells)."""
    thin = World([wall((3.0, 0), (3.0, 10), 0.05)], (0, 10, 0, 10), (1, 1, 0), (9, 1))
    for res in (0.1, 0.5, 1.0):
        g = Grid.from_world(thin, res, 0.0)
        assert not astar(g, thin.start[:2], thin.goal).path, res


@test
def test_cells_on_segment_are_connected():
    g = Grid(EMPTY.bounds, 0.1, 0.0)
    rng = np.random.default_rng(0)
    for _ in range(200):
        p, q = rng.uniform(0, 10, 2), rng.uniform(0, 10, 2)
        cells = g.cells_on_segment(*p, *q)
        assert cells[0] == g.cell(*p) and cells[-1] == g.cell(*q)
        steps = np.abs(np.diff(np.array(cells), axis=0)).sum(axis=1)
        assert (steps == 1).all()            # 4-connected, no skipped cells


@test
def test_astar_optimal_and_dijkstra_agree():
    g = Grid.from_world(WALL, 0.2, 0.3)
    a = astar(g, WALL.start[:2], WALL.goal, eight=True, weight=1.0)
    d = astar(g, WALL.start[:2], WALL.goal, eight=True, weight=0.0)
    assert a.path and d.path
    # same optimal cost on the grid (the real end points can shift the metric length slightly)
    assert abs(a.cost - d.cost) < 2 * g.res
    assert len(a.expanded) < len(d.expanded)
    w = astar(g, WALL.start[:2], WALL.goal, eight=True, weight=5.0)
    assert len(w.expanded) <= len(a.expanded)
    chk = GridChecker(g)
    for path in (a.path, d.path, w.path):
        assert all(chk.segment_free(p, q) for p, q in zip(path[1:-2], path[2:-1]))


@test
def test_astar_4_connected_is_longer():
    g = Grid.from_world(EMPTY, 0.5, 0.0)
    four = astar(g, (1, 1), (9, 9), eight=False)
    eight = astar(g, (1, 1), (9, 9), eight=True)
    assert four.cost > eight.cost * 1.3


@test
def test_rrt_and_rrtstar_paths_are_free():
    g = Grid.from_world(WALL, 0.1, 0.3)
    for chk in (GridChecker(g), GeometryChecker(WALL, 0.3)):
        r1 = rrt(chk, WALL.bounds, WALL.start[:2], WALL.goal, 3000, 0.5, rng=np.random.default_rng(1))
        r2 = rrtstar(chk, WALL.bounds, WALL.start[:2], WALL.goal, 3000, 0.5, radius=1.0,
                     rng=np.random.default_rng(1))
        for r in (r1, r2):
            assert r.path, r.message
            assert all(chk.segment_free(p, q) for p, q in zip(r.path, r.path[1:]))
            assert np.isclose(r.cost, path_length(r.path))
        assert r2.cost < r1.cost               # RRT* improves on RRT's first path


@test
def test_rrtstar_costs_stay_consistent():
    """After all the rewiring, every node's stored cost must equal the
    length of its path from the root -- the thing _propagate_cost guards."""
    from navdemo import planners
    chk = GridChecker(Grid.from_world(WALL, 0.1, 0.3))
    captured = {}
    orig = planners._tree_result

    def spy(tree, goal_node, events, iterations, t0):
        captured["tree"] = tree
        return orig(tree, goal_node, events, iterations, t0)

    planners._tree_result = spy
    try:
        rrtstar(chk, WALL.bounds, WALL.start[:2], WALL.goal, 1500, 0.5, radius=1.5,
                rng=np.random.default_rng(3))
    finally:
        planners._tree_result = orig
    tree = captured["tree"]
    for i in range(tree.n):
        assert np.isclose(tree.cost[i], path_length(tree.path_to(i)), atol=1e-9), i


@test
def test_shortcut_never_longer_and_still_free():
    g = Grid.from_world(WALL, 0.1, 0.3)
    chk = GridChecker(g)
    r = astar(g, WALL.start[:2], WALL.goal)
    s = shortcut(chk, r.path)
    assert path_length(s) <= r.cost + 1e-9 and len(s) < len(r.path)
    assert all(chk.segment_free(p, q) for p, q in zip(s, s[1:]))


@test
def test_lidar_ranges():
    lid = Lidar(max_range=20.0, n_rays=4)
    ang, rng_, hit = lid.scan(EMPTY, 2.0, 5.0, 0.0)     # rays at -180, -90, 0, 90 deg
    assert hit.all()
    assert np.allclose(sorted(rng_), sorted([2.0, 5.0, 8.0, 5.0]), atol=2e-3)
    ang, rng_, hit = Lidar(max_range=3.0, n_rays=4).scan(EMPTY, 2.0, 5.0, 0.0)
    assert hit.sum() == 1 and np.isclose(rng_.max(), 3.0)


@test
def test_mapped_grid_fills_in():
    g = MappedGrid(WALL.bounds, 0.1, 0.3)
    assert not g.known.any() and not g.occ.any()
    x, y = 3.0, 4.0
    new = g.integrate(x, y, *Lidar(5.0, 360).scan(WALL, x, y, 0.0))
    assert new and g.occ[g.cell(4.95, 4.0)]
    assert g.known[g.cell(4.0, 4.0)] and not g.occ[g.cell(4.0, 4.0)]
    assert not g.known[g.cell(9.0, 9.0)]        # behind the wall: never seen


def _cfg(**kw):
    s = PlanState()
    cfg = s.mission_config()
    cfg.update(kw)
    return cfg


def _run(world, cfg, seed=0, t_max=150, shortcut=False):
    m = Mission(world, ROBOT_RADIUS, np.random.default_rng(seed))
    m.build_map(cfg)
    m.plan(cfg)
    if shortcut:
        m.set_path_mode("shortcut", cfg)       # as 's' does for the current plan
    m.driving = bool(m.path)
    while m.driving and m.t < t_max:
        m.advance(0.1, cfg)
    return m


@test
def test_every_world_known_and_mapped():
    for w in builtin_worlds():
        for mapped in (False, True):
            m = _run(w, _cfg(mapped=mapped))
            assert m.done and not m.collided, (w.name, mapped, m.failed)


@test
def test_stops_at_the_goal():
    for w in builtin_worlds():
        m = _run(w, _cfg())
        assert np.hypot(m.robot.x - m.goal[0], m.robot.y - m.goal[1]) < 0.06, w.name


@test
def test_imperfect_buildings_still_work():
    """With the default 2 cm imperfection, every world still gets done, known
    and mapped -- the variant changes the details, not whether it works."""
    from navdemo.world import imperfect
    for variant in (0, 1):
        for w0 in builtin_worlds():
            w = imperfect(w0, 0.02, np.random.default_rng([0, 7, variant]))
            for mapped in (False, True):
                m = _run(w, _cfg(mapped=mapped))
                assert m.done and not m.collided, (w.name, variant, mapped)


@test
def test_dead_end_costs_extra_when_mapping():
    w = World.load("8_deadend.json")
    known = _run(w, _cfg(mapped=False))
    mapped = _run(w, _cfg(mapped=True))
    assert known.done and mapped.done
    assert mapped.replans > 0 and mapped.driven > known.driven + 3.0


@test
def test_no_inflation_means_collision():
    """Planning the robot as a point without growing the obstacles by its
    radius: the plan grazes the wall and the real robot hits it."""
    m = _run(World.load("1_gap.json"), _cfg(inflate=0.0))
    assert m.collided


@test
def test_exact_geometry_toggle():
    """x only applies to RRT/RRT* with a known map -- and must not break A*,
    which can't use it (it once handed A* a GeometryChecker and crashed)."""
    s = PlanState(exact_geometry=True)
    assert not s.uses_exact_geometry                 # A* selected
    w = World.load("2_narrow.json")
    m = _run(w, dict(s.mission_config(), res=0.5))   # A* on the grid: the door is closed
    assert not m.path and not m.done
    s.planner = "RRT*"
    assert s.uses_exact_geometry
    cfg = dict(s.mission_config(), res=0.5)
    m = _run(w, cfg, seed=1)
    assert m.done and not m.collided                 # through the real door
    s.mapped = True
    assert not s.uses_exact_geometry                 # mapping: the grid is all there is


@test
def test_turn_in_place_in_bug_trap():
    w = World.load("5_bugtrap.json")
    fast = _run(w, _cfg(law="pure pursuit"))                      # on by default here
    slow = _run(w, _cfg(law="pure pursuit", turn_in_place=False))
    assert fast.done and fast.t < 20 and slow.t > 2 * fast.t, (fast.t, slow.t)


@test
def test_rotated_world_known_map():
    """The world turned under the grid: the gap world still works at 30
    degrees, but the narrow passage's door closes -- rotation is where a
    grid hurts most."""
    from navdemo.world import moved, centre
    gap, narrow = World.load("1_gap.json"), World.load("2_narrow.json")
    m = _run(moved(gap, (0, 0), np.deg2rad(30), centre(gap)), _cfg())
    assert m.done and not m.collided
    m = _run(moved(narrow, (0, 0), np.deg2rad(45), centre(narrow)), _cfg())
    assert not m.path


@test
def test_known_map_is_made_from_samples():
    """Sparse samples leave holes, so the known map has fewer occupied cells
    than a dense survey; noise spreads them over more."""
    from navdemo.grid import known_map
    from navdemo.rasterize import sample_outlines
    w = World.load("6_rooms.json")
    occ = {}
    for name, spacing, sigma in (("dense", 0.02, 0.0), ("sparse", 0.5, 0.0), ("noisy", 0.02, 0.1)):
        P = sample_outlines(w.obstacles, w.room, spacing, sigma, np.random.default_rng(0))
        occ[name] = known_map(w, P, 0.1, 0.0).obstacle.sum()
    assert occ["sparse"] < occ["dense"] < occ["noisy"]


@test
def test_planning_keys_rotate_and_restore():
    from types import SimpleNamespace as N
    from navdemo import plankeys
    plankeys._MIN_REPEAT_INTERVAL = 0.0
    s = PlanState()
    h, r = plankeys.make_handler(s)

    def press(k):
        h(N(key=k))
        r(N(key=k))

    for k in (".", ".", "l", ","):
        press(k)
    assert np.isclose(np.rad2deg(s.world_angle), 6)
    s.start, s.goal = (1.0, 1.0), (2.0, 2.0)
    press("0")
    assert s.world_angle == 0 and s.start is None and s.goal is None
    press("m")
    assert s.mapped
    draw = s.sample_draw
    press("u")
    assert s.sample_draw == draw + 1


@test
def test_execution_toggle():
    """Planning first: execution starts off, its rows are hidden and space
    does nothing until 'e'."""
    from types import SimpleNamespace as N
    from navdemo import plankeys
    from navdemo.planstate import TUNABLES
    plankeys._MIN_REPEAT_INTERVAL = 0.0
    s = PlanState()
    h, r = plankeys.make_handler(s)

    def press(k):
        h(N(key=k))
        r(N(key=k))

    names = lambda: {TUNABLES[i].name for i in s.visible()}
    assert not s.execute and "vmax" not in names()
    press(" ")
    assert not s.drive
    press("e")
    assert s.execute and "vmax" in names()
    press(" ")
    assert s.drive


@test
def test_stop_and_turn_is_exact_but_slow():
    """Stop and turn follows the planned path exactly -- and stops at every
    corner, which makes it much slower than pure pursuit; shortcutting the
    path first removes most of the corners."""
    w = World.load("1_gap.json")
    times = {}
    for law, sc in (("pure pursuit", False), ("stop and turn", False), ("stop and turn", True)):
        m = _run(w, _cfg(law=law), t_max=200, shortcut=sc)
        assert m.done and not m.collided, (law, sc)
        times[(law, sc)] = m.t
        if law == "stop and turn":
            assert m.follower.stops > 0 and m.follower.max_abs_e < 0.05
    assert times[("stop and turn", False)] > 1.5 * times[("pure pursuit", False)]
    assert times[("stop and turn", True)] < times[("stop and turn", False)]


@test
def test_s_cycles_the_smoothing_of_the_current_path():
    """'s' picks how the plan is smoothed -- no new search, the plan stays
    as it was: spline, shortcut, shortcut + spline, and back. Every version
    is still free on the map, and the splines have finite curvature."""
    from types import SimpleNamespace as N
    from navdemo import plankeys
    from navdemo.grid import GridChecker
    from navdemo.planners import PATH_MODES
    w = World.load("1_gap.json")
    state = PlanState()
    plankeys._MIN_REPEAT_INTERVAL = 0.0
    h, rel = plankeys.make_handler(state)
    m = Mission(w, ROBOT_RADIUS, np.random.default_rng(0))
    m.build_map(state.mission_config())
    res = m.plan(state.mission_config())
    assert m.path == res.path and not m.shortened and m.curvature is None
    lengths = {}
    for mode in PATH_MODES[1:] + PATH_MODES[:1]:
        h(N(key="s")); rel(N(key="s"))
        assert state.path_mode == mode and state.restyle
        assert m.set_path_mode(state.path_mode, state.mission_config()) and m.path_mode == mode
        assert m.result is res and not m.old_paths
        chk = GridChecker(m.grid)
        assert all(chk.segment_free(a, b) for a, b in zip(m.path, m.path[1:])), mode
        assert np.allclose(m.path[0], res.path[0]) and np.allclose(m.path[-1], res.path[-1])
        assert ("spline" in mode) == (m.curvature is not None), mode
        lengths[mode] = path_length(m.path)
    assert lengths["shortcut"] < res.cost and lengths["shortcut + spline"] < lengths["shortcut"]
    assert m.path == res.path and not m.shortened


@test
def test_smoothing_applies_to_every_replan_and_mid_drive():
    """The selected smoothing is applied to every plan, replans while
    mapping as we go included; changed while driving, it smooths the rest
    of the plan from where the robot is, so it never drives back."""
    w = World.load("8_deadend.json")
    cfg = _cfg(mapped=True, path_mode="shortcut + spline")
    m = Mission(w, ROBOT_RADIUS, np.random.default_rng(0))
    m.build_map(cfg)
    m.plan(cfg)
    assert m.path_mode == "shortcut + spline"
    m.driving = True
    modes = []
    while m.driving and m.t < 150:
        before = m.result
        m.advance(0.1, cfg)
        if m.result is not before:
            modes.append(m.path_mode)
    assert m.done and not m.collided and m.replans > 0
    assert modes and all(md == "shortcut + spline" for md in modes)

    # changed half way: the new path starts at the robot
    w = World.load("1_gap.json")
    cfg = _cfg()
    m = Mission(w, ROBOT_RADIUS, np.random.default_rng(0))
    m.build_map(cfg)
    m.plan(cfg)
    m.driving = True
    while m.t < 6:
        m.advance(0.1, cfg)
    x, y = m.robot.x, m.robot.y
    left = m.result.cost - m.follower.s
    m.set_path_mode("shortcut + spline", cfg)
    assert np.allclose(m.path[0], (x, y)) and path_length(m.path) < left
    while m.driving and m.t < 60:
        m.advance(0.1, cfg)
    assert m.done and not m.collided and m.driven < 18


@test
def test_spline_tightens_where_it_would_hit():
    """Rounding a corner cuts inside it; where that would hit the map the
    control points are put closer together, down to a limit."""
    from navdemo.planners import spline

    class Box:            # free everywhere except a block inside the corner
        def segment_free(self, a, b):
            return not any(1.7 < x < 1.98 and 0.02 < y < 0.3
                           for x, y in np.linspace(a, b, 5))
    pts, used = spline(Box(), [(0, 0), (2, 0), (2, 2)], 1.0)
    assert pts is not None and used < 1.0
    free, used_free = spline(type("Free", (), {"segment_free": lambda s, a, b: True})(),
                             [(0, 0), (2, 0), (2, 2)], 1.0)
    assert used_free == 1.0


@test
def test_cost_near_obstacles_buys_clearance():
    """With the smoothed cost, A* keeps further from obstacles for a longer
    path -- and a narrow door, the only way through, stays usable."""
    from navdemo.path import Path
    from navdemo.planners import smoothed_cost

    occ = np.zeros((21, 21), bool)
    occ[10, 10] = True
    c = smoothed_cost(occ, 0.1, 0.3)
    assert c[10, 10] == c.max() and c[0, 0] < 1e-6 and 0 <= c.min() and c.max() <= 1

    def clearance(world, cw):
        cfg = _cfg(cost_weight=cw)
        m = Mission(world, ROBOT_RADIUS, np.random.default_rng(0))
        m.build_map(cfg)
        r = m.plan(cfg)
        assert r.path, (world.name, cw)
        P = Path(r.path)
        pts = np.array([P.point_at(s) for s in np.linspace(0, P.length, 500)])
        return r.cost, world.distance(pts).min()

    w = World.load("1_gap.json")
    (l0, d0), (l5, d5) = clearance(w, 0), clearance(w, 5)
    assert l5 > l0 and d5 > d0 + 0.2
    clearance(World.load("2_narrow.json"), 20)      # still gets through the door


@test
def test_goal_in_an_occupied_cell_is_reported_not_a_crash():
    """Every planner refuses a goal inside an obstacle -- and the panel,
    legend and search display cope with a result that searched nothing."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from navdemo import plandraw
    w = World.load("1_gap.json")
    fig = plt.figure()
    ax = plandraw.setup_axes(fig, "test")
    panel, legend, search = plandraw.Panel(fig), plandraw.Legend(fig), plandraw.SearchArtist(ax)
    for planner in ("A*", "RRT", "RRT*", "potential field"):
        s = PlanState()
        s.planner, s.execute = planner, True
        cfg = s.mission_config()
        m = Mission(w, ROBOT_RADIUS, np.random.default_rng(0))
        m.build_map(cfg)
        m.goal = (5.0, 2.0)                   # on the middle wall
        r = m.plan(cfg)
        assert not r.path and r.message and m.failed, planner
        panel.update(s, m, m.failed)
        legend.update(s, m)
        search.set(r, m.grid, search.total(r))
        assert r.message in panel.text.get_text() or "nodes" in panel.text.get_text()
    plt.close(fig)


@test
def test_potential_field():
    """Through the clutter world it finds a way; in front of a wall it gets
    stuck in a local minimum and says so; it can't plan without the
    known map's samples. The trace is a chain, replayed like a tree."""
    cfg = _cfg(planner="potential field")
    m = _run(World.load("4_clutter.json"), cfg)
    assert m.done and not m.collided
    r = m.result
    assert r.nodes is not None and len(r.events) == len(r.nodes) - 1
    assert all(c == p + 1 for c, p in r.events)

    m = Mission(World.load("1_gap.json"), ROBOT_RADIUS, np.random.default_rng(0))
    m.build_map(cfg)
    r = m.plan(cfg)
    assert not r.path and "local minimum" in r.message
    assert abs(r.nodes[-1][0] - 5.0) < 1.0          # stuck at the middle wall

    cfg = _cfg(planner="potential field", mapped=True)
    m = Mission(World.load("4_clutter.json"), ROBOT_RADIUS, np.random.default_rng(0))
    m.build_map(cfg)
    assert not m.plan(cfg).path and "sample points" in m.failed


@test
def test_plan_keys():
    s = PlanState()
    k = s.plan_key()
    s.step("iterations", +1)          # not an A* setting: plan stays valid
    assert s.plan_key() == k
    s.step("h_weight", +1)
    assert s.plan_key() != k
    g = s.grid_key()
    s.mapped = True
    assert s.grid_key() != g


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
