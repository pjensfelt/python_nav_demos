#!/usr/bin/env python3
"""Checks for run_grid.py: exact any-overlap cells, moving the world, and
the passability flood fill.

    python tests/test_grid.py

No test framework needed; it just asserts and prints.
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

from navdemo.rasterize import connected, rasterize, square_distance, transform_obstacles
from navdemo.world import Circle, World, box, grid_worlds, wall

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


BOUNDS = (0, 6, 0, 6)


def _brute_square_distance(ob, C, h, n=41):
    s = np.linspace(-h, h, n)
    SX, SY = np.meshgrid(s, s)
    pts = np.column_stack([SX.ravel(), SY.ravel()])
    return np.array([ob.distance(c + pts).min() for c in C])


@test
def test_square_distance_matches_brute_force():
    rng = np.random.default_rng(0)
    C = rng.uniform(0, 5, (1500, 2))
    h = 0.25
    for ob in (wall((1, 1), (4, 3.3), 0.05), Circle((2.2, 2.7), 0.3), box((3, 1.2), (0.6, 0.2), 20)):
        exact = square_distance(ob, C, h)
        brute = _brute_square_distance(ob, C, h)
        # brute force samples the square, so it can only overestimate, by at most its spacing
        assert (exact <= brute + 1e-9).all()
        assert np.abs(exact - brute).max() < 2 * h / 40 * np.sqrt(2)
        assert ((exact <= 1e-9) == (brute <= 1e-9)).all()


@test
def test_any_overlap_never_misses_a_thin_wall():
    thin = [wall((3.1, 0), (3.1, 6), 0.03)]
    for res in (0.1, 0.25, 0.5, 1.0):
        g = rasterize(thin, BOUNDS, res, 0.0)
        assert 1 <= g.occ.any(axis=1).sum() <= 2   # one column, two where it straddles a border
        assert not connected(g, (1, 3), (5, 3))[0]


@test
def test_any_overlap_is_exact_not_half_diagonal():
    """A circle whose edge comes within half a diagonal of a cell centre, but
    doesn't touch the cell, must leave it free. (grid.py's old quick
    'conservative' rule, adding half the cell diagonal to a centre test,
    marked it.) That happens off a cell's *edge*: towards a corner the
    half-diagonal test happens to be exact."""
    from navdemo.grid import Grid
    # cell [0.5, 1.0] x [0.5, 1.0], centre (0.75, 0.75); the circle is 5 cm
    # to the right of its edge x = 1.0, but only 0.30 m from its centre
    c = Circle((1.45, 0.75), 0.4)
    assert 0.30 < 0.5 * np.sqrt(2) / 2
    g = rasterize([c], BOUNDS, 0.5, 0.0)
    assert not g.occ[1, 1]
    w = World([c], BOUNDS, (0.2, 0.2, 0), (5, 5))
    assert not Grid.from_world(w, 0.5, 0.0).occ[1, 1]   # the planning demo's grid is exact too


@test
def test_wall_cell_count_depends_on_offset():
    """A 0.2 m wall at 0.5 m cells covers 1 or 2 columns depending on where
    the cell borders fall -- ceil(w/c) or ceil(w/c) + 1."""
    counts = set()
    for u in np.linspace(0, 1, 26)[:-1]:
        obs, _ = transform_obstacles([wall((3.0, 0), (3.0, 6), 0.2)], (3, 3), (u * 0.5, 0), 0)
        g = rasterize(obs, BOUNDS, 0.5, 0.0)
        counts.add(int(g.occ.any(axis=1).sum()))
    assert counts == {1, 2}


@test
def test_door_opens_and_closes_with_offset():
    door = grid_worlds()[0]
    assert door.name == "door"
    opened = []
    for u in np.linspace(0, 1, 21)[:-1]:
        obs, move = transform_obstacles(door.obstacles, (3, 3), (0, u * 0.5), 0)
        g = rasterize(obs, door.bounds, 0.5, 0.0)
        name, a, b = door.probes[0]
        opened.append(connected(g, move(a)[0], move(b)[0])[0])
    assert any(opened) and not all(opened)
    assert not opened[0]           # closed at the start: the lecture's opening surprise


BIG = (-2, 8, -2, 8)     # a grid covering the 6 x 6 m room at any angle


def _open(w, res, offset=(0, 0), angle=0.0):
    from navdemo.rasterize import room_walls
    obs, move = transform_obstacles(w.obstacles + room_walls(w.room), (3, 3), offset, angle)
    g = rasterize(obs, BIG, res, 0.0)
    name, a, b = w.probes[0]
    return connected(g, move(a)[0], move(b)[0])[0]


@test
def test_rotation_in_the_door_world():
    """At 0.5 m cells the rotated door is closed at every angle; at 0.25 m it
    is open at every one."""
    door = grid_worlds()[0]
    angles = np.deg2rad(np.arange(0, 46, 3))
    assert not any(_open(door, 0.5, angle=-a) for a in angles)
    assert all(_open(door, 0.25, angle=-a) for a in angles)


@test
def test_diagonal_corridor_closes_with_coarse_cells():
    """The 0.6 m corridor along the diagonal: open up to 0.2 m cells, closed
    from 0.3 m, and in between it depends on where the cell borders fall."""
    w = grid_worlds()[2]
    assert w.name == "diagonal corridor"
    shifts = [(u * 0.37, u) for u in np.linspace(0, 1, 11)[:-1]]
    for res in (0.1, 0.2):
        assert all(_open(w, res, (dx * res, dy * res)) for dx, dy in shifts)
    assert not any(_open(w, 0.3, (dx * 0.3, dy * 0.3)) for dx, dy in shifts)
    at_quarter = [_open(w, 0.25, (dx * 0.25, dy * 0.25)) for dx, dy in shifts]
    assert any(at_quarter) and not all(at_quarter)
    # rotated 45 degrees, the corridor is aligned with the grid: at 0.25 m
    # cells, where the diagonal one depends on luck, the aligned one is open
    assert all(_open(w, 0.25, (u * 0.25, 0), angle=np.deg2rad(45))
               for u in np.linspace(0, 1, 6)[:-1])


@test
def test_transform_rotation_about_pivot():
    obs, move = transform_obstacles([Circle((4, 3), 0.1)], (3, 3), (0, 0), np.pi / 2)
    assert np.allclose(obs[0].c, [3, 4])
    assert np.allclose(move((3, 3))[0], [3, 3])


@test
def test_flood_fill_no_corner_squeeze():
    """Two occupied cells touching at a corner must block a diagonal step."""
    obs = [box((2.75, 2.75), (0.5, 0.5)), box((3.25, 3.25), (0.5, 0.5)),
           wall((3.0, 0), (3.0, 2.5), 0.1), wall((3.0, 3.5), (3.0, 6), 0.1)]
    g = rasterize(obs, BOUNDS, 0.5, 0.0)
    assert not connected(g, (1, 1), (5, 5))[0]


@test
def test_rotating_grid_equals_rotating_world_back():
    """Only the relative pose decides the cells: turning the grid by +a
    over a fixed world gives the same cells as turning the world by -a
    under a fixed grid."""
    from navdemo.gridstate import GridState, Pose
    w = grid_worlds()[2]
    s = GridState()
    s.set_value("res", 0.5)

    def cells(state):
        dx, dy, a = state.relative()
        obs, _ = transform_obstacles(w.obstacles, (3, 3), (dx, dy), a)
        return rasterize(obs, w.bounds, 0.5, 0.0).occ

    s.grid = Pose(np.array([0.13, -0.07]), np.deg2rad(20))
    by_grid = cells(s)
    s.grid = Pose()
    s.world = Pose(-(np.array([[np.cos(-np.deg2rad(20)), -np.sin(-np.deg2rad(20))],
                              [np.sin(-np.deg2rad(20)), np.cos(-np.deg2rad(20))]])
                     @ np.array([0.13, -0.07])), -np.deg2rad(20))
    assert (cells(s) == by_grid).all()


@test
def test_grid_keys():
    from types import SimpleNamespace as N
    from navdemo import gridstate
    gridstate._MIN_REPEAT_INTERVAL = 0.0
    s = gridstate.GridState()
    h, r = gridstate.make_handler(s)

    def press(k):
        h(N(key=k))
        r(N(key=k))

    assert s.value("res") == 0.1                    # starts fine
    assert not s.planning                           # and without planning
    press("p")
    assert s.planning
    shown = s.show_samples
    press("d")
    assert s.show_samples != shown
    press("right"); press("]")
    assert np.isclose(s.grid.offset[0], 0.01) and np.isclose(s.grid.angle, np.deg2rad(1))
    press("."); press("."); press(",")
    assert np.isclose(s.grid.angle, np.deg2rad(2))
    press("l"); press("l"); press("k")
    assert np.isclose(s.grid.angle, np.deg2rad(7))
    press("w"); press("up")
    assert s.moving == "world" and np.isclose(s.world.offset[1], 0.01)
    press("r")
    assert s.sweep_kind == "r"
    s.start, s.goal = (1.0, 1.0), (5.0, 5.0)        # as if clicked
    press("0")
    assert not s.grid.offset.any() and s.grid.angle == 0 and not s.world.offset.any()
    assert s.start is None and s.goal is None       # back to the designed start and goal


@test
def test_touching_is_not_overlapping():
    """A 0.2 m wall whose faces lie exactly on cell borders covers two 0.1 m
    columns, not four."""
    g = rasterize([wall((3.0, 0), (3.0, 6), 0.2)], BOUNDS, 0.1, 0.0)
    assert g.occ.any(axis=1).sum() == 2


@test
def test_grid_first_inflation_is_a_superset():
    """Inflating the occupied cells can only add cells compared to
    inflating the real geometry -- including near the room walls, which lie
    just outside the grid (that once went missing)."""
    from navdemo.rasterize import room_walls
    rng = np.random.default_rng(1)
    for w in grid_worlds()[:3]:
        for res, inf in ((0.1, 0.1), (0.25, 0.1), (0.25, 0.3), (0.5, 0.2)):
            off = rng.uniform(0, res, 2)
            obs, _ = transform_obstacles(w.obstacles + room_walls(w.bounds), (3, 3), off,
                                         rng.uniform(0, 0.8))
            wf = rasterize(obs, w.bounds, res, inf, "world first")
            gf = rasterize(obs, w.bounds, res, inf, "grid first")
            assert not (wf.occ & ~gf.occ).any(), (w.name, res, inf)
    # and it matters when cells are coarse compared to the inflation
    door = grid_worlds()[0]
    obs = door.obstacles + room_walls(door.bounds)
    name, a, b = door.probes[0]
    wf = rasterize(obs, door.bounds, 0.25, 0.1, "world first")
    gf = rasterize(obs, door.bounds, 0.25, 0.1, "grid first")
    assert connected(wf, a, b)[0] and not connected(gf, a, b)[0]


@test
def test_room_walls_stop_detours():
    """With the grid rotated past the world's edge, a connection must not go
    round the end of a wall that runs to the boundary."""
    from navdemo.rasterize import room_walls
    from navdemo.planners import path_length
    door = grid_worlds()[0]
    obs, move = transform_obstacles(door.obstacles + room_walls(door.bounds), (3, 3), (0, 0),
                                    np.deg2rad(-30))
    g = rasterize(obs, door.bounds, 0.25, 0.0)
    name, a, b = door.probes[0]
    ok, why, reached, path = connected(g, move(a)[0], move(b)[0])
    if ok:
        assert path_length(path) < 1.5 * np.hypot(*(np.array(a) - b))


@test
def test_no_squeezing_between_corners_and_path():
    """A staircase connected only at corners is blocked; and the path found
    runs from a to b."""
    obs = [box((1.25, 1.25), (0.5, 0.5)), box((1.75, 1.75), (0.5, 0.5))]
    g = rasterize(obs, (0, 3, 0, 3), 0.5, 0.0)
    ok, why, reached, path = connected(g, (1.75, 1.25), (1.25, 1.75))
    assert not ok or len(path) > 2               # never the one diagonal squeeze
    ok, why, reached, path = connected(g, (0.25, 0.25), (2.75, 2.75))
    assert ok and path[0] == (0.25, 0.25) and path[-1] == (2.75, 2.75)


@test
def test_samples_lie_on_the_outlines():
    from navdemo.rasterize import sample_outlines
    obs = [Circle((2, 2), 0.5), box((4, 4), (1.0, 0.4), 30)]
    P = sample_outlines(obs, BOUNDS, 0.05, 0.0, np.random.default_rng(0))
    on_circle = np.abs(np.hypot(*(P - [2, 2]).T) - 0.5) < 1e-6
    assert on_circle.sum() == int(np.ceil(2 * np.pi * 0.5 / 0.05))
    rest = P[~on_circle]
    # the box's and the room's outlines: every point is on one of them, spaced ~0.05 apart
    dist_box = obs[1].distance(rest)
    on_room = (np.isclose(rest[:, 0], 0, atol=1e-6) | np.isclose(rest[:, 0], 6)
               | np.isclose(rest[:, 1], 0) | np.isclose(rest[:, 1], 6))
    assert ((dist_box < 1e-6) | on_room).all()
    assert abs(on_room.sum() - 24 / 0.05) <= 1


@test
def test_noiseless_dense_samples_stay_inside_any_overlap():
    """Without noise every sample is on an obstacle, so every cell it marks
    also overlaps the obstacle -- a subset of the any-overlap cells (hollow,
    so not equal)."""
    from navdemo.rasterize import rasterize_samples, room_walls, sample_outlines
    w = grid_worlds()[7]                       # clutter
    P = sample_outlines(w.obstacles, w.bounds, 0.02, 0.0, np.random.default_rng(0))
    for res in (0.1, 0.25):
        smp = rasterize_samples(P, w.bounds, res, 0.0).occ
        ovl = rasterize(w.obstacles + room_walls(w.bounds), w.bounds, res, 0.0).occ
        # a sample exactly on a cell border lands in one of the two cells,
        # which "any overlap" may not count (it ignores mere touching)
        extra = smp & ~ovl
        assert extra.sum() <= 0.01 * smp.sum(), (res, extra.sum())
        assert (ovl & ~smp).sum() > 0          # hollow: the insides stay free


@test
def test_sparse_or_noisy_samples_leak_on_fine_grids():
    """Too few points, or points spread by noise over cells smaller than the
    noise, leave holes in the thin partition, and the planned path goes
    straight through it instead of round by the doorway. Bigger cells
    gather enough hits each."""
    from navdemo.planners import astar, path_length
    from navdemo.rasterize import rasterize_samples, room_walls, sample_outlines
    w = grid_worlds()[3]
    assert w.name == "thin partition"
    name, a, b = w.probes[0]
    right = path_length(astar(rasterize(w.obstacles + room_walls(w.room), w.bounds, 0.1, 0.0),
                              a, b).path)
    assert right > 2 * np.hypot(*(np.array(a) - b))      # the way round, by the doorway

    def leak_rate(res, spacing, sigma, draws=10):
        leaks = 0
        for seed in range(draws):
            P = sample_outlines(w.obstacles, w.room, spacing, sigma, np.random.default_rng(seed))
            path = astar(rasterize_samples(P, w.bounds, res, 0.0), a, b).path
            leaks += bool(path) and path_length(path) < 0.7 * right
        return leaks / draws

    assert leak_rate(0.1, 0.05, 0.0) == 0
    assert leak_rate(0.1, 0.5, 0.0) == 1          # sparse
    assert leak_rate(0.1, 0.1, 0.05) >= 0.8       # noise on a fine grid
    assert leak_rate(0.5, 0.1, 0.05) == 0         # same samples, coarse cells


@test
def test_samples_rule_state():
    from types import SimpleNamespace as N
    from navdemo import gridstate
    gridstate._MIN_REPEAT_INTERVAL = 0.0
    s = gridstate.GridState()
    h, r = gridstate.make_handler(s)

    def press(k):
        h(N(key=k))
        r(N(key=k))

    names = lambda: [gridstate.TUNABLES[i].name for i in s.visible()]
    assert "spacing" not in names()
    press("m")
    assert s.rule == "samples" and "spacing" in names()
    assert s.effective_inflate_order == "grid first"
    draw = s.sample_draw
    press("u")
    assert s.sample_draw == draw + 1
    for _ in range(3):
        press("tab")                           # cursor onto a samples-only row
    assert gridstate.TUNABLES[s.cursor].name in gridstate.SAMPLE_ONLY
    press("m")
    assert s.rule == "any overlap" and s.effective_inflate_order == "grid first"
    assert s.cursor in s.visible()             # not left on a hidden row
    for _ in range(6):
        press("tab")
        assert s.cursor in s.visible()


@test
def test_imperfect_worlds():
    from navdemo.world import imperfect
    w = grid_worlds()[0]
    assert imperfect(w, 0.0, np.random.default_rng(0)) is w          # 0 = as drawn
    a = imperfect(w, 0.02, np.random.default_rng([0, 7, 0]))
    b = imperfect(w, 0.02, np.random.default_rng([0, 7, 0]))
    c = imperfect(w, 0.02, np.random.default_rng([0, 7, 1]))
    assert np.allclose(a.room, b.room) and not np.allclose(a.room, c.room)   # repeatable variants
    moved = np.concatenate([(pa.v - p0.v).ravel() for pa, p0 in zip(a.obstacles, w.obstacles)]
                           + [(a.room - w.room).ravel()])
    assert 0.005 < moved.std() < 0.05                                 # centimetres, not metres
    lo, hi = a.room.min(axis=0), a.room.max(axis=0)
    assert np.allclose(a.bounds, (lo[0], hi[0], lo[1], hi[1]))
    assert a.probes == w.probes and a.start == w.start                 # design points stay


@test
def test_room_walls_exact_for_a_skewed_room():
    """The room as one 'outside' obstacle: its cell test matches brute force
    for a room that isn't square."""
    from navdemo.world import Outside
    room = Outside([(0.1, -0.05), (5.9, 0.12), (6.05, 6.0), (-0.08, 5.93)])
    rng = np.random.default_rng(3)
    C = rng.uniform(-0.5, 6.5, (800, 2))
    exact = square_distance(room, C, 0.25)
    brute = _brute_square_distance(room, C, 0.25)
    assert ((exact <= 1e-9) == (brute <= 1e-9)).all()
    assert np.abs(exact - brute).max() < 0.02


@test
def test_planning_world_list_unchanged():
    from navdemo.world import builtin_worlds
    assert [w.name for w in builtin_worlds()][:2] == ["gap", "narrow passage"]
    assert len(builtin_worlds()) == 8


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
