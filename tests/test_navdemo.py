#!/usr/bin/env python3
"""Checks that the path geometry and the controllers do what they claim.

    python tests/test_navdemo.py

No test framework needed; it just asserts and prints.
"""

import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

from navdemo.params import DemoState, TUNABLES
from navdemo.path import Path, builtin_paths
from navdemo.sim import Robot, Follower, control_pure_pursuit, wrap_angle, PHYSICS_DT

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


L_SHAPE = Path([(0, 0), (1, 0), (1, 1)])


@test
def test_closest_point_and_side():
    x, y, s, e = L_SHAPE.closest(0.5, 0.2)
    assert np.allclose([x, y, s, e], [0.5, 0.0, 0.5, 0.2])      # left of the path: e > 0
    x, y, s, e = L_SHAPE.closest(1.3, 0.5)
    assert np.allclose([x, y, s, e], [1.0, 0.5, 1.5, -0.3])     # right of the path: e < 0


@test
def test_closest_respects_s_min():
    """A path that doubles back: the globally closest point is on the way
    back, but progress along the path must never go backwards."""
    p = Path([(0, 0), (2, 0), (2, 0.1), (0, 0.1)])
    _, _, s, _ = p.closest(0.5, 0.09, s_min=0.0, window=1.0)
    assert s < 1.5, s                         # stays on the outbound leg
    _, _, s, _ = p.closest(0.5, 0.09, s_min=0.0)
    assert s > 2.0, s                         # unrestricted: jumps to the return leg


@test
def test_point_at_and_extrapolation():
    assert np.allclose(L_SHAPE.point_at(0.5), [0.5, 0.0])
    assert np.allclose(L_SHAPE.point_at(1.5), [1.0, 0.5])
    assert np.allclose(L_SHAPE.point_at(2.5), [1.0, 1.5])       # past the end: straight on


@test
def test_pure_pursuit_arc_passes_through_target():
    """Driving the commanded curvature w/v from the robot's pose must hit
    the lookahead point -- that's the definition of pure pursuit."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        pose = (0.0, 0.0, rng.uniform(-np.pi, np.pi))
        target = rng.uniform(-1, 1, 2)
        v, w, alpha = control_pure_pursuit(pose, target, 1.0, np.inf)
        if abs(alpha) > np.deg2rad(170):
            continue
        kappa = w / v
        # centre of the arc, a distance 1/kappa to the robot's left
        c = np.array([-np.sin(pose[2]), np.cos(pose[2])]) / kappa
        assert np.isclose(np.hypot(*(target - c)), 1 / abs(kappa), rtol=1e-6)


@test
def test_acceleration_limits_respected():
    r = Robot()
    accV, accW = 2.0, np.deg2rad(360)
    vs, ws = [], []
    for _ in range(500):
        r.physics_step(1.0, 3.0, accV, accW)
        vs.append(r.v)
        ws.append(r.w)
    assert np.max(np.diff(vs)) <= accV * PHYSICS_DT + 1e-12
    assert np.max(np.diff(ws)) <= accW * PHYSICS_DT + 1e-12
    assert np.isclose(vs[-1], 1.0)


@test
def test_heading_integrates_w():
    """The MATLAB demo turned 10x too fast (a += w*dt with the control dt
    inside the physics loop). Here one second at w must turn exactly w rad."""
    r = Robot()
    r.w = 0.5
    for _ in range(int(round(1.0 / PHYSICS_DT))):
        r.physics_step(0.0, 0.5, 1.0, 1.0)
    assert np.isclose(r.a, 0.5, atol=1e-9)


@test
def test_default_settings_follow_every_builtin_path():
    state = DemoState()
    for law, ld in (("heading-P", 0.1), ("pure pursuit", 0.3)):
        state.law = law
        state.set_value("lookahead", ld)
        p = state.controller_params()
        for path in builtin_paths():
            f = Follower(path, Robot(*path.start, 0.0))
            while not f.done and f.t < 60:
                f.advance(0.1, p)
            assert f.done, (law, path.name)
            assert f.max_abs_e < 0.25, (law, path.name, f.max_abs_e)


@test
def test_ladders_and_set_value():
    s = DemoState()
    for t in TUNABLES:
        assert 0 <= t.default < len(t.ladder), t.name
    s.set_value("accw", np.deg2rad(360))
    assert np.isclose(s.value("accw"), np.deg2rad(360))
    s.set_value("sigma", np.inf)
    assert np.isinf(s.value("sigma"))
    s.step("lookahead", +100)
    assert s.value("lookahead") == TUNABLES[0].ladder[-1]


@test
def test_keys_and_mouse_drawing():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from types import SimpleNamespace
    from navdemo import keys, draw

    fig = plt.figure(figsize=(11, 7))
    ax = draw.setup_axes(fig, "test")
    state = DemoState()
    keys._MIN_REPEAT_INTERVAL = 0.0  # the test presses faster than any human
    handler, release = keys.make_handler(state, fig, ax)

    def press(k):
        handler(SimpleNamespace(key=k))
        release(SimpleNamespace(key=k))

    press("tab"); press("tab")
    assert state.cursor == 2
    press("backtab")
    assert state.cursor == 1
    before = state.idx["vmax"]
    handler(SimpleNamespace(key="<"))
    assert state.idx["vmax"] == before - 1
    press("c")
    assert state.law == "pure pursuit"
    press("3")
    assert state.newPath == 2
    press(" ")
    assert state.paused

    drawer = keys.PathDrawer(fig, ax, state)
    ev = lambda x, y: SimpleNamespace(inaxes=ax, button=1, xdata=x, ydata=y)
    drawer.press(ev(0, 0))
    for x in np.linspace(0.05, 1.0, 20):
        drawer.move(ev(x, 0.5 * x))
    drawer.release(ev(1.0, 0.5))
    assert isinstance(state.newPath, Path) and len(state.newPath.xy) == 21
    plt.close(fig)


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
