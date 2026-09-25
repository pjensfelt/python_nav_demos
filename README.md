# Python navigation demos

Demos of path following for navigation, used during lectures. They are a
Python port of the MATLAB `matlab_pure_pursuit` demo, and share their window
layout and keyboard conventions with the localization/SLAM demos in
[python_loc_demos](https://github.com/pjensfelt/python_loc_demo).

Repository: `git@github.com:pjensfelt/python_nav_demos.git`

This repo stands alone. It sits next to `python_loc_demos/` in the same
parent folder but does not import from it. The few helpers both need
(screenshots, the keymap reset, key debouncing) are copied into `navdemo/`.

## Getting started

```sh
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

.venv/bin/python run_pure_pursuit.py            # pure pursuit path following
.venv/bin/python tests/test_navdemo.py          # checks
```

The figure window has to have keyboard focus for the keys to work. Press `h`
for the key list at any time.

## Pure pursuit (`run_pure_pursuit.py`)

A unicycle robot follows a path. On every control step it:

1. finds the **closest point** on the path (black dot). The search only looks
   ahead of the previous closest point, so the robot can't short-cut where a
   path crosses or doubles back on itself;
2. picks the **lookahead point** (red ring) a distance `lookahd` further along
   the path. Past the end, the last segment is extended in a straight line;
3. computes reference speeds `vRef`, `wRef` that steer toward it.

The robot only chooses the *reference* speeds. The actual `v` and `w` follow
them under the acceleration limits `acc_v` / `acc_w`. That's why a short
lookahead, a high gain or a slow control loop shows up as overshoot and
oscillation instead of perfect tracking.

On screen: the path (blue, goal marked with a square), the driven trail
(red), the lookahead circle, and the **arc the robot would drive if it kept
its current command**, `kappa = wRef / vRef` (green, dashed). The strip charts
bottom left show `v`, `w` and the signed cross-track error `e` (positive =
robot left of the path). They correspond to figure 3 of the MATLAB demo.

### Two control laws (`c` toggles)

**heading-P**, the MATLAB demo's controller (`calcCtrl` in `pure_pursuit.m`):

```
aErr = atan2(yT - y, xT - x) - a
wRef = kP * aErr
vRef = v_max * exp(-aErr^2 / (2 slowdn^2))
```

**pure pursuit**, the classic geometric law: drive the circular arc that is
tangent to the robot's heading and passes through the lookahead point,

```
kappa = 2 sin(alpha) / Ld          (alpha: angle to the target, Ld: distance to it)
vRef  = v_max * exp(-alpha^2 / (2 slowdn^2))
wRef  = vRef * kappa
```

There is no gain here: the lookahead distance *is* the gain. With pure
pursuit the green arc always passes exactly through the red ring. With
heading-P it generally doesn't. Pure pursuit wants a longer lookahead than
heading-P: try `lookahd` 0.3–0.5 m.

### Parameters

| row | meaning | default |
|---|---|---|
| `lookahd` | lookahead distance | 0.1 m |
| `v_max` | top speed | 1 m/s |
| `kP` | heading gain (heading-P only; shows `(unused)` otherwise) | 10 /s |
| `slowdn` | sigma of the slow-down with heading error; `off` = always `v_max` | 60° |
| `acc_v` | translational acceleration limit | 2 m/s² |
| `acc_w` | rotational acceleration limit | 3600 °/s² |
| `ctrl_dt` | control period (physics always runs at 1 ms) | 0.01 s |
| `speed` | simulation speed vs wall clock, for slow motion | 1x |

**Difference from the MATLAB demo:** `pure_pursuit.m` integrated the heading
with `a = a + w*dt` inside its physics sub-step loop, using the control
period `dt` instead of the sub-step `ddt`. The robot therefore turned 10x
faster than `w` said, which in effect gave it 10x the gain and 10x the
rotational acceleration. That is fixed here, and the defaults `kP = 10`,
`acc_w = 3600°/s²` reproduce what the MATLAB demo actually did. Its
`kP = 1`, `acc_w = 360°/s²` are one `<` press away.

Things to try:

* `acc_w` down to 360°/s² with `kP = 10`: the gain asks for more than the
  robot can deliver, and it falls into a limit cycle around the path.
* `ctrl_dt` up to 0.1–0.2 s: the same controller, sampled too slowly, goes
  unstable.
* `lookahd` up: smoother, but corners get cut. Down: tighter, until it
  oscillates.
* `slowdn` off: the robot takes corners at full speed.
* `d` a few times to see how each law recovers from being knocked off the path.

### Paths

`1`…`4` select the paths from the MATLAB demo (`paths/path1..4.csv`,
converted from `path.mat`, `path1.mat`, `path2.mat` and `path3.mat`).

**Drag with the mouse** in the plot to draw a new path, as `samplepath.m`
did. The robot restarts at its first point. `w` saves the current path to
`paths/drawn_<time>.csv`, and `--path FILE.csv` loads one.

### Keys

| | | | |
|---|---|---|---|
| `space` | pause / run | `tab` / `shift-tab` | select a parameter |
| `r` | reset (restart the path) | `>` / `<` | raise / lower it |
| `d` | disturb the robot | `g` | lookahead geometry on/off |
| `c` | control law: heading-P / pure pursuit | `t` | driven trail on/off |
| `1`…`4` | built-in path | `S` | screenshot (2 PNGs, in `snapshots/`) |
| `w` | save the path to `paths/` | `h` | key list |
| mouse drag | draw a new path | `q` | quit |

### Command line

```sh
.venv/bin/python run_pure_pursuit.py --path 3 --set lookahead=0.3
.venv/bin/python run_pure_pursuit.py --law pure-pursuit --set lookahead=0.5
.venv/bin/python run_pure_pursuit.py --set accw=360 --set kp=10     # limit cycle
.venv/bin/python run_pure_pursuit.py --headless --path 2            # run to the goal, print a summary
.venv/bin/python run_pure_pursuit.py --snapshot out.png --steps 150
```

`--set` takes any parameter name (`lookahead`, `vmax`, `kp`, `sigma`,
`accv`, `accw`, `ctrl_dt`, `time_scale`), with angles in degrees and `inf`
for unlimited/off. `--theta0` sets the starting heading in degrees. The robot
always starts at the path's first point.

## Layout

```
run_pure_pursuit.py   the demo program
navdemo/path.py       path geometry: closest point, lookahead point (find_closest_point.m, get_lookahead_point.m)
navdemo/sim.py        robot with acceleration limits, the two control laws, the follower loop
navdemo/params.py     parameter ladders and the demo state
navdemo/draw.py       robot, geometry overlay, strip charts, text panel
navdemo/keys.py       keyboard and mouse
navdemo/app.py        argument parsing, main loop, screenshots
paths/                path1..4.csv from the MATLAB demo, plus any you save
tests/test_navdemo.py checks, no framework needed
```
