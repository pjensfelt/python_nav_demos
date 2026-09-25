# Python navigation demos

Navigation demos used during lectures: following a path (pure pursuit, a
Python port of the MATLAB `matlab_pure_pursuit` demo), and planning a path
on a grid made from the real world, then driving it there (A*, RRT, RRT*,
with a known map or one built as the robot goes). They share their window
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
.venv/bin/python run_planning.py                # plan on a grid, drive in the real world
.venv/bin/python tests/test_navdemo.py          # checks (pure pursuit)
.venv/bin/python tests/test_planning.py         # checks (planning, ~30 s)
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

**Target behind the robot (`b`).** Pure pursuit's w is proportional to v.
With the target behind, the slow-down makes v almost zero, so w is almost
zero too, and the robot crawls along a huge arc instead of turning round.
Try `--law pure-pursuit --set lookahead=0.3 --theta0 180`: the robot starts
facing backwards and takes forever. `b` switches on the usual fix: if the
target is more than 90° off, stop and turn on the spot until it's back in
front. It's off by default here, so you see the problem first, and on by
default in the planning demo. Heading-P doesn't need it, because its
w = kP·aErr doesn't depend on v.

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

**Stopping at the goal** also differs from the MATLAB demo. There the robot
drove at full speed until the goal and only then asked for v = 0, so it
overshot by v²/(2·acc_v): 0.25 m at the defaults. Here the speed is capped by
a braking profile, v ≤ √(2·a·d) with d the distance left along the path and
a = acc_v/2. The robot decelerates uniformly over the last stretch and comes
to rest on the goal, within a couple of centimetres
(`Follower.brake_for_goal` in `navdemo/sim.py`, shared by both demos).

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
| `b` | pure pursuit: turn in place when the target is behind | | |
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
for unlimited/off. `--theta0` sets the starting heading in degrees, and `--turn-in-place` starts with `b` on. The robot
always starts at the path's first point.

## Planning (`run_planning.py`)

The point of this demo is what happens between the real world and the
planner, not the planners themselves:

**real geometry → grid (cell size, inflation) → planner → pure pursuit, driving in the real geometry**

The world is made of geometric primitives: circles, boxes, walls and
polygons, in `worlds/*.json`. The planners never see it. They see an
occupancy grid made from it, the way a real robot only ever has a map, not
the world. Both are drawn on top of each other: the real obstacles as black
outlines, the grid as shaded cells. The robot then drives the plan in the
*real* world, so anything the grid got wrong shows up as a collision.

### Two ways to get the map (`k` toggles)

**Known map.** The grid is made up front from the real geometry, with one test
per cell centre. You plan once and drive.

**Map as we go.** The robot starts with an empty map and a 360° lidar (rays
cast against the real geometry).
- **Each scan:** the cells a ray passes through become known free, and the
  cells around each hit become occupied (plus inflation).
- **Unknown cells:** shaded beige, and planned through as if free. This is
  the optimistic "free space assumption" that makes planning possible before
  you've seen everything, and that leads the robot the wrong way.
- **Replanning:** after every scan (10 Hz), the rest of the path is checked
  against the map. If something new blocks it, the robot replans from where
  it is.
- **Earlier plans:** they stay on screen, dashed, so every change of mind is
  visible.
- **Limitation:** occupied cells are never cleared again. A real mapper keeps
  a probability per cell (log-odds) so that noise and moving objects can be
  forgotten.

### The grid

| row | meaning | default |
|---|---|---|
| `cell` | cell size | 0.1 m |
| `inflate` | obstacles are grown by this before gridding, so the planner can treat the robot as a point | 0.3 m |

`m` switches how a cell is decided:
- **center:** a cell is occupied if its centre is within `inflate` of an
  obstacle. It's cheap, but anything thinner than a cell can fall between the
  centres and vanish.
- **conservative:** a cell is occupied if *any part* of it might be. The
  centre test gets half the cell diagonal added. Nothing is ever missed, but
  gaps close sooner.

The robot radius is 0.2 m. The default inflation of 0.3 m is the radius plus
a 0.1 m margin. With exactly 0.2 m there is no room left for tracking error
(pure pursuit cuts corners, and diagonal steps between free cells pass closer
than a cell centre), and the robot clips obstacles.

### Planners (`p` cycles)

Implementations in `navdemo/planners.py` are kept short to be readable.

- **A\*** on the grid, 8- or 4-connected (`n`). `h_weight` scales the
  heuristic: 0 is Dijkstra, 1 is A\*, >1 is weighted A\*. Expanded cells are
  shown coloured by expansion order.
- **RRT** in the continuous plane, collision checked on the grid. Its
  parameters are `iters`, `step` (max edge length) and `goal_bias`. `f`
  chooses between stopping at the first path and using all iterations.
- **RRT\*** adds the best-parent choice and rewiring within `radius`, and
  always uses all iterations.
- **`x`** makes RRT/RRT* check against the exact geometry instead of the grid.
  This skips discretization entirely. The obstacles grown by `inflate` (what
  the planner now avoids) are drawn dashed red, and the unused grid is faded.
  It's slower: about 2 s instead of 0.1 s for 2000 RRT* iterations. It does
  nothing with A\*, which needs cells to search, or while mapping as we go,
  where the grid is all the robot has. The panel's `checks:` line always says
  which check is in use, and pressing `x` prints why when it doesn't apply.
  Like every planner setting, it takes effect when you replan (`enter`).
- **`s`** shortcuts the path: it jumps to the furthest point still in line of
  sight (the WASP `optimize_path`). The raw path stays dotted underneath.

Structure taken from the WASP assignment 3 planners
(KTH-RPL/wasp_autonomous_systems, branch ht26, `src/wasp_as_ass_3`), with A*
completed. Their RRT* only rewired; here it also picks the best parent and
passes cost changes on to descendants.

### Execution

The plan is driven with the same pure pursuit code as `run_pure_pursuit.py`
(`navdemo/sim.py`, unchanged): `lookahd`, `v_max`, `kP` and `speed` are
rows here, and `c` switches control law. The robot is checked against the
real geometry every 20 ms, and stops, red, on contact.

### Things to try

* **World 3 (thin walls):** `cell` 1 m, or 0.5 m with `inflate` 0.2. The 5 cm
  walls vanish from the grid, A\* plans straight through, and the robot hits
  the wall. Press `m` for conservative cells and the walls are back. Or keep
  the coarse grid, switch to RRT\* (`p` twice) and press `x`, then `enter`:
  checked against the real geometry, it goes round the walls the grid lost.
* **World 2 (narrow passage):** switch to conservative cells, and the 0.8 m
  door closes. The robot takes the long way (18.5 m instead of 9.5 m). At
  0.5 m cells there is no path at all.
* **`inflate` down to 0.2, then 0:** the plan grazes obstacles. At 0 the
  robot collides in every world.
* **World 8 (dead end), `k` for mapping as we go:** the robot drives into the
  cul-de-sac, sees the back wall, replans and backs out. It drives about
  17 m where the known map needs 12 m.
  - Lower `lidar_rng` to 1.5 m and it goes much deeper in first, driving
    about 28 m.
* **`sig_rng` 0.1 m while mapping (world 8):** noisy hits land in free space.
  Since occupied cells are never cleared, the map fills up and the robot
  ends with no path at all. This is the argument for a probabilistic
  (log-odds) map.
* **RRT vs RRT\* while mapping:** every replan is a new random tree, so the
  robot's route changes a lot more than with A\*.
* **World 5 (bug trap):** compare how many cells Dijkstra (`h_weight` 0), A\*
  and weighted A\* expand.

### Keys

| | | | |
|---|---|---|---|
| `enter` | plan (from where the robot is) | `k` | known map / map as we go |
| `space` | drive / pause (plans first if needed) | `m` | cells: center / conservative |
| `r` | reset (in mapping mode, a fresh map) | `p` | planner: A* / RRT / RRT* |
| `1`…`8` | world | `n` | A*: 8 / 4 connectivity |
| left click | set the goal | `x` | RRT/RRT*: grid / exact geometry |
| right click | set the start | `f` | RRT: stop at first path / all iterations |
| `tab` / `shift-tab` | select a parameter | `s` | shortcut the path |
| `>` / `<` | raise / lower it | `c` / `b` | control law / turn in place (pure pursuit) |
| `g` / `o` / `e` | grid / real obstacles / search on/off | `l` / `t` | lookahead geometry / trail on/off |
| `S` | screenshot | `h` / `q` | key list / quit |

Rows that don't apply (for example RRT's `step` while A\* is selected, or the
lidar rows with a known map) are hidden, and `tab` skips them. Changing a grid
setting rebuilds the map and resets the robot. Changing a planner setting
marks the plan as stale until you press `enter`.

### Command line

```sh
.venv/bin/python run_planning.py --world 3 --set res=1.0              # thin walls vanish
.venv/bin/python run_planning.py --world 8 --mapped                   # into the dead end and out
.venv/bin/python run_planning.py --world 4 --planner "RRT*" --seed 2
.venv/bin/python run_planning.py --headless --world 8 --mapped        # plan, drive, print a summary
```

`--set` takes any row name (`res`, `inflate`, `sensor_range`, `rays`,
`noise`, `h_weight`, `iterations`, `step`, `goal_bias`, `radius`, `anim`,
`lookahead`, `vmax`, `kp`, `time_scale`). Other options are `--exact`, `--conservative`,
`--law pure-pursuit`, `--no-turn-in-place`, `--snapshot FILE.png` and `--steps N`.

### Worlds

`worlds/1_gap.json` … `8_deadend.json`: key `n` loads the `n`-th file in
name order. The format is described at the top of `navdemo/world.py`.
`--world FILE.json` loads your own.

## Layout

```
run_pure_pursuit.py    pure pursuit demo
run_planning.py        planning demo
navdemo/path.py        path geometry: closest point, lookahead point (find_closest_point.m, get_lookahead_point.m)
navdemo/sim.py         robot with acceleration limits, the two control laws, the follower loop
navdemo/params.py      pure pursuit parameter ladders and state
navdemo/draw.py        pure pursuit drawing: robot, geometry overlay, strip charts, panel
navdemo/keys.py        pure pursuit keyboard and mouse
navdemo/app.py         argument parsing, main loop, screenshots
navdemo/world.py       the real world: circles and polygons, distances, world files
navdemo/grid.py        occupancy grid from the world, line traversal, collision checkers
navdemo/mapping.py     lidar and the grid that is built as the robot goes
navdemo/planners.py    A*, RRT, RRT*, shortcutting
navdemo/mission.py     plan -> drive -> sense -> replan, collisions with the real world
navdemo/planstate.py   planning parameter ladders and state
navdemo/plandraw.py    planning drawing: obstacles, grid, search, scan, panel
navdemo/plankeys.py    planning keyboard and mouse
paths/                 path1..4.csv from the MATLAB demo, plus any you save
worlds/                world files for run_planning.py
tests/                 checks, no framework needed
```
