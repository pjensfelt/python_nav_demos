# Python navigation demos

Navigation demos used during lectures: how the real world turns into a grid
(and how much that grid depends on where the cell borders fall), following a
path (pure pursuit, a Python port of the MATLAB `matlab_pure_pursuit` demo),
and planning a path on a grid made from the real world, then driving it
there (A*, RRT, RRT*, with a known map or one built as the robot goes). They share their window
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

.venv/bin/python run_grid.py                    # the real world and its grid
.venv/bin/python run_pure_pursuit.py            # pure pursuit path following
.venv/bin/python run_planning.py                # plan on a grid, drive in the real world
.venv/bin/python tests/test_grid.py             # checks (grid)
.venv/bin/python tests/test_navdemo.py          # checks (pure pursuit)
.venv/bin/python tests/test_planning.py         # checks (planning, ~30 s)
```

The figure window has to have keyboard focus for the keys to work. Press `h`
for the key list at any time.

## The world and its grid (`run_grid.py`)

A stripped-down demo that shows only the real world, its occupancy grid, and
the choices that decide one from the other. It's the place to start the
discussion of grid maps (lecture slides 9–12) before anything is planned on
them. Planning starts off; `p` turns it on, as a test of whether the grid
still lets you through.

The real obstacles are drawn as outlines, and the world's boundary is a wall
too (the world is a room). Over them is the grid: dark cells are obstacle
cells, light cells are there only because of the inflation (`inflate`, 0 to
0.5 m in 2 cm steps; hold `>` to run through them). The grid always has one
ring of cells round the room, so the room's walls show up in it too. The
demo starts at 0.1 m cells, where the grid still looks like the world;
coarsen it with `>` to watch it fall apart.

### The world as built, not as drawn (`imperfect`, `v`)

The world files use round numbers (0.2 m walls, square rooms), and round
numbers line up with the cell lattice in ways no real building does. A wall
whose faces lie exactly on cell borders, or a door exactly 1.5 cells wide,
produces effects of the world file rather than of discretization. So both
demos show the building *as built*: every corner of every wall, box and of
the room itself is moved by 2D Gaussian noise of std `imperfect` (2 cm by
default), and circles move and change size a little. Walls come out slightly
tapered and rooms not quite square. Start, goal and probe points stay where
they were designed. `v` builds another variant; variant 0 is the same every
time, and `imperfect` 0 gives the world exactly as drawn.

Because of this, the numbers quoted below are **for the worlds as drawn**
(`--set imperfect=0`). With the default imperfection they change a little
from variant to variant; the door world, for example, is open at 39–51 % of
the offsets, depending on the building. That spread is itself the point:
what matters is how often a coarse grid closes a door, not what one lucky
alignment does.

### Two ways to make cells (`m`)

- **any overlap** (default, uses the geometric model): a cell is occupied if
  anything of the obstacle is inside it, tested exactly (the cell square
  against the obstacle; a cell that only touches an obstacle along an edge
  doesn't count). The natural rule for a binary map, and the safe one: it
  never loses an obstacle, but it eats free space.
- **samples** (sensor-like): points every `spacing` metres along every
  outline, including the room's walls, scattered by 2D Gaussian noise of std
  `noise`. A cell is occupied if at least `min_hits` points land in it. This
  is how a map from a range sensor comes about: only surfaces are seen, so
  obstacles come out hollow. Too sparse samples leave holes, noise thickens
  walls and adds stray points. The samples stay fixed to the world while
  you move the grid; `u` draws a fresh set, `d` shows or hides the points.

Any overlap never loses an obstacle, so what it loses is free space. With
cell size c, a wall of width w covers ⌈w/c⌉ or ⌈w/c⌉ + 1 cells depending
on where the cell borders fall, and a gap of width g keeps ⌊g/c⌋ or
⌊g/c⌋ − 1 free cells. Samples can fail the other way too: with too few
points, or noisy points spread over cells smaller than the noise, walls get
holes.

### Inflation: grid first or world first (`i`)

- **grid first** (default): make cells of the obstacles, then grow the
  *occupied cells* by `inflate`, in the grid (a costmap's inflation layer).
  This is what a robot building its map from sensor data has to do. The map
  only knows that a cell is occupied, not where in it the obstacle is, so
  it must assume the obstacle could be right at the cell's edge: any
  inflation above 0 adds at least one ring of cells round every occupied
  cell. The discretization error comes first and the inflation is added on
  top of it, rounded to whole cells again. The errors compound, so gaps
  close sooner.
- **world first:** grow the real obstacles by `inflate`, then make cells of
  the grown shapes. Only possible if you have a geometric model of the
  world, and more precise: every free cell is still at least `inflate` from
  the obstacles, but no more than it has to be.

With world first and an inflation smaller than a cell, you will see dark
(obstacle) cells right next to white (free) ones, with no inflation cell in
between. That isn't a bug. It happens where the obstacle only clips the far
edge of the dark cell: the dark cell then already reaches more than
`inflate` beyond the obstacle, so the margin lies inside it, and the next
cell is free. (World 1 rotated 14°, 0.2 m cells, 0.14 m inflation: 44 dark
cells border free ones, yet no free cell is closer than 0.141 m to an
obstacle. Grid first gives none, at the price of free space starting only
0.245 m from the obstacles.)

The samples rule is always grid first, since it has no model.

### Moving the grid over the world

Only the pose of one relative to the other decides the cells, but moving the
*grid* over the fixed world is the more honest picture: the building is what
it is, the map frame is whatever you happened to choose. `w` switches to
moving the world instead.
- Arrow keys shift by a tenth of a cell (with shift, a fiftieth), or drag
  with the mouse.
- `,` / `.` rotate by one degree about the world's centre, and `k` / `l`
  (right above them) by five. Hold a key to keep turning. `[` `]` and
  `{` `}` do the same, where the keyboard layout makes those easy.
  Rotation is where coarse cells hurt most: a wall that isn't aligned with
  the grid becomes a staircase.
- `x` / `y` sweep one cell along the grid's own axes and back; `r` sweeps
  the rotation from 0 to 45° and back. Run headless (`--headless --sweep`),
  a sweep prints the occupied % and, with `--plan`, open or not for each
  probe, as a table against the offset or angle.
- `0` puts everything back.

The grid always covers the whole world: rotated, it grows by the cells
needed, and cells entirely outside the world are drawn faint grey and left
out of the statistics.

### Planning through the grid (`p`, off at the start)

Each world has one or more pairs of probe points, e.g. either side of a
door, placed to demo something specific. With planning on, a left click
moves the goal and a right click the start (a left *drag* still moves the
grid); `0` puts the designed ones back. The demo plans between each pair
with the same A\* as the planning demo (8-connected, never squeezing
diagonally between two occupied cells that touch at a corner). It draws the
path, gives its length against the straight line, or says the pair is
blocked (and why, if a probe point lands in an occupied cell). `f` tints the
cells the planner expanded for the first probe. A 45° staircase of free
cells that touch only at their corners counts as closed, though the eye
sees a way through it: a robot of any real size wouldn't fit.

### Cost and the other numbers

The panel gives the number of cells and their memory (1 byte each). It also
times how long it took to **make the grid** and, with planning on, to **plan**
on it. Halve
the cell size and there are four times as many cells, and roughly four
times the time. For the 10 × 10 m clutter world (world 8):

| cell | cells | make the grid | planning |
|---|---|---|---|
| 0.1 m | 10,000 | ~15 ms | ~20 ms |
| 0.05 m | 40,000 | ~40 ms | ~80 ms |
| 0.02 m | 250,000 | ~0.2 s | ~0.5 s |
| 0.01 m | 1,000,000 | ~0.8 s | ~2 s |

At 1 cm the window visibly lags on every key press, which is the point.

### Worlds (keys `1`…`9`)

Four made for this demo, in `worlds/grid/`, then the first five planning
worlds. The panel shows the number and name (and the variant). Numbers
below are for the worlds as drawn (`imperfect` 0), 0.5 m cells and any
overlap unless stated, with planning on (`p`) to see open or blocked.

| key | world | what it shows |
|---|---|---|
| `1` | door | a 0.75 m door, 1.5 cells: **closed at the start pose**, open at 51 % of the offsets along y (`y`) |
| `2` | pillars | 10 cm pillars 0.5 m apart, gaps a robot fits through: open at only 12 % of the offsets along y |
| `3` | diagonal corridor | two triangular rooms joined only by a 0.6 m corridor (the robot is 0.4 m) along the room's diagonal, with a 1 m door into it from each room at opposite ends; the probe points are in the two rooms. The corridor's walls become staircases of cells: open at up to 0.2 m cells, at 0.25 m open at only 37 % of the offsets along y, from 0.3 m always closed. Rotate the grid 45° (`r`) and it lines up with the cells |
| `4` | thin partition | a 3 cm partition with a 0.7 m doorway near the top, the probe points either side of it, far from the doorway. The right path goes round by the doorway (9.4 m). Coarse cells close the doorway (open at 44 % of the offsets along y); noisy or sparse samples put holes in the partition, and the path goes straight through the wall (see below) |

### Things to try

* **World 1, `>` to 0.5 m, `y`:** the door opens and closes with a few
  centimetres of shift. At 0.25 m it stays open.
* **World 1 at 0.25 m, `inflate` 0.1:** with grid-first inflation (the
  default) the door is open at only 2 % of the offsets along y; press `i`
  for world first and it stays open at every one.
* **World 1 at 0.5 m, `r`, or hold `.`:** rotated, the door is closed at
  every angle from 0 to 45°. At 0.25 m it stays open at every angle.
* **World 3, `>` from 0.1 m to 0.3 m:** the diagonal corridor's walls turn
  into staircases that eat into it, until it closes. At 0.25 m, `y` shows it
  opening and closing with the grid's position; `r` shows it opening when
  the grid lines up with it at 45°.
* **World 4, `m` (samples), `noise` 0.05 m, `spacing` 0.1 m:** at 0.1 m cells
  the noise spreads the points over several small cells, some rows of the
  partition get no hit, and the planned path goes straight through the wall
  (about 3.3 m instead of 9.4 m round by the doorway; 5 of 6 draws, `u` for
  another). The same samples at 0.25 m or 0.5 m cells never leak. A finer
  grid is not always better: the resolution has to suit the data. With
  noiseless points, `spacing` 0.3 m leaks too (sparse samples).
* **Any world at 0.02 m or 0.01 m:** look at the time line.

### Keys

| | | | |
|---|---|---|---|
| arrows / mouse drag | shift the grid (or world) | `m` | cells: any overlap / samples |
| `shift`+arrows | shift 1/50 cell | `i` | inflate: grid first / world first |
| `,` / `.`, `k` / `l` | rotate ∓1°, ∓5° | | |
| `w` | move the grid / the world | `tab` / `shift-tab` | select a parameter |
| `x` / `y` / `r` | sweep one cell / the rotation | `>` / `<` | raise / lower it |
| `0` | back to the start: poses, and the designed start and goal | `u` / `d` | samples: fresh set / show points |
| left click / right click | set the goal / the start (planning on) | | |
| `g` / `o` / `f` | grid / obstacles / planner's cells | | |
| `1`…`9` | world | `p` | planning between the probe points on / off |
| `v` | another variant of the building | `S` / `h` / `q` | screenshot / key list / quit |

### Command line

```sh
.venv/bin/python run_grid.py --world 1 --set res=0.5 --sweep y --plan
.venv/bin/python run_grid.py --world 3 --set res=0.25 --sweep y --plan
.venv/bin/python run_grid.py --world 4 --rule samples --set sigma=0.05 --set spacing=0.1 --plan
.venv/bin/python run_grid.py --headless --world 1 --set res=0.5 --sweep y --plan   # the sweep as a text table
```

`--set` takes `res`, `inflate`, `imperfect`, `spacing`, `sigma` and
`min_hits`. `--seed N` picks the buildings (and the samples). Other
options: `--plan` (start with planning on), `--rule any-overlap|samples`,
`--inflate grid|world` (the inflation order; the amount is `--set inflate=…`), `--seed N`
(samples), `--grid-offset DX DY`, `--grid-rotate DEG`, `--world-offset`,
`--world-rotate`, `--move grid|world`, `--snapshot FILE.png`. A world file
can list its own probes (see the top of `navdemo/world.py`); without them,
the probe is start → goal.

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

### Two ways to get the map (`m` toggles)

**Known map.** The grid is made up front, as a robot would have it from an
earlier survey with its sensor: from points sampled along the obstacles'
outlines (orange; `d` hides them), exactly as the grid demo's "samples"
rule. A cell is occupied if at least `min_hits` points land in it, and the
occupied cells are then inflated in the grid (there is no geometric model
to inflate first). `spacing` and `smp_noise` set how dense and how noisy
the survey was, and `u` draws a fresh one. You plan once and drive.

**Map as we go.** The robot starts with an empty map and a 360° lidar (rays
cast against the real geometry). As in a real mapping system there are two
layers: the **map**, which holds only what the sensor said about each cell
(dark cells), and the **planning map**, made from it by growing the occupied
cells by `inflate` (light cells) and made again whenever the map changes.
An inflated map can't be updated in place: once cells are merged, you can't
tell which obstacle made which one occupied, so nothing could be removed.
- **Each scan:** the cells a ray passes through become known free, and the
  cell each hit lands in becomes occupied in the map.
- **Unknown cells:** shaded beige, and planned through as if free. This is
  the optimistic "free space assumption" that makes planning possible before
  you've seen everything, and that leads the robot the wrong way.
- **Replanning:** after every scan (10 Hz), the rest of the path is checked
  against the map. If something new blocks it, the robot replans from where
  it is.
- **Earlier plans:** they stay on screen, dashed, so every change of mind is
  visible.
- **Limitation:** occupied cells are never cleared from the map yet. A real
  mapper lets rays clear cells, or keeps a probability per cell (log-odds),
  so that noise and moving objects can be forgotten. That only ever touches
  the map; the planning map is simply made again.

### The grid

| row | meaning | default |
|---|---|---|
| `cell` | cell size | 0.1 m |
| `inflate` | obstacles are grown by this before gridding, so the planner can treat the robot as a point | 0.3 m |
| `imperfect` | the building as built: corners moved by this much (see the grid demo); `v` for another variant | 0.02 m |
| `spacing` | known map: distance between survey points along the outlines | 0.05 m |
| `smp_noise` | known map: noise (std) on the survey points | 0 |
| `min_hits` | known map: points a cell needs to count as occupied | 1 |

The grid stays axis-aligned; the **world can be rotated** under it (`,` /
`.` by 1°, `k` / `l` by 5°; `0` turns it back and restores the designed start
and goal). The whole world turns: obstacles, room, start and goal, and the
robot drives in the rotated world. The survey points turn with it, so
rotating shows discretization, not new noise. A world that isn't aligned
with the grid is the normal case, and it is where the grid hurts most.

How cells are made, and what else can go wrong there, is the subject of
`run_grid.py`.

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

The numbers are for the worlds as drawn (`--set imperfect=0`); with the
default imperfection they vary from variant to variant (`v`). In world 2,
for example, variant 1's door is just narrow enough for the default grid to
close it, and the robot takes the long way (18 m instead of 9 m).

* **World 2 (narrow passage), `cell` up:** at 0.1 m the robot goes through
  the 0.8 m door (9.4 m). At 0.2 m the grid has closed it and the robot takes
  the long way (18.7 m); from 0.25 m there is no path at all.
* **The same at 0.5 m, then RRT\* (`p` twice), `x`, `enter`:** checked against
  the real geometry instead of the grid, RRT\* goes straight through the door
  the grid closed (9 m). World 6 (rooms) is the same.
* **World 3 (thin walls):** the 5 cm walls are never lost, but they grow as
  thick as the cells. At 0.5 m the way round is 29 m; at 1 m even the goal
  is in an occupied cell.
* **World 2, rotate the world (`.` held, or `l`):** at the default 0.1 m
  cells the 0.8 m door is open when the world is aligned with the grid, and
  closed at 30° and 45°. The maze's one narrow opening closes already at 15°.
* **`inflate` down to 0.2, then 0:** the plan grazes obstacles. At 0 the
  robot collides in every world.
* **World 8 (dead end), `m` for mapping as we go:** the robot drives into the
  cul-de-sac, sees the back wall, replans and backs out. It drives about
  17 m where the known map needs 12 m.
  - Lower `lidar_rng` to 1.5 m and it goes much deeper in first, driving
    about 28 m.
* **`sig_rng` 0.1 m while mapping (world 2 or 6):** noisy hits land in free
  space. Since occupied cells are never cleared from the map, it fills up
  and the robot ends with no path at all (every seed tried; in world 8, most
  of them). This is the argument for a map that can forget, which the
  two-layer structure below makes possible.
* **RRT vs RRT\* while mapping:** every replan is a new random tree, so the
  robot's route changes a lot more than with A\*.
* **World 5 (bug trap):** compare how many cells Dijkstra (`h_weight` 0), A\*
  and weighted A\* expand.

### Keys

| | | | |
|---|---|---|---|
| `enter` | plan (from where the robot is) | `m` | known map / map as we go |
| `,` / `.`, `k` / `l` | rotate the world ∓1°, ∓5° | `0` | unrotated, designed start and goal |
| `u` / `d` | known map: fresh survey / show its points | | |
| `space` | drive / pause (plans first if needed) | `p` | planner: A* / RRT / RRT* |
| `r` | reset (in mapping mode, a fresh map) | | |
| `1`…`8` / `v` | world / another variant of the building | `n` | A*: 8 / 4 connectivity |
| left click | set the goal | `x` | RRT/RRT*: grid / exact geometry |
| right click | set the start | `f` | RRT: stop at first path / all iterations |
| `tab` / `shift-tab` | select a parameter | `s` | shortcut the path |
| `>` / `<` | raise / lower it | `c` / `b` | control law / turn in place (pure pursuit) |
| `g` / `o` / `e` | grid / real obstacles / search on/off | `a` / `t` | lookahead geometry / trail on/off |
| `S` | screenshot | `h` / `q` | key list / quit |

Rows that don't apply (for example RRT's `step` while A\* is selected, or the
lidar rows with a known map) are hidden, and `tab` skips them. Changing a grid
setting rebuilds the map and resets the robot. Changing a planner setting
marks the plan as stale until you press `enter`.

### Command line

```sh
.venv/bin/python run_planning.py --world 2 --set res=0.5 --planner "RRT*" --exact --seed 1   # through the door the grid closed
.venv/bin/python run_planning.py --world 8 --mapped                   # into the dead end and out
.venv/bin/python run_planning.py --world 4 --planner "RRT*" --seed 2
.venv/bin/python run_planning.py --headless --world 8 --mapped        # plan, drive, print a summary
```

`--set` takes any row name (`res`, `inflate`, `imperfect`, `spacing`,
`sample_sigma`, `min_hits`, `sensor_range`, `rays`,
`noise`, `h_weight`, `iterations`, `step`, `goal_bias`, `radius`, `anim`,
`lookahead`, `vmax`, `kp`, `time_scale`). Other options are `--rotate DEG`, `--exact`,
`--law pure-pursuit`, `--no-turn-in-place`, `--snapshot FILE.png` and `--steps N`.

### Worlds

`worlds/1_gap.json` … `8_deadend.json`: key `n` loads the `n`-th file in
name order. The format is described at the top of `navdemo/world.py`.
`--world FILE.json` loads your own.

## Layout

```
run_grid.py            grid demo
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
navdemo/rasterize.py   grid demo cells: exact any overlap, samples, center; inflation; moving the world
navdemo/gridstate.py   grid demo parameters, state, keys and mouse
navdemo/griddraw.py    grid demo drawing: cells, obstacles, probes, panel
navdemo/mapping.py     lidar and the grid that is built as the robot goes
navdemo/planners.py    A*, RRT, RRT*, shortcutting
navdemo/mission.py     plan -> drive -> sense -> replan, collisions with the real world
navdemo/planstate.py   planning parameter ladders and state
navdemo/plandraw.py    planning drawing: obstacles, grid, search, scan, panel
navdemo/plankeys.py    planning keyboard and mouse
paths/                 path1..4.csv from the MATLAB demo, plus any you save
worlds/                world files for run_planning.py
worlds/grid/           world files made for run_grid.py
tests/                 checks, no framework needed
```
