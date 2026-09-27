# Status of the navigation demos

Written 2026-09-26, at the end of a long working session, so that nothing
important is lost. **README.md** is the full reference for what each demo
does (keys, parameters, numbers, things to try); **IDEAS.md** holds the ideas
and design discussions not built yet. This file says where things stand,
what was decided and why, and what comes next.

## The setup

- `python_nav_demos/` is its own git repo, `git@github.com:pjensfelt/python_nav_demos.git`
  (branch `main`). It sits next to `python_loc_demos/` (repo
  `pjensfelt/python_loc_demo`, the localization/SLAM demos) in the same
  parent folder, which is not a repo itself. The two repos share nothing:
  `navdemo/` does not import `locdemo/`; the few helpers both need are
  copied.
- Each repo has its own `.venv`. Always run nav demos from inside
  `python_nav_demos/` with `.venv/bin/python ...` (running from the parent
  folder fails: there is no `.venv` there).
- Tests: `tests/test_grid.py`, `tests/test_navdemo.py` (pure pursuit),
  `tests/test_planning.py`. No framework, just `python tests/<file>.py`. The
  planning suite is slow (a few minutes: it drives every world several ways).
- The course is DD2410 at KTH; the lecture is `Lecture12_dd2410_Navigation.pdf`
  (66 slides). Patric's part is about the **real-world aspects** of
  navigation -- the students have already seen A*/RRT in known worlds.

## The three demos (see README.md for details)

1. **`run_grid.py` -- the world and its grid.** Only the real world and its
   occupancy grid. Cell size, inflation (grid first by default, world first
   with `i`), two cell rules (`m`: exact "any overlap" from the model, or
   "samples" along the outlines with spacing/noise/min_hits, sensor-like),
   the imperfect building (`imperfect`, `v`), moving and rotating the grid
   over the world (arrows, mouse drag, `,` `.` `k` `l`, sweeps `x` `y` `r`,
   `0` back), cell count / memory / time, and optional planning between
   designed probe points (`p`, with click to set start/goal). Four worlds of
   its own: door, pillars, diagonal corridor between two rooms, thin
   partition with a doorway. Finished for now.
2. **`run_pure_pursuit.py` -- following a path.** Port of the MATLAB
   `matlab_pure_pursuit` demo: three control laws (`c`: heading-P as in the
   MATLAB demo, classic pure pursuit, and stop and turn), lookahead and the
   other controller settings, the four MATLAB paths, drawing your own path
   with the mouse. Braking into the goal, turn-in-place (`b`). The MATLAB
   heading-integration bug is fixed (defaults kP 10, acc_w 3600°/s²
   reproduce what the MATLAB demo actually did). Finished, apart from the
   idea of adding noise (see IDEAS.md).
3. **`run_planning.py` -- planning on a grid, driving in the real world.**
   Known map (made from samples, as the grid demo's samples rule) or map as
   you go (`m`: lidar, two layers: the map proper and a planning map
   regenerated from it by inflation, replanning). A*, RRT, RRT* (`p`),
   exact-geometry checks for RRT (`x`), shortcutting (`s`), the imperfect
   building, rotating the world under the grid (`,` `.` `k` `l`, `0`).
   **Execution is off at the start** (`e`): first the planner, then how to
   follow its path, with the three control laws (stop and turn shows why
   pure pursuit or smoothing is needed). **Being worked on now.**

## Latest changes

Everything is committed and pushed (tested, all tests pass). Since the
first version of this file (2026-09-27):
- **`s` shortcuts the existing plan** in the planning demo (no new search;
  `s` again restores it; only before driving), and a red **COLLISION**
  label above the robot;
- **localization jitter** (`loc_xy`, `loc_th`) in both demos: fresh
  Gaussian noise on the pose the controller uses at every control step, no
  drift (Patric: drift would be too hard to handle). The estimate is drawn
  as a dashed purple robot. Stop and turn now counts a corner as reached
  when it has got that far along the leg, so it copes with jitter up to
  about 5 cm;
- **lookaheads down to 1 cm** (1, 3, 5, 7 cm) in both demos, to show
  absurdly small values;
- the pure pursuit demo **hides the rows the control law doesn't use**
  (blank gaps keep the layout; tab skips them), and takes
  `--law stop-and-turn`; the planning demo's `--set` takes angles in degrees.
Check `git -C python_nav_demos status` when resuming anyway.

## Decisions and preferences to keep (why the demos look the way they do)

- **Start simple, let features be turned on.** Planning is off at the start
  in the grid demo (`p`), execution is off at the start in the planning demo
  (`e`).
- **Cut what isn't about the demo's topic.** Removed along the way: the
  center-sampling cell rule, the changed-cells-vs-reference overlay (there
  is no privileged reference -- every pose is just another realisation), the
  dashed outline of the grown obstacles in the grid demo, the sweep strip
  charts, the 4 / 8 / corner-cutting moves in the grid demo, the grid
  demo's panel lines for occupied counts.
- **Each world demos one clear thing**, and its start/goal (probes) are
  designed for that; `0` restores them.
- **No artificial alignment.** Worlds are perturbed "as built" (2 cm default),
  rooms aren't square, walls that end at the room's boundary reach 0.2 m past
  it (so the perturbation can't open a crack). The cell lattice is anchored
  to the room as drawn, so moving the world really moves it across the cells.
- **A map from sensor data has no model:** inflate in the grid (grid first),
  keep the map proper and regenerate the inflated planning map from it
  (never update an inflated map in place). The known map in the planning
  demo is made from samples for that reason.
- **Single, unshifted keys that work on a Swedish Mac keyboard** (`,` `.` `k`
  `l` for rotation; brackets need Option there). The same key means the same
  thing in both demos where possible.
- **Discuss before coding** when Patric asks "thoughts?" or "what do you
  mean"; he often refines the design in a few steps.
- **Every number quoted in the README is measured** (usually headless, with
  `--set imperfect=0` for the worlds as drawn); re-measure after changes
  instead of trusting old numbers.
- Planning worlds' comments and README claims were rechecked after the
  switch to sample-based known maps; they hold.

## Next steps

### Planning demo (where we are)

From the list agreed when moving to the planning demo:
1. **Start simple** -- partly done (execution toggle). Still open: a legend
   that only shows what is on screen (it lists the RRT tree, driven trail,
   earlier plans ... even when they aren't there).
2. **Trim the worlds to one lesson each** -- not done. Proposal: drop "thin
   walls" (its lesson, walls vanishing between cell centres, is gone; the
   grid demo covers thin walls), maybe fold "gap" into "rooms"; keep narrow
   passage (grid closes the door; exact-geometry RRT gets through; rotation
   closes it), bug trap (A* vs Dijkstra expansions), dead end (map as you
   go), maze (long path; its narrow opening closes when rotated 15°), rooms,
   clutter (RRT randomness).
3. **Execution settings shown only when driving** -- done with `e`.
4. **Corner cutting** as a third choice next to `n`'s 4/8 connectivity -- not
   done; measured example in IDEAS.md (door world, 0.5 m cells, rotated
   0–45°: closed at every angle without it, open at 68 % with it).

### The bigger ideas (details in IDEAS.md)

In the order suggested, all waiting for a go-ahead and a design round:
- **Local maps that forget + dynamic obstacles** (slides 35–38): map update
  modes (last scan only / accumulate / clear along rays / decay), a person in
  a doorway who leaves, a chair seen and then out of view. The two-layer map
  already makes clearing possible (it only touches the map proper).
- **Reactive local planners** (slides 26–33): potential field, VFH, DWA as
  executors next to pure pursuit. The bug trap shows potential-field local
  minima; DWA fits the simulator's acceleration limits.
- **Cost map / Gaussian smoothing for clearance** (slides 13–15).
- **Pure pursuit with noise** (slide 4 idea): noise on the pose the
  controller uses and/or on actuation; lookahead vs jitter.
- **Emergency stop and what comes after it** (slides 39–40): protective vs
  latched stop, speed-dependent safety field (v²/2a, the same as the braking
  profile), the recovery ladder, operator interventions as the metric, a
  "lecture mode" that freezes and asks the class what to do. Three open
  questions are listed in IDEAS.md.
- **Path smoothing by optimization** (slides 16–22) and **robot shape**
  (slide 10), lower priority.

## Practical notes

- matplotlib: the demos clear matplotlib's default keymap; keys only work
  when the window has focus.
- numpy 2 on macOS (Accelerate) emits spurious divide-by-zero warnings from
  `@` (matmul) on some inputs; the geometry code writes such products out
  instead. If the warnings appear again, look for a new `@`.
- `S` saves screenshots to `snapshots/` (git-ignored). Never delete that
  folder wholesale: it can hold Patric's own screenshots.
- The world files keep round, "as drawn" numbers; the demos perturb them.
  `worlds/*.json` are the planning worlds (keys 1–8), `worlds/grid/*.json`
  the grid demo's own (keys 1–4, then the first planning worlds).
