# Status of the navigation demos

Last updated 2026-09-27. **README.md** is the full reference for what each
demo does (keys, parameters, measured numbers, things to try); **IDEAS.md**
holds the ideas and design discussions. This file says where things stand,
what was decided and why, and what comes next.

## The setup

- `python_nav_demos/` is its own git repo, `git@github.com:pjensfelt/python_nav_demos.git`
  (branch `main`). It sits next to `python_loc_demos/` (repo
  `pjensfelt/python_loc_demo`, the localization/SLAM demos) in the same
  parent folder, which is not a repo itself. The two repos share nothing:
  `navdemo/` does not import `locdemo/`; the few helpers both need are
  copied.
- Each repo has its own `.venv`; use `python_nav_demos/.venv/bin/python`,
  from inside `python_nav_demos/`. Patric himself runs the demos with the
  system `python3` (3.9.6, numpy 2.0.2, matplotlib 3.9.4 -- the same versions
  as the venv), so the code must work there too.
- Tests, no framework, `python tests/<file>.py`: `test_grid.py` (22),
  `test_navdemo.py` (pure pursuit, 14), `test_planning.py` (30, ~2 min),
  `test_avoid.py` (14, ~2 min). All pass.
- The course is DD2410 at KTH; the lecture is `Lecture12_dd2410_Navigation.pdf`
  (66 slides). Patric's part is about the **real-world aspects** of
  navigation -- the students have already seen A*/RRT in known worlds.

## The four demos (see README.md for details)

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
   MATLAB demo, classic pure pursuit, and stop and turn), lookahead down to
   1 cm, the other controller settings, the four MATLAB paths, drawing your
   own path. Braking into the goal, turn-in-place (`b`), **localization
   jitter** (`loc_xy`, `loc_th`; the believed pose drawn dashed purple). Rows
   the control law doesn't use are left blank and skipped by tab. The MATLAB
   heading-integration bug is fixed (defaults kP 10, acc_w 3600°/s²
   reproduce what the MATLAB demo actually did). Finished.
3. **`run_planning.py` -- planning on a grid, driving in the real world.**
   Known map (made from samples, as the grid demo's samples rule) or map as
   you go (`m`: lidar, two layers: the map proper and a planning map
   regenerated from it by inflation, replanning). Planners (`p`): A* (with
   an optional **cost near obstacles**, `cost_w` / `cost_sig`, a Gaussian blur
   of the planning map shown as a red tint), RRT, RRT* (exact-geometry
   checks with `x`), and a **potential field** on the known map's sample
   points (`k_rep`, `d0`; the potential shaded with `y`). **Smoothing** (`s`,
   a setting applied to every plan and replan): as planned / B-spline /
   shortcut / shortcut + spline, with max curvature on the panel. The
   imperfect building, rotating the world under the grid (`,` `.` `k` `l`,
   `0`). **Execution is off at the start** (`e`): first the planner, then how
   to follow its path, with the three control laws and localization jitter.
   The legend lists only what is on screen.
4. **`run_avoid.py` -- obstacle avoidance with only the lidar.** Kept
   separate from the planning demo, which is full enough: this one is local
   and reactive, with things the map doesn't have. Worlds in `worlds/avoid/`
   (corridor, trap, office, hall) with obstacles in the map, obstacles NOT in
   the map, and people walking back and forth (by default they wait for the
   robot and give way after 2 s; `w` makes them walk blindly, `p` takes them
   out). Methods (`c`): VFH (the default), potential field, DWA, each with its
   own picture (`a`). Goal (`m`): clicked (you are the global planner) or a
   carrot along an A* path on the map. A sensor **field of view** (`fov`)
   and a **local map** (6 × 6 m, 0.1 m cells, moving with the robot), with
   three keys as Patric asked: `l` on/off, `u` add hits only / also clear
   along the rays, `f` forget (fade with half-life `forget`) on/off. Off,
   the methods use only the latest scan.

## Decisions and preferences to keep (why the demos look the way they do)

- **Start simple, let features be turned on.** Planning is off at the start
  in the grid demo (`p`), execution is off at the start in the planning demo
  (`e`); new options are toggles or settings that default to off.
- **One demo, one topic.** Obstacle avoidance got its own demo rather than
  more rows in the planning demo.
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
  keep the map proper and regenerate the inflated planning map (and the cost
  map) from it -- never update an inflated map in place. The known map in the
  planning demo is made from samples for that reason.
- **Noise as jitter, not drift** (Patric: drift would be too hard to handle).
- **Smoothing is a setting, not a one-off action:** it has to apply to every
  replan while driving, and can change mid-drive (it then smooths the rest of
  the plan from the robot).
- **People behave like people:** they wait for the robot and give way, or a
  robot and a person in a doorway deadlock. Walking blindly is an option,
  to show that reactive methods assume the world stands still.
- **Single, unshifted keys that work on a Swedish Mac keyboard** (`,` `.` `k`
  `l` for rotation; brackets need Option there). The same key means the same
  thing across demos where possible.
- **Discuss before coding** when Patric asks "thoughts?", "what do you
  mean" or "don't code yet"; he often refines the design in a few steps.
- **Every number quoted in the README is measured** (usually headless, with
  `--set imperfect=0` for the worlds as drawn); re-measure after changes
  instead of trusting old numbers.

## Next steps

### Obstacle avoidance

- The local map is built (2026-09-27); a new world for the out-of-view case
  only if the existing ones don't show it (Patric: "only if needed"). The
  A* carrot keeps planning on the global map only (Patric: leave the local
  map out of it).
- The emergency stop (below) probably belongs in this demo.

### Planning demo

- **Trim the worlds to one lesson each** -- not done. Proposal: drop "thin
  walls" (its lesson, walls vanishing between cell centres, is gone; the
  grid demo covers thin walls), maybe fold "gap" into "rooms"; keep narrow
  passage (grid closes the door; exact-geometry RRT gets through; rotation
  closes it), bug trap (A* vs Dijkstra expansions), dead end (map as you
  go), maze (long path; its narrow opening closes when rotated 15°), rooms,
  clutter (RRT randomness, the one world the potential field gets through).
- **Corner cutting** as a third choice next to `n`'s 4/8 connectivity -- not
  done; measured example in IDEAS.md (door world, 0.5 m cells, rotated
  0–45°: closed at every angle without it, open at 68 % with it).
- Proposed, not decided: **slowing down in curves** (v² · curvature within a
  sideways limit, using the spline's curvature) and **actuation noise**
  (wheel slip; Patric: skip for now). The point of the latter would be that
  feedback corrects actuation errors but not localization errors.

### The bigger ideas (details in IDEAS.md)

All waiting for a go-ahead and a design round:
- **Emergency stop and what comes after it** (slides 39–40): protective vs
  latched stop, speed-dependent safety field (v²/2a, the same as the braking
  profile), the recovery ladder, operator interventions as the metric, a
  "lecture mode" that freezes and asks the class what to do. Three open
  questions are listed in IDEAS.md. Probably belongs in `run_avoid.py`.
- **Path smoothing by optimization** (slides 16–22) and **robot shape**
  (slide 10), lower priority.
- Done: grid demo, cost map for clearance (slides 13–15), localization
  jitter (slide 4 idea), spline smoothing, reactive local planners
  (slides 26–33) as `run_avoid.py`.

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
  the grid demo's own (keys 1–4, then the first planning worlds),
  `worlds/avoid/*.json` the avoidance demo's (keys 1–4; the planning format
  plus `unmapped` and `movers`).
