# Ideas for more navigation demos

Notes from going through `Lecture12_dd2410_Navigation.pdf` (66 slides) on
2026-09-25. Nothing here is implemented yet. The focus of this part of the
course is the **real-world aspects** of navigation: students have already
seen RRT and friends in known worlds in the planning module.

## What the demos already cover

| Slides | Topic | Covered by |
|---|---|---|
| 2–3 | Recap: robot-relative uncertainty, covariance | loc demos (`v` in EKF-SLAM, covariance heatmap) |
| 4 | "Show planning output from grid based and sampled based" | `run_planning.py`, A* vs RRT/RRT* |
| 9–10 | Occupancy grids: cell size, amount of expansion | `cell`, `inflate`, center/conservative cells |
| 11 | Paths close to obstacles, grid-aligned, not smooth | visible in every A* plan |
| 12 | "Why can we not just expand obstacles more?" | world 2: raise `inflate` and the door closes |
| 16 | Path smoothing: direct connection between nodes | `s` (shortcut) |
| 23 | Pure pursuit, lookahead circle | `run_pure_pursuit.py` |
| 24 | Environment not fully known, replanning | map-as-you-go mode (`k`) |
| 37 (type II) | A sensor outlier gets the robot stuck | `sig_rng` 0.1 m while mapping → no path |

## Grid demo (`run_grid.py`) — agreed as the starting point

A stripped-down demo that exposes only the real world and its grid, and the
options connected to them, so the discussion can start from the grid itself
before any planning. No planner, and no robot beyond a footprint circle for
scale.

**Shown:** the real geometry (outlines), the grid, and the obstacles grown by
`inflate` (dashed), i.e. what the grid is trying to approximate.

**Cell rule** (decided 2026-09-25):
- **Any overlap is the default and main rule**: a cell is occupied if
  anything is inside it. That is the natural rule for a binary map, and the
  safe one: it never loses an obstacle.
- **Center sample is kept only as the contrast**: cheap point sampling, and
  why it's dangerous (thin walls can vanish).
- **Area fraction dropped** (occupied if at least a fraction p of the cell is
  covered). It comes back naturally in the local-map demo instead (gap 1): as
  soon as rays also clear cells, a partly occupied cell gets both hits and
  pass-throughs, and counting them *is* a fraction, which is where occupancy
  probabilities come from.
- **Make "any overlap" exact:** test the cell square itself against the
  grown obstacle. The current "conservative" rule in `grid.py` is an
  approximation (centre within `inflate` + half the cell diagonal) that
  marks a few extra cells near corners.

**The two rules fail in opposite directions:** any overlap loses free space,
center sampling loses obstacles. With cell size c, a wall of width w covers
⌈w/c⌉ or ⌈w/c⌉ + 1 cells depending on where the cell borders fall, and a
gap of width g keeps ⌊g/c⌋ or ⌊g/c⌋ − 1 free cells. A door 1.5 cells wide
therefore has one free cell at some offsets and none at others.

**Moving the world** relative to the grid, to show that a coarse grid isn't
*a* picture of the world but one of many:
- arrow keys shift it in small steps (e.g. a tenth of a cell); mouse drag
  too;
- two keys rotate it a degree at a time (an axis-aligned wall and a 45° wall
  discretize very differently: a clean band vs a staircase);
- a sweep key slides the world across one full cell and back, so the grid
  flickers through its variants and returns: the effect is periodic in the
  offset.

**Making the differences visible, not just watchable:**
- **changed cells**: overlay of cells that differ from the grid at offset 0;
- **passable?**: two marked points (e.g. either side of a door) and a traffic
  light for whether they're connected through free cells. A flood fill, not a
  planner, so the demo stays about the grid. Headline effect: the same door
  is open or closed depending on a 10 cm shift;
- **strip chart** bottom-left: occupied fraction and passable yes/no against
  the offset, filled in during a sweep;
- **panel numbers**: cells, occupied %, cells changed vs offset 0, current
  offset and rotation.

**Worlds:** the existing ones, plus a few made for this: a door slightly
wider than two cells, a row of thin pillars, a 45° wall, a thin wall at a
sub-cell position.

**Still open:**
1. Inflation in this demo at all, or purely "world → cells" with inflation
   left to the planning demo? Keeping it makes the door-closing effect much
   stronger; leaving it out makes the demo cleaner.
2. Robot shape (slide 10, gap 7) as a footprint you drag around to see which
   cells it covers: here, or later?

## Gaps, in suggested order

1. **Local maps that forget (slides 35–38).** Best fit for the real-world
   angle. Today the map only accumulates. Add a map-update mode:
   - *last scan only* — answers slide 25's question of why not just use
     the latest reading;
   - *accumulate forever* — today's behaviour;
   - *clear along rays* — the sensor adds and removes (as in assignment 4);
   - *decay over time* — with the "let me try over here again" failure.

   To make the trade-off visible it needs **dynamic obstacles**: a person
   standing in a doorway who leaves (slide 36), a chair that is seen and
   then out of view (type I).

   With clearing, a binary map must decide what a cell that gets both hits
   and pass-throughs is (last writer wins, or occupied beats free); counting
   hits against misses instead gives a per-cell fraction, i.e. an occupancy
   probability. This is where the area-fraction idea dropped from the grid
   demo belongs.
2. **Reactive local planners (slides 26–33): potential field, VFH, DWA.**
   A second choice of executor next to pure pursuit, following the global
   plan's carrot or heading straight for the goal. The bug-trap world
   already shows potential-field local minima. DWA fits especially well:
   the simulator already has acceleration limits, which *is* the dynamic
   window.
3. **Cost map / Gaussian smoothing for clearance (slides 13–15).** A soft
   cost near obstacles for A*, as the answer to slide 12's question. Keeps
   the narrow passage usable while preferring its centre.
4. **Pure pursuit with noise levels (slide 4 idea).** Noise on the pose
   the controller uses and/or on actuation, to show the lookahead trade-off
   under noise (a short lookahead amplifies jitter). True-only noise, or
   true/model if a filter estimates the pose — a link to the loc demos.
5. **Emergency stop / safety layer (slides 39–40)** — see the next section.
6. **Path smoothing by nonlinear optimization (slides 16–22)**, next to
   the shortcutting.
7. **Robot shape (slide 10).** A non-circular footprint, where inflating
   by one radius is no longer exact.

Left as slides/videos: social navigation and proxemics (41–44), the case
studies (45–64).

**Open structure question:** items 1, 2 and 5 are layers of the control
hierarchy on slide 7. Proposal: put them in `run_planning.py` as a choice of
executor (pure pursuit / potential field / VFH / DWA) plus map-update modes
and a safety layer, rather than new demos. Alternative: a separate
`run_avoid.py` focused on local behaviour.

## Emergency stop: and then what?

Patric's angle: the interesting part is not the stop itself but **what to
do after it**. Many robots simply expect an operator to come and reset the
e-stop.

### Two different kinds of stop

| | Protective stop | Emergency stop |
|---|---|---|
| Triggered by | something inside the safety field (lidar zone ahead) | contact (bumper), or a person pressing the button |
| After the cause is gone | the robot may resume by itself | latched: someone must deliberately reset it |
| Cost | seconds | an operator walking over, minutes to hours |

Roughly how industrial mobile-robot standards treat it: ISO 3691-4
(driverless trucks) allows automatic restart after a protective stop once
the field is clear; resetting an e-stop must be manual and must not by
itself restart motion (ISO 13850). **Check the exact wording before putting
it on a slide.**

- **The safety field should scale with speed:** length = braking distance
  v²/2a + margin — the same expression as the brake-into-goal profile in
  `Follower.brake_for_goal`. Drive faster and the field grows; a too-short
  field turns protective stops into collisions.
- **What the demo does today is already a latched e-stop** (robot turns
  red, waits for `r`). It just isn't framed as one, and `r` is a full
  restart rather than "operator resets and the mission continues".

### Recovery after a stop

The robot usually doesn't know *why* it's in trouble: the map was wrong
(vanished thin wall), tracking error (too little inflation), a person
stepped in, localization drift, a sensor outlier. Each recovery fixes some
causes and makes others worse — that's the discussion point. A typical
escalation ladder (what ROS Nav2's recovery behaviours do):

1. **Wait** — fixes dynamic obstacles; useless for static ones.
2. **Clear the local map and rescan** — fixes outliers and stale
   obstacles (slide 37); if the obstacle is real it comes straight back.
3. **Rotate in place to see more** — it has a 360° lidar here; a real
   robot often doesn't.
4. **Back up along its own trail** — the trail was free a moment ago, but
   many robots have no rear sensor: this is reversing blind.
5. **Replan with less inflation** — gets through the tight spot at the
   price of the margin against tracking error; exactly how you cause the
   next collision.
6. **Give up and call an operator** — safe, but what you're trying to
   avoid.

**Metric to show:** operator interventions per hour or per km, next to time
lost — the number that decides whether a system is viable. Links to the
case studies: Harry Plotter has an operator nearby (a latched stop is
acceptable); a hospital delivery robot has staff around, but every call
annoys them; STRANDS aimed at 100 days of autonomy, where "wait for an
operator" doesn't scale.

### How to stage it

- **Mode A, automatic:** runs the recovery ladder, counts interventions.
- **Mode B, lecture:** when the robot stops, the simulation freezes and
  the panel asks "what should the robot do?", one key per option. Pick one
  with the students and watch the consequence (e.g. backing up blind in
  world 8, replanning with less inflation in world 2 until it clips the
  door frame).
- **Scenarios:**
  - the thin-wall collision (map wrong);
  - a person who steps into the corridor and leaves after 10 s (needs the
    dynamic obstacles from gap 1);
  - a sensor outlier blocking a doorway (clearing fixes it).

### Open questions

1. Model both stop kinds (protective field + latched collision e-stop), or
   only the latched one?
2. Mode A, Mode B, or both? (B looks like the better fit for a lecture.)
3. Build this together with dynamic obstacles and local-map forgetting
   (gap 1)? Several recoveries only make sense once obstacles can move or
   disappear.
