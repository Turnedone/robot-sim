# Devlog

A chronological record of how robot-sim was built: what was built at each stage, the decisions
and why, the problems hit and how they were fixed, and what is next. Built in October 2026 with an
AI coding assistant (Claude); the reasoning below is what the project is built on and is meant to
be understood and explained, not just read. Detailed write-ups of each bug are in the
[engineering log](engineering-log.md); the ideas are explained per stage in docs 01–07.

---

## Day 1 — Why this project

**Goal:** a portfolio project that shows real robotics skills (C++, autonomy algorithms, ROS), to
move from database development (SQL / PL/SQL / Oracle APEX) towards autonomy engineering.

**Decision: build the autonomy stack from scratch instead of only using ROS packages.** Using Nav2
shows you can configure a tool; writing kinematics, a lidar model, mapping, planning, control and
localization yourself shows you understand what the tools do. ROS integration comes last, on top.

**Decision: keep the robot logic free of graphics.** `robot_core` has no window, no ROS, no I/O
beyond map files. The raylib viewer only draws. This paid off three times: the logic was testable
without a screen, it ran in CI, and it later dropped into ROS 2 nodes unchanged.

---

## Stage 1 — Motion

**Built:** a differential-drive robot with a pose (x, y, heading), velocity commands (v, ω), wheel
speed and acceleration limits, collision as a "bumper", and a fixed 50 Hz simulation step.

**Decisions:**
- *Exact arc integration* instead of Euler steps, because errors from straight-line steps grow with
  speed × timestep, which matters for odometry later.
- *When a wheel is too fast, scale both wheels*, which keeps the curvature, so the robot follows the
  planned curve and only slows down. Clipping one wheel would change the turn radius.
- *Fixed timestep with an accumulator*, so runs are repeatable and a slow frame cannot make the robot
  jump through a wall.

**Problem:** the test meant to prove exact integration (a full circle returns to the start) also
passed for the naive method, because a regular polygon closes too. **Fix:** test a quarter circle.
Lesson: a test is only useful if it would fail for the wrong implementation.

---

## Stage 2 — Lidar

**Built:** a simulated 180-beam 2D lidar with noise, dropouts, range limits and a 5 Hz scan rate.

**Decision: DDA ray casting** (step from grid line to grid line) instead of marching in small
steps, because it is exact and fast and never skips a corner. The same walk was reused for mapping.

**Problem:** a "perfect sensor" test crashed: `std::normal_distribution` refuses a zero spread.
**Fix:** only add noise when the spread is positive.

---

## Stage 3 — Mapping

**Built:** an occupancy grid using log-odds. Beams vote "free" along their path and "occupied"
where they end.

**Decisions and problems (each found by a failing test):**
- Dozens of beams per scan cross the same nearby cell, so counting each made one scan look certain.
  → **one vote per cell per scan, by majority.**
- Noise put half the wall hits in the free cell in front of the wall. → **±5 cm surface tolerance**:
  the wall is the cell boundary closest to the measured distance.
- Beams with no return were clearing cells through walls. → **a missing reading does not vote.**
- Far-away grazing beams created false walls in corridors. → **usable range** of 2.5 m of the 3.5 m
  (the same idea as GMapping's `maxUrange`).
- Caps on the log-odds so a map can still change its mind when something moves.

---

## Stage 4 — Planning and control

**Built:** A\* on the robot's own map (unknown space allowed but penalised), pure pursuit path
following, re-planning after every scan, and a check of each driving arc against known walls.

**Problems, mostly in the maze, where corridors leave 7 cm on each side of the robot:**
- Settings looked fine on 3 runs and failed on others. → judge every change on **many noise seeds**
  (later: 20–30 seeds not used for tuning).
- Pure pursuit cuts corners and wedged the robot sideways. → the aim point is **not allowed past the
  next corner** of the path.
- Bumps at dead ends. → **check the arc before driving it**, turn on the spot if it would hit.
- A crash that looked like undefined behaviour. The first explanation (a `printf`) was wrong; the
  real cause was an **older C++ runtime DLL on PATH** from Git. → link the runtime statically.
  Lesson: compare environments before blaming code, and verify an explanation before writing it down.

**Result:** 80/80 runs across both maps reached the goal without touching a wall.

---

## Making it professional

GoogleTest, CMake options (viewer, warnings as errors, sanitizers, MSVC), clang-format and
clang-tidy (which found an integer overflow risk and a floating-point loop counter), checksum-pinned
dependencies, an MIT license, and GitHub Actions CI on Linux (GCC, Clang), Windows (MSVC), with
AddressSanitizer + UBSan, plus a ROS 2 container job.

---

## Stage 5 — Localization

**Built:** wheel slip and odometry drift, then a particle filter (Monte Carlo localization, the
algorithm behind ROS's AMCL) against a saved map.

**The hardest stage. Problems in order:**
1. The particle cloud collapsed onto one point while the robot stood still. → **only update after
   moving** (AMCL's `update_min_d`).
2. Augmented MCL could not tell it was wrong: its long-term average adapts to a wrong fit. →
   an **absolute health check**.
3. "Outside the map" counted as a perfect fit, so guesses near the edge won. → outside = maximum
   distance; distances measured to wall **surfaces**.
4. Tuned on 10 seeds, failed on the next 10. → fresh seeds for every claim.
5. The tests drove with the true pose; the viewer acted on the estimate and got lost. → **don't
   navigate while lost** (turn on the spot, like Nav2's spin recovery) and test the real case.
6. A wrong pose in a narrow corridor still matched 93% of beam end points. → the health check
   **ray-casts** each beam and compares expected and measured range.

Also: a precomputed distance field so 6000 particles stay fast, and an adaptive particle count
(many while searching, few when confident).

**Result (30 fresh seeds each):** tracking 30/30, finding itself from nothing 30/30, recovering from
being kidnapped 30/30.

---

## Stage 6 — ROS 2

**Built:** four nodes (simulator, localizer, mapper, navigator) using standard topics and the TF
tree (`map → odom → base_link`), launch files, an RViz layout, and a test that runs the nodes
together and checks the robot really reaches a goal. CI builds and tests them in the official
`ros:jazzy` container.

**Problem:** a test threshold failed by 8 mm on a local machine. → derive limits from the actual
tolerances (goal tolerance + allowed localization error) instead of guessing.

---

## Stage 7 — SLAM

**Built:** a grid-based Rao-Blackwellized particle filter (the idea behind GMapping): each particle
holds a pose and its own map, scan-matches before adding a scan, and good histories survive
resampling.

**Problems:**
- The estimate lagged behind while exploring: beams ending in unmapped space scored as bad fits
  and pulled the match backwards. → **skip beams that end where the map has never looked.**
- The **corridor problem**: along a featureless stretch a scan does not say how far along you are. →
  an **odometry prior** in scan matching, with motion noise calibrated to the real odometry.
- A blunt navigation fix broke open-space driving. → check directly whether the straight line would
  hit a wall.
- A weak false wall blocked the only path for ever. → keep mapping while standing still, and a
  recovery that re-plans through weakly-supported walls (like Nav2's clear-costmap).

**Result:** with no map and slipping wheels, 40/40 explorations reached the goal; worst pose error 7 cm.

---

## Stage 8 — Gazebo

**Built:** a WSL2 setup (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2), first Nav2's own
TurtleBot3 demo, then **our localizer and navigator driving the TurtleBot3 in place of AMCL and Nav2**.

**What it took:** map origin handling (the ROS map starts at (−10, −10)), inflation for 5 cm cells
(and a fix when a test showed paths 5 mm too close to walls), path smoothing, the lidar's mounting
offset, simulated time, and Gazebo's model path.

**Result:** across the world in 14 s; estimate within 5 cm of Gazebo's ground truth.

---

## Lessons that came up again and again

- A test is only worth something if it can fail for the wrong implementation.
- When tuning stops helping, the problem is structural; trace one failing run instead.
- Judge changes on data you did not tune on.
- Test the system the way it is really used.
- Verify an explanation before writing it down; the first one is often wrong.

## What's next

- Grow the SLAM map instead of assuming its size; pose-graph SLAM with loop closure for large areas.
- Let the ROS navigator wait while localization is unhealthy (the core already does).
- Compare our stack and Nav2 on the same Gazebo routes (time, path length, clearance).
- Run on a real robot.
