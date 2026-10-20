# Engineering log

Problems found during development, how each was tracked down, and what was changed.
Most were caught by a failing test, not by watching the simulator.

---

### 1. A "perfect" sensor crashed (lidar)

**Symptom.** The lidar tests crashed with an assertion inside `<random>`.
**Cause.** Tests use a perfect sensor with noise σ = 0, and `std::normal_distribution`
requires σ > 0.
**Fix.** Only add noise when σ > 0.
**Lesson.** Edge cases of library functions; tests with "ideal" settings exercise them.

### 2. A test that could not fail (motion)

**Symptom.** None — the test passed. But the claim behind it was wrong.
**Investigation.** The "one full circle returns to the start" test was meant to prove exact arc
integration. Simulating the naive (Euler) method showed it **also** returns exactly to the start
after a full lap: with constant turning, the polygon it draws closes up.
**Fix.** Check the position after a **quarter** lap, where Euler is ~6 mm off and exact integration
is exact.
**Lesson.** A test is only worth something if it would fail for the wrong implementation.

### 3. One scan made nearby cells 100% certain (mapping)

**Cause.** Every beam voted separately; near the robot dozens of beams of one scan cross the same cell.
**Fix.** One vote per cell per scan. Beams from one scan are one look at the world, not
independent evidence.

### 4. Walls a cell too thick, then false walls beside corners (mapping)

**Cause.** In this simulator walls lie exactly on cell boundaries, so noise drops about half of the
hits into the free cell in front of a wall. A first fix (push the hit point 5 cm deeper) pushed hits
*out the other side* of thin obstacles near their corners. A second ("the first boundary inside a
±5 cm window") picked the free cell beside a wall for beams skimming along it.
**Fix.** The hit goes to the boundary **closest** to the measured distance, and each cell's
per-scan vote is a **majority** of its beams rather than "any hit wins" — so one stray beam is
outvoted by the many that passed through.
**Lesson.** Debugging by printing the actual numbers (the log-odds of every cell) beat guessing:
each print showed exactly which cell was wrong.

### 5. A corner cell marked free (mapping)

**Cause.** Dropouts (no return) were treated as "nothing within 3.5 m" and cleared every cell along
the beam — straight through walls. Ordinary wall cells get enough hits to outvote that; a corner
cell, rarely hit, did not.
**Fix.** A missing reading is ambiguous ("nothing there" or "sensor failed"), so it does not vote.

### 6. My test checked a wall the sensor could not see (mapping)

A test expected a wall 3.69 m away to be mapped; the lidar's range is 3.5 m. The map was right,
the test was wrong. Kept as a test of the opposite: out-of-range walls stay unknown.

### 7. The robot bumped walls in the maze (navigation)

**Investigation.** Logging every bump showed one per run, always at the same spot — a dead end
where the robot turns around in a corridor with 7 cm to spare on each side.
**Fix.** Before sending a command, simulate the arc it would drive and check it against the robot's
own map; if it would hit a known wall, turn on the spot instead. Bumps went to zero in all runs.

### 8. …then it got stuck instead (navigation)

**Investigation.** Tracing commands showed the robot rotating back and forth on the spot, 21 cm off
a corridor's centre line next to a wall corner: every arc towards the path was (correctly) refused.
**Cause.** Pure pursuit cuts corners; each cut shifted it sideways until it was wedged.
**Fix.** The lookahead point never passes the next corner of the path, so the robot drives to the
corner, turns, and continues on the centre line.
**Lesson.** The safety check fixed the *symptom* (bumps); the *cause* was the controller's geometry.

### 9. A crash that moved around — and a wrong first explanation

**Symptom.** Programs built with optimisation sometimes crashed with a segmentation fault, but not
with bounds checks on, not under the debugger, not with the undefined-behaviour checker. That
pattern usually means undefined behaviour, so the first suspect was a division by zero producing
NaN in the controller.
**First (wrong) conclusion.** A NaN check found nothing. A crashing debug program had a
`printf("%zu")` line that a working one did not, so I blamed that — without proving it.
**What it really was.** Later, after the switch to GoogleTest, the test program crashed in Git
Bash but passed in PowerShell. Same binary, different shell — so not the
code. On Windows a program built with GCC loads the C++ runtime from `libstdc++-6.dll`, from the
first folder on `PATH` that has one. Git Bash puts **Git's own, older** copy first. Experiments
settled it: the old "`%zu`" program runs fine with the right DLL, and every build runs in every
shell once the runtime is linked statically.
**Fix.** On MinGW the runtime is linked into the executable (`-static`), so it no longer depends on
`PATH` — and the `.exe` runs on computers without a compiler installed.
**Lesson.** "It crashes in one environment and not another" — compare the environments before the
code. And a fix that was never verified is a guess: the `%zu` explanation was wrong.
### 10. Tuning that only looked like it worked (navigation)

**Symptom.** A sweep over controller settings (lookahead, turn-on-the-spot angle, curve slowdown)
on 3 noise seeds found **one** setting out of 24 that never failed.
**Decision.** One lucky setting out of 24 is a sign of a structural problem, not a solution.
Every later change was judged on 40 runs (20 noise seeds × 2 maps).

### 11. "No path" because of false walls far away (mapping → navigation)

**Investigation.** The stuck runs reported *NoPath*. Listing every cell where the robot's map
disagreed with reality showed false walls 3–3.5 m away, at the far end of a long corridor.
**Cause.** Far-away beams skim corridor walls at a shallow angle, where noise can pick the wrong
cell, and few beams reach each far cell to outvote it. Once marked, the robot never went back to
look, so the error was permanent.
**Fix.** A **usable range** (2.5 m of 3.5 m): beyond it a beam only marks free space; its hit is
ignored (GMapping's `maxUrange`). That left 2 failures in 40 runs.

### 12. The last two failures

- **Wedged at an S-bend.** Each re-plan started the path at the robot and jumped to the *next*
  cell's centre, skipping its own cell's centre — at a turn, a diagonal cut across the corner.
  **Fix:** keep the own cell's centre unless the robot has passed it.
- **One false wall in the only gap to the goal**, from a single hit vote.
  **Fix:** classifying a cell needs at least two agreeing votes either way (free already did).

**Result:** 80 of 80 runs (40 noise seeds × 2 maps) reach the goal with zero bumps; the test suite
runs 20 of them every time.

---

## Localization (stage 5)

### 13. The particle cloud collapsed while the robot stood still

**Symptom.** In the viewer, with no starting guess, all 3000 particles collapsed onto one wrong
point within a second; the robot believed it had arrived and never moved.
**Cause.** Every scan was used to update the filter, including the same scan over and over while
the robot stood still. Each repeat sharpened the same weights; with no motion there was no motion
noise to spread the particles out again.
**Fix.** Only update after the robot has moved or turned a minimum amount, as AMCL does
(`update_min_d`, `update_min_a`).

### 14. Augmented MCL could not see that it was wrong

**Symptom.** Finding itself from nothing and recovering from a kidnapping each failed in about half
the runs, whatever the settings (12 combinations of tempering, resampling rule and sensor
softness were measured).
**Cause.** Augmented MCL decides it is lost when the recent scan fit drops below the long-term
average fit. But the long-term average adapts to whatever fit the filter has: a filter that settles
in the wrong place makes that mediocre fit its baseline and never notices.
**Fix.** An absolute health check that does not adapt: the share of beams that fit the map from the
estimate. Tempering and conditional resampling, tried along the way, did not help and were removed.

### 15. "Outside the map" counted as a perfect fit

**Symptom.** Traces of failing runs showed wrong estimates with a scan fit of 97–100%, always near the
map's edge.
**Cause.** The distance field returned 0 ("right at a wall") for points outside the map, so from a
pose near the edge, every beam ending outside the map counted as a perfect match.
**Fix.** Outside the map gets the maximum distance (nothing is known there), and distances are
measured to wall *surfaces* (a beam cannot end deep inside a wall). Recovery times dropped from
up to 158 s to about 20 s.

### 16. Tuned on 10 seeds, failed on the next 10

A setting that worked on seeds 1–10 failed on half of seeds 11–20. Since then every result is
reported over 20–30 seeds that were not used for tuning.

### 17. The tests drove with the true pose; the viewer did not

**Symptom.** All tests passed, but in the viewer the robot, starting with no idea where it was,
settled on a wrong place and turned back and forth for ever.
**Cause.** The global-localization tests drove the robot with its **true** pose, testing the
filter on its own. In the viewer the robot acts on its **estimate** from the first second, driving
somewhere based on a wrong belief. The wrong place also fitted 80–93% of the beams, so with a
"lost" threshold of 50% it never counted as lost.
**Fix.** The robot does not navigate until localization is healthy; it turns on the spot instead
(Nav2's spin recovery). The "lost" threshold went to 85% (correctly localized, 95–100% of beams
fit). The tests now run the realistic case: the robot acts only on its own estimate.
**Lesson.** Test the system as it is really used, not just its parts.

### 18. A wrong pose in a corridor fitted 93% of the beams

**Symptom.** Still in the viewer: a wrong estimate in a narrow corridor kept a fit of up to 93%.
**Cause.** The likelihood field only looks at where beams **end**. In a narrow corridor almost any
end point lies near some wall, even if the beam would have had to pass through another wall to get
there.
**Fix.** The health check casts each beam on the map from the estimate and compares the expected
range with the measured one, which also uses the free space along the beam. Too slow for thousands
of particles, cheap for one estimate. Result over 30 seeds each: kidnap recovery 30/30 (slowest
9 s), finding itself 30/30 (slowest 23 s), tracking 30/30 (worst 10.5 cm).
