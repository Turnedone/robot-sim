# Self-test

Questions an interviewer could ask about this project. Try answering each out loud before
opening the answer. Every answer links to where it is explained in detail.

## Motion

<details><summary>1. What are forward and inverse kinematics for a differential-drive robot, and where is each used?</summary>

Forward: wheel speeds → motion, `v = (vR + vL)/2`, `ω = (vR − vL)/L`; used to move the robot and,
on a real robot, for **odometry** from wheel encoders. Inverse: desired `(v, ω)` → wheel speeds,
`vL = v − ωL/2`, `vR = v + ωL/2`; used to execute velocity commands. [01-motion](01-motion.md)
</details>

<details><summary>2. Both wheels turn at −0.3 m/s. What are v and ω?</summary>

`v = −0.3 m/s` (straight backwards), `ω = 0`.
</details>

<details><summary>3. A command asks for more wheel speed than the motor has. What do you do and why?</summary>

Scale both wheels by the same factor, which keeps the curvature ω/v — the robot follows the planned
curve, only slower. Clipping one wheel changes the turning radius. [01-motion](01-motion.md)
</details>

<details><summary>4. Why integrate along exact arcs instead of Euler steps? When does it matter?</summary>

Euler approximates arcs with straight lines; its error grows with v·dt. Exact integration is
exact for constant velocities within a step. It matters for odometry at low update rates.
</details>

<details><summary>5. Why a fixed timestep, and what is the accumulator?</summary>

Determinism, no tunnelling through walls after a long frame, and real control loops run at fixed
rates. The accumulator banks real elapsed time and spends it in fixed 20 ms steps.
</details>

## Sensing

<details><summary>6. How does DDA ray casting work and why not just step along the ray?</summary>

It jumps from one grid-line crossing to the next (whichever of the next vertical / horizontal line
is nearer), visiting exactly the cells the ray crosses. Fixed-step marching is slow and can skip a
cell the ray only clips. [02-lidar](02-lidar.md)
</details>

<details><summary>7. Robot at (2, 3) facing 90°; beam at −90° reads 1.5 m. Where is the hit?</summary>

Beam direction on the map = 90° − 90° = 0°, so (3.5, 3).
</details>

<details><summary>8. What is TF? What is the difference between the map and odom frames?</summary>

ROS's library for chaining coordinate transforms between frames (map → odom → base_link → laser)
with timestamps. `odom` is smooth but drifts; `map` is globally correct but can jump; localization
publishes map → odom to absorb the drift. [02-lidar](02-lidar.md)
</details>

## Mapping

<details><summary>9. What is log-odds and why use it in an occupancy grid?</summary>

`l = log(p/(1−p))`. Bayesian updates with independent readings become additions; 0 = unknown.
</details>

<details><summary>10. Why cap the log-odds?</summary>

So the map can change its mind: a removed obstacle clears in seconds instead of needing as many
misses as it ever had hits.
</details>

<details><summary>11. A cell gets hit, miss, miss, hit, hit. Log-odds? Class?</summary>

3 × 0.85 − 2 × 0.41 = 1.73 → occupied (threshold +1.2).
</details>

<details><summary>12. Why only one vote per cell per scan?</summary>

Many beams of one scan crossing the same cell are one look at the world, not independent evidence;
counting each made a single scan look certain. [03-mapping](03-mapping.md)
</details>

<details><summary>13. Why do beams with no return not update the map?</summary>

"No return" can mean nothing in range or a sensor dropout; treating it as free space erased real
walls behind the gap.
</details>

<details><summary>14. Why is the usable range shorter than the sensor range?</summary>

Far hits are unreliable (few beams per cell, shallow angles in corridors); they caused permanent
false walls. Beyond 2.5 m beams only clear space. [engineering log #11](engineering-log.md)
</details>

## Planning and control

<details><summary>15. How does A* work, and why is it optimal?</summary>

Expands cells in order of `f = g + h`. With an admissible heuristic (never overestimates — here
the octile distance) the first time the goal is expanded its path is the cheapest. Dijkstra is A*
with h = 0. [04-planning-control](04-planning-control.md)
</details>

<details><summary>16. Should unknown cells be passable for the planner?</summary>

Yes, with extra cost (optimistic planning): otherwise the robot cannot plan into unexplored space.
Re-plan when the map changes. But never cut a corner past an unseen cell.
</details>

<details><summary>17. Explain pure pursuit and its formula.</summary>

Aim at a point a lookahead distance ahead on the path and drive the arc through it:
κ = 2·y/d², where y is the target's sideways offset in the robot frame and d its distance; ω = v·κ.
</details>

<details><summary>18. What is pure pursuit's weakness and how did you deal with it?</summary>

It cuts corners (more with longer lookahead). In narrow corridors that wedged the robot, so the
lookahead point is not allowed past the next corner of the path. Trade-off: it stops at corners.
</details>

<details><summary>19. How does the robot avoid hitting walls its controller does not foresee?</summary>

Before sending a command it simulates the arc and checks it against walls in its own map; if
unsafe it rotates on the spot (always safe for a round robot).
</details>

<details><summary>20. How do you know the navigation is reliable, not just lucky?</summary>

The closed-loop test runs the whole stack on two maps with 10 different noise seeds each; all must
arrive with zero bumps. During development 80 runs were used to judge each change.
</details>

## Localization

<details><summary>21. Why does odometry drift, and why does it get worse over time?</summary>

Wheels slip and are never exactly the assumed size. Each step's small error is added to the pose,
and heading errors turn into position errors that grow with the distance driven. Nothing corrects
it. [05-localization](05-localization.md)
</details>

<details><summary>22. Explain the three steps of a particle filter.</summary>

Predict: move every particle by the odometry change plus its own noise. Correct: weigh each by how
well the scan fits the map from its pose. Resample: draw a new set in proportion to the weights.
</details>

<details><summary>23. What is the likelihood field model, and what is its weakness?</summary>

A beam is likely if its end point is close to a wall in the map (precomputed distance field) —
fast and smooth. It ignores the free space along the beam, so a wrong pose can fit if beam ends
happen to land near some wall, e.g. in a narrow corridor.
</details>

<details><summary>24. Why only update the filter after the robot has moved?</summary>

A standing robot sees the same scan repeatedly; re-weighing by it over and over collapses the
cloud onto one point, and without motion noise nothing spreads it out again (AMCL's
update_min_d / update_min_a).
</details>

<details><summary>25. How does your filter notice that it is lost, and why not Augmented MCL?</summary>

It ray-casts the beams from the estimate and checks what share measure the expected range; below
85% for 10 updates while confident, it searches the whole map again. Augmented MCL compares recent
and long-term average fit, but the long-term average adapts to a wrong fit, so a filter that
settled in the wrong place never noticed. [engineering log 14–18](engineering-log.md)
</details>

<details><summary>26. Why should a robot not navigate while localization is unhealthy?</summary>

It would drive towards a goal based on a wrong belief about where it is. Turning on the spot
instead gives the filter new views without risk (Nav2's spin recovery).
</details>

## ROS 2

<details><summary>27. What are the map, odom and base_link frames, and who publishes which transform?</summary>

The driver/odometry publishes odom → base_link (smooth, drifts); localization publishes
map → odom (the correction, may jump). map → base_link is their composition. [06-ros2](06-ros2.md)
</details>

<details><summary>28. How do you compute map → odom from the filter's estimate?</summary>

map→odom = (map→base_link) · (odom→base_link)⁻¹, i.e. the estimate composed with the inverse of
the odometry pose.
</details>

<details><summary>29. Why does the map topic use "transient local" durability?</summary>

So a node that subscribes later (RViz, the navigator) still receives the last map, like a latched
topic in ROS 1.
</details>
