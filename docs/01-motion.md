# 1. Motion: a differential-drive robot

Code: [`src/robot.hpp`](../src/robot.hpp), [`src/robot.cpp`](../src/robot.cpp) ·
Tests: [`tests/test_robot.cpp`](../tests/test_robot.cpp)

The robot is modelled on a TurtleBot3-sized robot: two driven wheels on one axle
0.30 m apart, a round body of radius 0.18 m, top wheel speed 0.6 m/s.
It steers only by spinning its wheels at different speeds.

## State and commands

- **Pose** `(x, y, θ)`: position in meters in the map frame and heading in radians
  (0 = facing +x, counter-clockwise positive). The map frame has y pointing **up**,
  as in ROS (REP 103) — the viewer flips it for the screen.
- **Twist** `(v, ω)`: forward speed (m/s) and turn rate (rad/s). This is what a planner or a
  joystick asks for, the same idea as `geometry_msgs/Twist` on ROS's `/cmd_vel`.

## Kinematics

When the wheels turn at different speeds the robot rotates around a point beside it, the
**instantaneous centre of rotation (ICR)**. Both wheels move on circles around that same point;
the outer wheel's circle is larger, so it must move faster.

With R the distance from the robot's centre to the ICR and L the wheel separation:

```
vL = ω (R − L/2)          vR = ω (R + L/2)
```

Subtracting and adding gives:

| | Formula | Used for |
|---|---|---|
| **Forward kinematics** (wheels → motion) | `v = (vR + vL) / 2`, `ω = (vR − vL) / L` | `step()`: moving the robot; on a real robot, turning wheel-encoder readings into **odometry** |
| **Inverse kinematics** (motion → wheels) | `vL = v − ωL/2`, `vR = v + ωL/2` | `setCommand()`: turning a `Twist` into motor speeds |

Example: `vL = 0.4`, `vR = 0.6` → `v = 0.5 m/s`, `ω = 0.67 rad/s`, turning radius `v/ω = 0.75 m`.

## Keeping the curve when a wheel is too fast

Asking for `v = 0.5, ω = 2.0` (a 0.25 m radius turn) needs `vR = 0.8 m/s`, above the 0.6 m/s limit.

- Clipping only that wheel to 0.6 gives `v = 0.4, ω = 1.33` — a **0.30 m** turn. The robot no
  longer follows the curve that was planned, and could clip a corner.
- Scaling **both** wheels by the same factor (0.6/0.8) gives `v = 0.375, ω = 1.5` — still a
  **0.25 m** turn, just slower.

What is preserved is the **curvature** κ = ω/v. The path's geometry is what keeps the robot safe;
speed can always give way. (Test: *"speed limit: the curve is kept, only slower"*.)

Wheels also cannot change speed instantly: each ramps towards its target at up to 1.2 m/s².

## Integrating the motion: exact arcs

Each simulation step (20 ms) the robot must move according to `v` and `ω`.

- **Euler integration** — "drive straight for dt, then turn" — approximates the curve with short
  straight lines. Its position error is about `v·dt`: measured on a 1.2 m circle it is 0.6 cm at
  50 Hz and 3 cm at 10 Hz.
- **Exact integration** (used here): with `v` and `ω` constant during a step the robot moves on a
  true arc of radius `R = v/ω`:

  ```
  θ' = θ + ω·dt
  x' = x + R (sin θ' − sin θ)
  y' = y − R (cos θ' − cos θ)
  ```

  Driving straight (ω ≈ 0) has its own branch, since R would be infinite.

It matters most for **odometry** read at low rates, where small per-step errors add up to drift.
It is only exact while the wheel speeds are constant within a step; during acceleration there is a
tiny error, as in real odometry code.

Note on testing it: after one **full** lap the naive method also closes up exactly (the polygon
closes), so a full-circle test cannot tell the methods apart. The test checks a **quarter** lap,
which only exact integration lands on precisely. (See the [engineering log](engineering-log.md).)

## Fixed-timestep simulation

Physics runs in fixed 20 ms steps (50 Hz), independent of the screen's frame rate, using an
accumulator in the viewer's main loop:

```cpp
accumulator += frameTime;          // bank the real time that passed (capped at 0.25 s)
while (accumulator >= 0.02) {      // spend it in exact 20 ms chunks
    robot.step(0.02, grid);
    accumulator -= 0.02;
}
```

Why:
- **Deterministic**: the same inputs give the same run on any computer, so runs and tests repeat.
- **No tunnelling**: one long frame (say 0.5 s) would otherwise move the robot 30 cm in one jump,
  through a thin wall, because collisions are only checked at the end of a step.
- **Like real robots**: control loops run on fixed-rate timers (a ROS 2 timer, `ros2_control`),
  and controllers are tuned for a constant step.

The 0.25 s cap stops a long freeze from triggering hundreds of catch-up steps at once.

## Collisions: a bumper

If a step would make the robot's circle overlap a wall cell, the step is rejected and both wheels
stop — like a real bumper switch. The circle-vs-square test finds the point of the cell nearest
to the robot's centre and compares that distance with the radius. Outside the map counts as wall.
