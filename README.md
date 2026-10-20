# robot-sim

A 2D mobile-robot simulator in modern C++ (C++17) that implements a small but complete
**autonomy stack** from scratch: a differential-drive robot with a noisy lidar builds its
own map, plans a path on it, and drives there — starting from knowing nothing.

![The robot planning through an unexplored maze](docs/images/navigation.png)

*The robot (blue) has only mapped its corner of a maze (white = free, black = wall,
grey = never seen). The orange line is its current A\* plan, which goes optimistically
through unknown space and is re-planned five times a second as the lidar reveals walls.*

## What it does

| Stage | What the robot can do | Main ideas | Docs |
|---|---|---|---|
| Motion | Drives like a real two-wheeled robot | Differential-drive kinematics, exact arc integration, wheel speed/acceleration limits, fixed-timestep simulation | [01-motion](docs/01-motion.md) |
| Sensing | Sees the world with a simulated 2D lidar | DDA ray casting, sensor noise model, dropouts, range limits, sensor frames | [02-lidar](docs/02-lidar.md) |
| Mapping | Builds its own map from noisy scans | Occupancy grid, log-odds Bayesian updates, per-scan voting, usable range | [03-mapping](docs/03-mapping.md) |
| Planning & control | Goes anywhere you click, re-planning as it learns | A\* on the robot's own map, optimistic planning, pure pursuit, arc collision checking | [04-planning-control](docs/04-planning-control.md) |
| Localization | *next:* knows where it is although its wheels slip | Particle filter (Monte Carlo localization) | — |

How it was debugged — the problems the tests found and how each was tracked down — is in the
[engineering log](docs/engineering-log.md). Questions to check your understanding are in the
[self-test](docs/self-test.md).

## Design

```
            robot_core  (no graphics, unit-tested)                   robot_sim (viewer)
 ┌────────────────────────────────────────────────────────────┐    ┌─────────────────┐
 │ grid       the true world, map frame (meters, y up)        │    │ raylib window   │
 │ robot      DiffDriveRobot: kinematics, limits, bumper      │    │ keyboard/mouse  │
 │ lidar      castRay (DDA), Lidar: noise, dropouts, ranges   │ ◄──│ draws whatever  │
 │ occupancy  OccupancyMap: log-odds mapping from scans       │    │ the core        │
 │ planner    A* on the OccupancyMap                          │    │ produces        │
 │ controller PurePursuit path follower                       │    └─────────────────┘
 │ navigator  Navigator: re-plan on every scan, safety check  │
 └────────────────────────────────────────────────────────────┘
```

The robot logic (`robot_core`) knows nothing about graphics; the viewer only draws what the
logic produces. That keeps the logic testable on its own, and means it could be wrapped in
ROS 2 nodes unchanged, with RViz replacing the viewer. The structure mirrors ROS conventions
on purpose: `Twist` commands (like `/cmd_vel`), a `LaserScan` with the same fields as
`sensor_msgs/LaserScan`, a map frame with y up (REP 103), and a planner/controller split like Nav2.

## Build and run

Needs a C++17 compiler and CMake ≥ 3.25. raylib (for the window) is downloaded automatically.

```bash
cmake -S . -B build
cmake --build build
./build/robot_sim                      # warehouse map
./build/robot_sim maps/maze.txt --go   # maze, robot sets off for the goal by itself
./build/robot_sim --text               # print the map in the terminal instead
```

On Windows with MinGW add `-G "MinGW Makefiles"` to the first command.

### Controls

| Input | Action |
|---|---|
| left click | send the robot to that cell |
| G | send the robot to the map's goal (green) |
| arrow keys | drive by hand (cancels any goal) |
| right click | add / remove a wall — the robot's map catches up as it sees the change |
| M | switch view: robot's map / true map / lidar only |
| L | show / hide lidar rays |
| R | reset |

The header shows the robot's pose and wheel speeds, how much of the map it has explored,
and how many cells it has **wrong** (sure about, but the opposite of reality) — this stays at 0.

## Tests

```bash
ctest --test-dir build --output-on-failure
```

| Suite | Checks |
|---|---|
| `test_robot` | Motion against hand-worked answers: straight lines, spins, exact arcs, speed caps keep curvature, acceleration, bumper, angle wrapping, map frame |
| `test_lidar` | Ray distances (straight, diagonal, off-centre), range limits, noise statistics, repeatability with a fixed seed |
| `test_mapping` | Log-odds maths, one scan is not enough, hidden cells stay unknown, out-of-range walls stay unknown, removed obstacles fade, a noisy lidar maps a room with zero wrong cells |
| `test_navigation` | A\* (gaps, walled-in goals, no corner cutting, preferring known space) and the **whole robot closing the loop**: on both maps, 10 runs each with different sensor noise, it must reach the goal every time without touching a wall |

## Maps

Plain text: `#` wall, `.` free, `S` start, `G` goal; one character = one 0.5 m cell.
See [maps/](maps).

## Known limitations and next steps

- **Localization.** The robot currently knows its true pose. Next: wheel slip, odometry drift and a particle filter.
- **Path smoothing.** A\* paths on a grid have many small corners; the controller stops and turns at each one, which is safe in narrow corridors but slow in open space. Line-of-sight smoothing would fix this.
- **ROS 2.** Wrap the core as ROS 2 nodes and run it against Gazebo / Nav2.
