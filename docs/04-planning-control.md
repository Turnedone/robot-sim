# 4. Planning and control: going somewhere on your own

Code: [`src/planner.*`](../src/planner.hpp), [`src/controller.*`](../src/controller.hpp),
[`src/navigator.*`](../src/navigator.hpp) · Tests: [`tests/test_navigation.cpp`](../tests/test_navigation.cpp)

The split follows ROS 2's Nav2: a **global planner** finds a route on the map, a **controller**
follows it, and a **navigator** connects them.

```
every scan (5 Hz):   lidar → map.integrate() → nav.replan()  →  A* path
every tick (50 Hz):  nav.command(pose, map) → pure pursuit → safety check → Twist → robot
```

## Planning: A\*

A\* finds the cheapest path through the grid. It explores cells in order of
**f = g + h**: `g` is the cost of the best path found so far to the cell, `h` an estimate of
the cost from there to the goal. Cells wait in a priority queue ordered by `f`.

- **Heuristic**: the *octile* distance (straight-line distance on an 8-connected grid). It never
  overestimates the true cost — it is **admissible** — so A\* is guaranteed to return the cheapest
  path, while exploring far fewer cells than Dijkstra (which is A\* with h = 0).
- **Moves**: 8 directions; straight costs 1, diagonal √2 (per cell).
- **Costs on top of distance**: unknown cell +50%, cell next to a known wall +200% (keeps the path
  away from walls — a tiny version of Nav2's costmap *inflation*).

### Planning on the robot's own map

The robot plans on what it **knows**, not on the true world:

- **Known walls** are blocked.
- **Unknown cells are passable** ("optimistic planning"); otherwise the robot could never plan
  into places it has not seen. When the lidar reveals a wall on the route, the next re-plan
  goes around it. The robot re-plans after **every scan**.
- **No diagonal squeezing past corners unless both corner cells are known to be free**:
  an unseen corner may turn out to be a wall.
- If no path exists (say the map has a wall in the only gap), the navigator reports **NoPath**,
  stops, keeps the goal and tries again after the next scan.

## Control: pure pursuit

Pure pursuit is a classic path follower (still used, e.g. Nav2's Regulated Pure Pursuit):

1. Find the path point closest to the robot (searching only forward from last time, so it never
   jumps back to an earlier part of the path).
2. Pick the **lookahead point**: the first path point at least 0.35 m away.
3. Express it in the robot's frame: `ahead` and `left` (y).
4. Drive the circular arc through it: **curvature κ = 2·left / d²**, then `ω = v·κ`.

Speed is lowered on tight curves (`v = vmax / (1 + 0.6|κ|)`) and near the goal or a corner
(`v ≤ distance`). If the target is more than ~57° off the nose, the robot turns on the spot first.

**Change from the textbook version: the lookahead point never passes the next corner of the
path.** Textbook pure pursuit starts turning before a corner and cuts it, shifting the robot
sideways. The maze's corridors leave only 7 cm on each side of the robot, and the cuts added up
until it was wedged 21 cm off-centre against a wall corner. Stopping the aim at the corner makes
the robot drive to it, turn, and continue along the centre line. Trade-off: it stops at every
corner of a grid path, which is slow in open space (path smoothing would fix this).

## Safety: checking the arc before driving it

Before a command is sent, the navigator simulates the arc it would follow (0.45 m ahead, in 4 cm
steps) and checks the robot's circle against cells **the robot's own map** believes are walls.
If the arc would hit one, the robot turns on the spot towards the path instead — a round robot can
always do that safely — and drives once the arc from its new heading is clear. Nav2's Regulated
Pure Pursuit has the same kind of check. In a maze run it refuses about 190 unsafe arcs.

## Re-planning details

- The path starts at the robot's actual position. The centre of the robot's **own** cell is kept
  in the path unless the robot has already passed it; skipping it at a turn made the path cut
  diagonally across the corner.
- The controller runs at 50 Hz; planning runs at 5 Hz (after each map update). A\* on these maps
  takes well under a millisecond.

## How it is tested

Besides unit tests for A\*, `test_navigation` runs the **whole loop without graphics**: the robot
starts knowing nothing, maps, plans, and drives, on both maps, **10 runs each with different
lidar noise**. Every run must reach the goal and the robot must never touch a wall.
Testing over many noise seeds matters: during development, several settings looked perfect on
3 runs and failed on others.
