#!/usr/bin/env bash
# Builds and tests the robot_sim_ros package in a workspace at ~/robot_sim_ws,
# straight from this repository (no copy needed).
#   bash tools/wsl/build-ros2.sh          build only
#   bash tools/wsl/build-ros2.sh --test   build and run the tests
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
ws="$HOME/robot_sim_ws"
mkdir -p "$ws"
cd "$ws"
source /opt/ros/jazzy/setup.bash
colcon build --base-paths "$repo/ros2" --event-handlers console_cohesion+
if [ "${1:-}" = "--test" ]; then
    colcon test --base-paths "$repo/ros2" --event-handlers console_cohesion+ --return-code-on-test-failure
    colcon test-result --verbose
fi
echo "Built. Use it with:  source $ws/install/setup.bash"
