#!/bin/bash
# Entrypoint for tbuggy_vio Docker container.
# Sources ROS2 Humble and the colcon workspace on every container start,
# then executes whatever command was passed (default: bash).
set -e

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash

exec "$@"
