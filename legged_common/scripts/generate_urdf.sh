#!/usr/bin/env sh
REPO_ROOT="${LEGGED_CONTROL_REPO:-/workspace/src/legged_control}"
GENERATED_DIR="${REPO_ROOT}/legged_control"
mkdir -p "${GENERATED_DIR}"
rosrun xacro xacro "$1" robot_type:="$2" > "${GENERATED_DIR}/$2.urdf"
