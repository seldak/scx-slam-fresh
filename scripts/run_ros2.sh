#!/usr/bin/env bash
# SPDX-License-Identifier: MIT

set -eo pipefail

mode="${1:-build}"
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scx_fresh_dir="${SCX_FRESH_DIR:-$repo_dir/../scx_fresh}"
ros_setup="${ROS2_SETUP:-/opt/ros/lyrical/setup.bash}"
expected_distro="${ROS2_DISTRO_EXPECTED:-lyrical}"

case "$mode" in
    build|test) ;;
    *)
        echo "usage: $0 [build|test]" >&2
        exit 2
        ;;
esac

if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "$expected_distro" ]]; then
    echo "ROS 2 ${ROS_DISTRO} is active; expected ${expected_distro}." >&2
    echo "Start a clean shell or unset the existing ROS overlay." >&2
    exit 1
fi

if [[ ! -f "$ros_setup" ]]; then
    echo "ROS 2 ${expected_distro} is optional and was not found." >&2
    echo "Install it or set ROS2_SETUP to its setup.bash path." >&2
    echo "The standalone scheduler and simulator still build with plain 'make'." >&2
    exit 1
fi

# shellcheck disable=SC1090
source "$ros_setup"
set -u

if [[ "${ROS_DISTRO:-}" != "$expected_distro" ]]; then
    echo "Sourcing $ros_setup selected '${ROS_DISTRO:-none}', expected '$expected_distro'." >&2
    exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon is required for the optional ROS 2 workspace." >&2
    exit 1
fi

# Repeated local builds should not flood the desktop notification service.
if [[ -n "${COLCON_EXTENSION_BLOCKLIST:-}" ]]; then
    export COLCON_EXTENSION_BLOCKLIST="${COLCON_EXTENSION_BLOCKLIST}:colcon_notification.desktop_notification"
else
    export COLCON_EXTENSION_BLOCKLIST="colcon_notification.desktop_notification"
fi

build_args=(
    --base-paths "$repo_dir/ros2"
    --build-base "$repo_dir/.ros2-build"
    --install-base "$repo_dir/.ros2-install"
    --symlink-install
    --cmake-args "-DSCX_FRESH_DIR=$scx_fresh_dir"
)

colcon --log-base "$repo_dir/.ros2-log" build "${build_args[@]}"

if [[ "$mode" == "test" ]]; then
    colcon --log-base "$repo_dir/.ros2-log" test \
        --base-paths "$repo_dir/ros2" \
        --build-base "$repo_dir/.ros2-build" \
        --install-base "$repo_dir/.ros2-install" \
        --event-handlers console_direct+
    colcon --log-base "$repo_dir/.ros2-log" test-result \
        --test-result-base "$repo_dir/.ros2-build" --verbose
fi
