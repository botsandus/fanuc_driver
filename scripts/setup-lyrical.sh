#!/usr/bin/env bash
# Set up the FANUC ROS 2 driver (Lyrical port) in a colcon workspace.
#
# Run it from a clone of this branch placed at <workspace>/src/fanuc_driver:
#
#   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
#   git clone --branch lyrical --single-branch --recurse-submodules \
#       https://github.com/botsandus/fanuc_driver.git
#   fanuc_driver/scripts/setup-lyrical.sh
#
# ROS 2 Lyrical (ros-lyrical-desktop) must already be installed:
# https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html
#
# SKIP_APT=1 skips the sudo apt step (packages must already be installed).
set -euo pipefail

step() { printf '\n\033[1;36m=== STEP %s ===\033[0m\n' "$1"; }
run()  { printf '\033[0;33m$ %s\033[0m\n' "$*"; "$@"; }
die()  { echo "setup-lyrical: $*" >&2; exit 1; }

repo="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
src="$(dirname "$repo")"
ws="$(dirname "$src")"
[[ "$(basename "$src")" == "src" ]] || die "expected the repo at <workspace>/src/fanuc_driver, found $repo"
[[ -f /opt/ros/lyrical/setup.bash ]] || die "/opt/ros/lyrical not found; install ros-lyrical-desktop first (see header)"

step "1: apt packages"
if [[ -z "${SKIP_APT:-}" ]]; then
    run sudo apt install -y git-lfs ros-dev-tools \
        ros-lyrical-force-torque-sensor-broadcaster ros-lyrical-ros2controlcli
else
    echo "SKIP_APT set; skipping"
fi
run git lfs install

step "2: source checkouts in $src"
cd "$src"
[[ -d fanuc_description ]] || run git clone https://github.com/FANUC-CORPORATION/fanuc_description.git
run git -C "$repo" submodule update --init --recursive

step "3: rosdep"
set +u; source /opt/ros/lyrical/setup.bash; set -u
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || run sudo rosdep init
run rosdep update
cd "$ws"
run rosdep install --ignore-src --from-paths src -y

step "4: colcon build (tests off: the test code is not ported)"
run colcon build --symlink-install --cmake-args -DBUILD_TESTING=0

printf '\n\033[1;32mDONE. source %s/install/setup.bash\033[0m\n' "$ws"
