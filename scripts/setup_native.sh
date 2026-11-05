#!/bin/bash
# scripts/setup_native.sh
# One-shot native (non-Docker) setup for rbot on Ubuntu 24.04 + ROS 2 Jazzy.
#
# Installs every runtime dependency, checks the Gazebo CLI, runs rosdep,
# builds the workspace, and optionally launches the simulation.
#
# Usage:
#   bash scripts/setup_native.sh          # install deps + build only
#   bash scripts/setup_native.sh --run    # install deps + build + launch Gazebo
#   bash scripts/setup_native.sh --headless --run
#
# The script is idempotent: re-running it only installs what is missing.
# NOTE: no 'set -u' here — ROS 2 / colcon setup.bash files reference
# unset variables (AMENT_TRACE_SETUP_FILES, COLCON_TRACE) and would abort.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

RUN=false
EXTRA_LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --run)      RUN=true ;;
        --headless) EXTRA_LAUNCH_ARGS+=("headless:=true") ;;
        *)          EXTRA_LAUNCH_ARGS+=("$arg") ;;
    esac
done

log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[WARN] %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m[FAIL] %s\033[0m\n' "$*" >&2; exit 1; }

[ -d /opt/ros/jazzy ] || die "ROS 2 Jazzy not found at /opt/ros/jazzy"
if [ -z "${ROS_DISTRO:-}" ]; then
    # shellcheck disable=SC1091
    . /opt/ros/jazzy/setup.bash
fi

log "Installing ROS 2 Jazzy runtime dependencies"
PKGS=(
    ros-jazzy-xacro
    ros-jazzy-joint-state-publisher
    ros-jazzy-joint-state-publisher-gui
    ros-jazzy-ros2-control
    ros-jazzy-ros2-controllers
    ros-jazzy-gz-ros2-control
    ros-jazzy-ros-gz
    ros-jazzy-nav2-bringup
    ros-jazzy-nav2-mppi-controller
    ros-jazzy-nav2-smac-planner
    ros-jazzy-nav2-velocity-smoother
    ros-jazzy-nav2-lifecycle-manager
    ros-jazzy-slam-toolbox
    ros-jazzy-robot-localization
    ros-jazzy-imu-filter-madgwick
    ros-jazzy-teleop-twist-keyboard
    ros-jazzy-teleop-twist-joy
    ros-jazzy-joy
    ros-jazzy-rmw-cyclonedds-cpp
    ros-jazzy-pcl-ros
    ros-jazzy-image-transport
    ros-jazzy-image-proc
    ros-jazzy-depth-image-proc
    ros-jazzy-cv-bridge
    ros-jazzy-stereo-image-proc
    ros-jazzy-rviz2
)

# Skip package names the configured apt sources do not know about, so a single
# stale name cannot abort the whole transaction.
AVAILABLE=()
for pkg in "${PKGS[@]}"; do
    if apt-cache show "$pkg" >/dev/null 2>&1; then
        AVAILABLE+=("$pkg")
    else
        warn "skipping '$pkg' (not available in the configured apt sources)"
    fi
done

sudo apt-get update || die "apt-get update failed"
sudo apt-get install -y "${AVAILABLE[@]}" || die "dependency installation failed"

log "Checking Gazebo command line"
if ! command -v gz >/dev/null 2>&1; then
    warn "'gz' not on PATH yet; installing Gazebo Harmonic from the OSRF repository"
    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
        || die "failed to download the OSRF repository key"
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    sudo apt-get update || die "apt-get update failed after adding the OSRF repository"
    sudo apt-get install -y gz-harmonic || die "gz-harmonic installation failed"
fi

# The ROS vendor packages ship the CLI under /opt/ros/jazzy/opt/*/bin; make sure
# a fresh shell can find it even when the ament hook was not sourced yet.
if ! command -v gz >/dev/null 2>&1; then
    for candidate in /opt/ros/jazzy/opt/gz_sim_vendor/bin/gz /opt/ros/jazzy/bin/gz; do
        if [ -x "$candidate" ]; then
            export PATH="$(dirname "$candidate"):$PATH"
            break
        fi
    done
fi
command -v gz >/dev/null 2>&1 || die "'gz' still not found; Gazebo installation is incomplete"

log "Resolving remaining dependencies with rosdep"
sudo rosdep init >/dev/null 2>&1 || true
rosdep update || warn "rosdep update failed (offline?); continuing"
cd "$WS_ROOT" || die "cannot enter $WS_ROOT"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
    --skip-keys "ament_python" || warn "rosdep install reported an issue; continuing"

log "Building workspace"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+ \
    || die "colcon build failed"

printf '\n\033[1;32mSetup complete.\033[0m\n'
printf '  source %s/install/setup.bash\n' "$WS_ROOT"
printf '  ros2 launch rlai_bringup simulation.launch.py\n\n'

if [ "$RUN" = true ]; then
    log "Launching simulation"
    # shellcheck disable=SC1091
    . "$WS_ROOT/install/setup.bash"
    exec ros2 launch rlai_bringup simulation.launch.py "${EXTRA_LAUNCH_ARGS[@]}"
fi
