#!/bin/bash
# scripts/map_warehouse_search.sh
# Builds the map used by the search demo:
#   simulation + SLAM Toolbox in the background, keyboard teleop in the
#   foreground. Ctrl+C saves the map to maps/warehouse_search.{pgm,yaml} and
#   then shuts the simulation down.
#
# Usage:
#   bash scripts/map_warehouse_search.sh              # headless, fastest
#   bash scripts/map_warehouse_search.sh --gui        # Gazebo 3-D window
#   bash scripts/map_warehouse_search.sh --rviz       # mapping RViz
#   bash scripts/map_warehouse_search.sh --gui --rviz # both (slowest)
#
# Performance notes: on WSL the Gazebo GUI and RViz both compete with the
# physics server for CPU. The depth camera is disabled here because mapping
# only needs the LiDAR.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_FILE="${TMPDIR:-/tmp}/rbot_mapping.log"
MAP_PREFIX="$WS_ROOT/maps/warehouse_search"
LAUNCH_PID=""
CLEANED_UP=false

HEADLESS=true
SHOW_RVIZ=false
LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --gui)      HEADLESS=false ;;
        --headless) HEADLESS=true ;;
        --rviz)     SHOW_RVIZ=true ;;
        *)          LAUNCH_ARGS+=("$arg") ;;
    esac
done

if [ ! -f "$WS_ROOT/install/setup.bash" ]; then
    echo "Workspace is not built. Run: bash scripts/setup_native.sh" >&2
    exit 1
fi

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. "$WS_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

mkdir -p "$WS_ROOT/maps"

cleanup() {
    if [ "$CLEANED_UP" = true ]; then
        return
    fi
    CLEANED_UP=true
    printf '\n==> Saving map to %s\n' "$MAP_PREFIX"
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        timeout 25 ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" \
            --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 \
            || echo "map_saver_cli failed; the map was not saved" >&2

        printf '\n==> Stopping simulation\n'
        kill -INT -"$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
        sleep 2
        kill -TERM -"$LAUNCH_PID" 2>/dev/null || true
    fi
    pkill -f "gz sim" 2>/dev/null
    pkill -f "ros_gz_bridge/parameter_bridge" 2>/dev/null
    pkill -f "robot_localization/ekf_node" 2>/dev/null
    pkill -f "slam_toolbox" 2>/dev/null
    exit 0
}
trap cleanup EXIT INT TERM HUP

LAUNCH_ARGS_ALL=(
    "world:=warehouse_search"
    "headless:=$HEADLESS"
    "mapping_enabled:=true"
    "depth_camera_enabled:=false"
)
if [ "$SHOW_RVIZ" = true ]; then
    LAUNCH_ARGS_ALL+=("slam_rviz_enabled:=true")
fi

echo "==> Starting mapping session (log: $LOG_FILE)"
echo "    Gazebo GUI: $([ "$HEADLESS" = true ] && echo off || echo on), RViz: $([ "$SHOW_RVIZ" = true ] && echo on || echo off)"
setsid ros2 launch rlai_bringup simulation.launch.py \
    "${LAUNCH_ARGS_ALL[@]}" "${LAUNCH_ARGS[@]}" > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

echo "==> Waiting for SLAM Toolbox"
READY=false
for _ in $(seq 1 90); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "Simulation launch exited early. Last log lines:" >&2
        tail -n 20 "$LOG_FILE" >&2
        exit 1
    fi
    if grep -q "Managed nodes are active" "$LOG_FILE" 2>/dev/null; then
        READY=true
        break
    fi
    sleep 1
done

if [ "$READY" = true ]; then
    echo "==> SLAM is up. Drive around every aisle, then press Ctrl+C to save."
else
    echo "==> SLAM not confirmed after 90 s; starting teleop anyway." >&2
fi
echo "    w/a/s/d drive, q/e speed, space stop, Ctrl+C saves the map and exits."
echo ""

ros2 run rlai_teleop keyboard_teleop --ros-args -p use_sim_time:=true
