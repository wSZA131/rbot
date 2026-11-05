#!/bin/bash
# scripts/run_native.sh
# One-command native launch: Gazebo simulation in the background + keyboard
# teleop in the foreground. Ctrl+C stops teleop and shuts the simulation down.
#
# Usage:
#   bash scripts/run_native.sh                # Gazebo GUI + keyboard teleop
#   bash scripts/run_native.sh --headless     # server only + keyboard teleop
#   bash scripts/run_native.sh world:=demo_warehouse_visual
#
# Any unrecognised token is forwarded to the simulation launch file, so the
# usual arguments (world:=..., x:=..., mapping_enabled:=true, ...) keep working.
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_FILE="${TMPDIR:-/tmp}/rbot_sim.log"
LAUNCH_PID=""

LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --headless) LAUNCH_ARGS+=("headless:=true") ;;
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

CLEANED_UP=false
cleanup() {
    if [ "$CLEANED_UP" = true ]; then
        return
    fi
    CLEANED_UP=true
    printf '\n==> Stopping simulation\n'
    if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
        kill -INT -"$LAUNCH_PID" 2>/dev/null || kill -INT "$LAUNCH_PID" 2>/dev/null
        sleep 2
        kill -TERM -"$LAUNCH_PID" 2>/dev/null || true
    fi
    # Launch sometimes leaves orphaned child nodes behind; clean them up.
    pkill -f "gz sim" 2>/dev/null
    pkill -f "ros_gz_bridge/parameter_bridge" 2>/dev/null
    pkill -f "robot_localization/ekf_node" 2>/dev/null
    pkill -f "imu_filter_madgwick" 2>/dev/null
    pkill -f "nav2_velocity_smoother" 2>/dev/null
    pkill -f "imu_filter_madgwick_node" 2>/dev/null
    exit 0
}
trap cleanup EXIT INT TERM HUP

echo "==> Starting simulation (log: $LOG_FILE)"
setsid ros2 launch rlai_bringup simulation.launch.py "${LAUNCH_ARGS[@]}" \
    > "$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

echo "==> Waiting for Gazebo and the ros2_control stack"
READY=false
for _ in $(seq 1 90); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        echo "Simulation launch exited early. Last log lines:" >&2
        tail -n 20 "$LOG_FILE" >&2
        exit 1
    fi
    # velocity_smoother reports this once the control lifecycle is active.
    if grep -q "Managed nodes are active" "$LOG_FILE" 2>/dev/null; then
        READY=true
        break
    fi
    sleep 1
done

if [ "$READY" = true ]; then
    echo "==> Simulation is up. Keyboard teleop ready."
else
    echo "==> Controllers not confirmed after 90 s; starting teleop anyway." >&2
    echo "    Check $LOG_FILE if the robot does not move." >&2
fi
echo "    Log: $LOG_FILE    (q/e: speed, space: stop, Ctrl+C: quit)"
echo ""

ros2 run rlai_teleop keyboard_teleop --ros-args -p use_sim_time:=true
