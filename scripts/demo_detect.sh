#!/bin/bash
# scripts/demo_detect.sh
# Visual demo: Gazebo warehouse + open-vocabulary detector + depth-based map
# localisation. No map or Nav2 needed, so this is the fastest way to show the
# perception half of the search stack.
#
# What you see:
#   - Gazebo with the robot facing the backpack on the north rack
#   - /search/detections/image  : RGB frame with the detection box
#   - /search/target_location   : object position in the odom frame
#   - RViz (optional)           : target marker at the estimated position
#
# Usage:
#   bash scripts/demo_detect.sh                 # GUI + detector + mapper
#   bash scripts/demo_detect.sh --rviz          # also start RViz
#   bash scripts/demo_detect.sh --headless      # no Gazebo window
#   bash scripts/demo_detect.sh target:=bottle  # different prompt
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(dirname "$SCRIPT_DIR")"

MODEL_PATH="${MODEL_PATH:-$HOME/models/yoloe-11s-seg.pt}"
TARGET="backpack"
HEADLESS=false
SHOW_RVIZ=false
# ros2 launch takes name:=value tokens, not -x/-y flags.
GEOMETRY="x:=0.6 y:=1.55 z:=0.1 yaw:=1.5708"

for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=true ;;
        --rviz)     SHOW_RVIZ=true ;;
        target:=*)  TARGET="${arg#target:=}" ;;
        *)          echo "warning: ignoring unknown argument '$arg' (did you mean --rviz?)" >&2 ;;
    esac
done

if [ ! -f "$MODEL_PATH" ]; then
    echo "Model not found: $MODEL_PATH" >&2
    echo "Set MODEL_PATH=/path/to/yoloe-11s-seg.pt or download the weights first." >&2
    exit 1
fi

# shellcheck disable=SC1091
. /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
. "$WS_ROOT/install/setup.bash"

# The stack defaults to CycloneDDS; Fast DDS hits /dev/shm lock issues after a
# few restarts on this machine.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

PIDS=()
CLEANED_UP=false

cleanup() {
    if [ "$CLEANED_UP" = true ]; then
        return
    fi
    CLEANED_UP=true
    printf '\n==> Stopping demo\n'
    for pid in "${PIDS[@]}"; do
        kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL "-$pid" 2>/dev/null || true
    done
    pkill -f "gz sim" 2>/dev/null
    exit 0
}
trap cleanup EXIT INT TERM HUP

echo "==> Starting simulation (world=warehouse_search)"
if [ "$HEADLESS" = true ]; then
    setsid ros2 launch rlai_bringup simulation.launch.py world:=warehouse_search \
        headless:=true $GEOMETRY > /tmp/demo_sim.log 2>&1 &
else
    setsid ros2 launch rlai_bringup simulation.launch.py world:=warehouse_search \
        $GEOMETRY > /tmp/demo_sim.log 2>&1 &
fi
PIDS+=($!)

sleep 5
if tail -n 20 /tmp/demo_sim.log 2>/dev/null | grep -q "error:"; then
    echo "Simulation failed to start. Last log lines:" >&2
    tail -n 15 /tmp/demo_sim.log >&2
    exit 1
fi

echo "==> Starting detector ($TARGET)"
setsid ros2 run rlai_object_detection object_detector --ros-args \
    -p model_path:="$MODEL_PATH" -p use_sim_time:=true -p initial_label:="$TARGET" \
    > /tmp/demo_detector.log 2>&1 &
PIDS+=($!)

echo "==> Starting target mapper"
setsid ros2 run rlai_search target_mapper --ros-args \
    -p use_sim_time:=true -p map_frame:=odom > /tmp/demo_mapper.log 2>&1 &
PIDS+=($!)

if [ "$SHOW_RVIZ" = true ]; then
    sleep 2
    echo "==> Starting RViz (target marker + detection image are preconfigured)"
    setsid rviz2 -d "$WS_ROOT/install/rlai_search/share/rlai_search/rviz/detect_demo.rviz" \
        > /tmp/demo_rviz.log 2>&1 &
    PIDS+=($!)
fi

cat <<EOF

==> Demo running. Model warm-up takes ~10 s.

Watch detection:
  ros2 topic hz /search/detections
  python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rlai_interfaces.msg import ObjectLocation
rclpy.init()
node = Node('peek')
node.create_subscription(ObjectLocation, '/search/target_location',
                         lambda m: print(m.label, round(m.map_pose.pose.position.x, 2),
                                         round(m.map_pose.pose.position.y, 2),
                                         round(m.confidence, 2)), 10)
for _ in range(150):
    rclpy.spin_once(node, timeout_sec=0.2)
rclpy.shutdown()
PY

Logs: /tmp/demo_sim.log /tmp/demo_detector.log /tmp/demo_mapper.log
Press Ctrl+C to stop everything.
EOF

wait
