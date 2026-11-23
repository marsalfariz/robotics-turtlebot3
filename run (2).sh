#!/usr/bin/env bash
set -eo pipefail

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${LINE_FOLLOWER_BUILD_DIR:-$HOME/.local/share/ros-line-follower}"
mode="${1:-demo}"
display_mode="${2:-auto}"
if [[ "$mode" != demo && "$mode" != verify ]] ||
   [[ "$display_mode" != auto && "$display_mode" != gui && "$display_mode" != headless ]] ||
   (( $# > 2 )); then
    echo "Usage: bash run.sh [demo|verify] [auto|gui|headless]" >&2
    exit 2
fi
if [[ ! -f "$workspace/install/local_setup.bash" ]]; then
    echo "Run bash setup.sh first." >&2
    exit 1
fi
if [[ "$display_mode" == auto ]]; then
    display_mode=headless
    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
        display_mode=gui
    fi
fi

source /opt/ros/humble/setup.bash
source "$workspace/install/local_setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
export IGN_PARTITION="ros_line_follower_${UID}_$$"

# Prevent two runs from publishing on the same ROS topic names.
exec 9>"$workspace/run.lock"
if ! flock -n 9; then
    echo "A line-follower run is already active. Stop it before starting another." >&2
    exit 1
fi

mkdir -p "$project/evidence"
run_dir="$(mktemp -d "$project/evidence/$mode-$(date +%Y%m%d-%H%M%S)-XXXX")"
# A local Linux path avoids simulator quirks with spaces and Windows mounts.
world_dir="$(mktemp -d)"
trap 'rm -f -- "$world_dir/track.sdf"; rmdir -- "$world_dir"' EXIT
ros2 run line_follower make_world "$world_dir/track.sdf"
cp -- "$world_dir/track.sdf" "$run_dir/track.sdf"

printf 'Mode: %s (%s)\nROS_DOMAIN_ID: %s\nEvidence: %s\n' "$mode" "$display_mode" "$ROS_DOMAIN_ID" "$run_dir"
set +e
ros2 launch line_follower simulation.launch.py \
    world_file:="$world_dir/track.sdf" \
    headless:="$([[ "$display_mode" == headless ]] && echo true || echo false)" \
    verify:="$([[ "$mode" == verify ]] && echo true || echo false)" \
    evidence_dir:="$run_dir" 2>&1 | tee "$run_dir/launch.log"
status=${PIPESTATUS[0]}
set -e
if [[ "$mode" == verify ]]; then
    if [[ ! -s "$run_dir/result.json" ]]; then
        echo "Verification ended before a result was saved. See $run_dir/launch.log" >&2
        exit 1
    fi
    cat "$run_dir/result.json"
    python3 -c 'import json,sys; sys.exit(json.load(open(sys.argv[1]))["status"] != "passed")' "$run_dir/result.json"
fi
exit "$status"
