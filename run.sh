#!/usr/bin/env bash
set -eo pipefail

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-verify}"
display_mode="${2:-auto}"
if [[ "$mode" != demo && "$mode" != verify ]] ||
   [[ "$display_mode" != auto && "$display_mode" != gui && "$display_mode" != headless ]] ||
   (( $# > 2 )); then
    echo 'Usage: bash noetic/run.sh [demo|verify] [auto|gui|headless]' >&2
    exit 2
fi
if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
    echo 'Run bash noetic/setup.sh in Ubuntu 20.04 first.' >&2
    exit 1
fi
if [[ "$display_mode" == auto ]]; then
    display_mode=headless
    if [[ -n "${DISPLAY:-}" ]]; then
        display_mode=gui
    fi
fi

source /opt/ros/noetic/setup.bash
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GAZEBO_MODEL_DATABASE_URI=''
export ROS_MASTER_URI="${NOETIC_ROS_MASTER_URI:-http://localhost:11474}"
export ROS_HOSTNAME=localhost
unset ROS_IP
export GAZEBO_MASTER_URI="${NOETIC_GAZEBO_MASTER_URI:-http://localhost:11475}"

state="$HOME/.local/share/ros-line-follower-noetic"
mkdir -p "$state" "$project/evidence"
exec 9>"$state/run.lock"
if ! flock -n 9; then
    echo 'A Noetic line-follower run is already active.' >&2
    exit 1
fi
run_dir="$(mktemp -d "$project/evidence/noetic-$mode-$(date +%Y%m%d-%H%M%S)-XXXX")"
stage="$(mktemp -d "$state/run-XXXX")"
cleanup() {
    local package="$stage/line_follower_noetic"
    rm -f -- "$package/package.xml" "$package/node.py" "$package/track.world" "$package/simulation.launch"
    rmdir -- "$package" "$stage"
}
trap cleanup EXIT
package="$stage/line_follower_noetic"
mkdir "$package"
install -m 644 "$project/noetic/package.xml" "$project/noetic/simulation.launch" "$package/"
install -m 755 "$project/noetic/node.py" "$package/"
export ROS_PACKAGE_PATH="$stage${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export PYTHONPATH="$project/ros_ws/src/line_follower${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 "$project/noetic/make_world.py" \
    "$project/ros_ws/src/line_follower/worlds/track.sdf.in" "$package/track.world"
cp -- "$package/track.world" "$run_dir/track.world"

gui=true
launcher=(roslaunch line_follower_noetic simulation.launch)
if [[ "$display_mode" == headless ]]; then
    gui=false
    launcher=(xvfb-run -a -s '-screen 0 1024x768x24' "${launcher[@]}")
fi
verify=false
if [[ "$mode" == verify ]]; then
    verify=true
fi
printf 'Mode: Noetic %s (%s)\nEvidence: %s\n' "$mode" "$display_mode" "$run_dir"
set +e
"${launcher[@]}" gui:="$gui" verify:="$verify" world_file:="$package/track.world" \
    evidence_dir:="$run_dir" 2>&1 | tee "$run_dir/launch.log"
status=${PIPESTATUS[0]}
set -e
if [[ "$mode" == verify ]]; then
    if [[ ! -s "$run_dir/result.json" ]]; then
        echo "Verification stopped before saving a result. See $run_dir/launch.log" >&2
        exit 1
    fi
    cat "$run_dir/result.json"
    python3 -c 'import json,sys; sys.exit(json.load(open(sys.argv[1]))["status"] != "passed")' "$run_dir/result.json"
fi
exit "$status"
