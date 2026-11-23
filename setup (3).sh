#!/usr/bin/env bash
set -eo pipefail

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${LINE_FOLLOWER_BUILD_DIR:-$HOME/.local/share/ros-line-follower}"
export LANG=C.UTF-8 LC_ALL=C.UTF-8

if [[ ! -r /etc/os-release ]]; then
    echo "Could not identify the Linux distribution." >&2
    exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "${VERSION_CODENAME:-}" != jammy ]]; then
    echo "ROS 2 Humble requires Ubuntu 22.04 (Jammy) for this setup." >&2
    exit 1
fi

mkdir -p "$workspace" "$project/evidence"
log="$(mktemp "$project/evidence/setup-$(date +%Y%m%d-%H%M%S)-XXXX.log")"
echo "Setup log: $log"

install_ros_source() (
    local_release="$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
    asset="$(mktemp -d)"
    trap 'rm -f -- "$asset/ros2-apt-source.deb"; rmdir -- "$asset"' EXIT
    curl -fsSL --retry 3 -o "$asset/ros2-apt-source.deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$local_release/ros2-apt-source_${local_release}.jammy_all.deb"
    sudo dpkg -i "$asset/ros2-apt-source.deb"
)

install_and_build() {
    local packages=(
        ros-humble-ros-base ros-humble-ros-gz
        ros-humble-sensor-msgs ros-humble-nav-msgs
        ros-humble-geometry-msgs ros-humble-std-msgs
        python3-colcon-common-extensions
    )

    if ! dpkg-query -W -f='${db:Status-Abbrev}\n' "${packages[@]}" 2>/dev/null | grep -c '^ii ' | grep -qx "${#packages[@]}"; then
        sudo -v
        sudo apt-get update
        sudo apt-get install -y --no-remove ca-certificates curl software-properties-common
        sudo add-apt-repository -y universe
        if ! apt-cache show ros-humble-ros-gz >/dev/null 2>&1 &&
           ! dpkg-query -W -f='${db:Status-Abbrev}' ros2-apt-source 2>/dev/null | grep -q '^ii '; then
            install_ros_source
        fi
        sudo apt-get update
        sudo apt-get install -y --no-remove "${packages[@]}"
    fi

    source /opt/ros/humble/setup.bash
    cd "$workspace"
    colcon --log-base "$workspace/log" build \
        --base-paths "$project/ros_ws/src" \
        --build-base "$workspace/build" --install-base "$workspace/install"
    source "$workspace/install/local_setup.bash"

    printf '\nUbuntu: %s\nROS: %s\n' "$PRETTY_NAME" "$ROS_DISTRO"
    ign gazebo --versions
    ros2 pkg executables line_follower
    dpkg-query -W "${packages[@]}"
}

install_and_build 2>&1 | tee "$log"
