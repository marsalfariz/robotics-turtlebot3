#!/usr/bin/env bash
set -eo pipefail

source /etc/os-release
if [[ "$ID" != ubuntu || "${VERSION_CODENAME:-}" != focal ]]; then
    echo 'Use Ubuntu 20.04 (Focal) for ROS Noetic.' >&2
    exit 1
fi

project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$project/evidence"
log="$(mktemp "$project/evidence/noetic-setup-$(date +%Y%m%d-%H%M%S)-XXXX.log")"
echo "Setup log: $log"

install_source() (
    directory="$(mktemp -d)"
    trap 'rm -f -- "$directory/ros-apt-source.deb"; rmdir -- "$directory"' EXIT
    cd "$directory"
    curl -fsSL --retry 3 -o ros-apt-source.deb \
        https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros-apt-source_1.3.0.focal_all.deb
    echo '33a491f72e4e25491f8511c173526bb20ce84ff9faf596158f178f4900da5df1  ros-apt-source.deb' | sha256sum --check
    sudo dpkg -i ros-apt-source.deb
)

install_gazebo_source() (
    directory="$(mktemp -d)"
    trap 'rm -f -- "$directory/gazebo.gpg"; rmdir -- "$directory"' EXIT
    curl -fsSL --retry 3 -o "$directory/gazebo.gpg" https://packages.osrfoundation.org/gazebo.gpg
    sudo install -m 644 "$directory/gazebo.gpg" /usr/share/keyrings/ros-line-follower-gazebo.gpg
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-line-follower-gazebo.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable focal main\n' \
        "$(dpkg --print-architecture)" | sudo tee /etc/apt/sources.list.d/ros-line-follower-gazebo.list >/dev/null
)

install_packages() {
    local packages=(ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs gazebo11 xvfb xauth)
    if ! dpkg-query -W -f='${db:Status-Abbrev}\n' "${packages[@]}" 2>/dev/null | grep -c '^ii ' | grep -qx "${#packages[@]}"; then
        sudo -v
        sudo apt-get update
        sudo apt-get install -y --no-remove ca-certificates curl software-properties-common
        sudo add-apt-repository -y universe
        if ! apt-cache show ros-noetic-gazebo-ros-pkgs >/dev/null 2>&1 &&
           ! dpkg-query -W -f='${db:Status-Abbrev}' ros-apt-source 2>/dev/null | grep -q '^ii '; then
            install_source
        fi
        sudo apt-get update
        if ! apt-get -s install --no-remove "${packages[@]}" >/dev/null 2>&1; then
            install_gazebo_source
            sudo apt-get update
        fi
        sudo apt-get install -y --no-remove "${packages[@]}"
    fi
    source /opt/ros/noetic/setup.bash
    printf '\nUbuntu: %s\nROS: %s\n' "$PRETTY_NAME" "$ROS_DISTRO"
    command -v gzserver
    rospack find gazebo_ros
    dpkg-query -W "${packages[@]}"
}
install_packages 2>&1 | tee "$log"
