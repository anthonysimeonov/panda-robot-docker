#!/bin/bash

set -euxo pipefail

echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

apt update 

apt install -y ros-noetic-desktop-full

apt update && apt install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool build-essential

apt update && apt install -y \
    python3-dev \
    python3-virtualenv

rm -rf /var/lib/apt/lists/*
