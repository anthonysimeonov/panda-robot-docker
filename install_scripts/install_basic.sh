#!/bin/bash

set -euxo pipefail

apt update

DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
build-essential \
make \
cmake \
curl \
git \
wget \
gfortran \
software-properties-common \
net-tools \
ffmpeg \
unzip \
vim \
xserver-xorg-dev \
tmux \
terminator \
libboost-all-dev \
ca-certificates \
libjpeg-dev \
libprotobuf-dev \
libqt5multimedia5 \
libqt5x11extras5 \
libtbb2 \
libtheora0 \
libpng-dev \
qtbase5-dev \
zlib1g \
dirmngr \
gnupg2 \
lsb-release \
bc \
libssl-dev \
libelf-dev \
bison \
flex \
keyboard-configuration

apt clean

rm -rf /var/lib/apt/lists/*
