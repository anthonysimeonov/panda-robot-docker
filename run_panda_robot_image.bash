#!/bin/bash
set -euf -o pipefail

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run --rm -it \
    --shm-size=8gb \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH:rw" \
    --volume="$XSOCK:$XSOCK:rw" \
    --net=host \
    --privileged \
    --cap-add=ALL \
    --env="FRANKA_ROBOT_IP=$FRANKA_ROBOT_IP" \
    --env="ROS_IP=$ROS_IP" \
    --name=panda_robot_container-user franka-ros-ubuntu-20-user:v0

