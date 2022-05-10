#!/bin/bash
set -ef -o pipefail
if [ -z $1 ]
then
    RUN_MODE="gui"
else
    RUN_MODE=$1
fi

set -euf -o pipefail

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
IMAGE=franka-ros-ubuntu-20-user:v0


if [ $RUN_MODE = "remote" ]
then
    echo "Running without settings for GUI"
    docker run --rm -it \
        --shm-size=8gb \
        --volume="$NDF_SOURCE_DIR/../..:/home/${USER}/ndf_robot_private" \
        --net=host \
        --privileged \
        --cap-add=ALL \
        --env="FRANKA_ROBOT_IP=$FRANKA_ROBOT_IP" \
        --env="ROS_IP=$ROS_IP" \
        --name=panda_robot_container-user $IMAGE
else
    echo "Running with settings for GUI"

    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

    docker run --rm -it \
        --shm-size=8gb \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH:rw" \
        --volume="$XSOCK:$XSOCK:rw" \
        --volume="$NDF_SOURCE_DIR/../..:/home/${USER}/ndf_robot_private" \
        --net=host \
        --privileged \
        --cap-add=ALL \
        --env="FRANKA_ROBOT_IP=$FRANKA_ROBOT_IP" \
        --env="ROS_IP=$ROS_IP" \
        --name=panda_robot_container-user $IMAGE
fi

