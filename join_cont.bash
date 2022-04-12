#!/bin/bash
set -ef -o pipefail

CONTAINER_NAME="panda_robot_container-user"
if [ -z $1 ]
then 
    printf "Joining container as ${USER}\n\n"
    docker exec -it $CONTAINER_NAME bash
elif [ "$1" = "root" ]
then 
    printf "Joining container as $1\n\n"
    docker exec --env USER="root" --user root -it $CONTAINER_NAME bash
fi
