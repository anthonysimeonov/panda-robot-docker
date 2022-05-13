# panda-robot-docker

### Build
```
python docker_build.py \
    --file $(/path/to/dockerfile.dockerfile) \ # this is required to be explicitly specified
    --dry_run/-d \ # if you just want to print the resulting build command and not execute it 
    --image $(image_name) \ # name to give this image. python build script has a default
    --no_cache # if you want to restart the build from scratch and not use any of the cached build layers
```

### Run
First you have to export the IP address of the Franka Robot. You should also export the IP address of the host machine for whatever local network ROS will be run on (i.e., if you have another locally connected computer sending commands to the robot). Then run the `docker run` command in the bash script.
```
export FRANKA_ROBOT_IP=$FRANKA_ROBOT_IP
export ROS_IP=$ROS_IP
bash run_panda_robot_image_gui_options.bash
```

After running the container, ensure you can use GUI-based applications from the container. From outside the running container (i.e., in a new terminal):
```
./visualize_access.bash
```

### Make sure GUI works
Try running `rqt` from inside the container:
```
rosrun rqt rqt
```

Try running `rviz` from inside the container:
```
# in one terminal inside the container (i.e., in a tmux window)
roscore

# in another termainl inside the container (i.e., in a new tmux window/pane)
rviz
```

### Launch panda interface
First, join the container as root
```
# from the host machine
cd /path/to/panda-robot-docker
bash join_cont.bash root
```
Then, launch the driver node (see [here](https://github.com/rachelholladay/franka_ros_interface) for more info)
```
roslaunch franka_interface interface.launch
```
