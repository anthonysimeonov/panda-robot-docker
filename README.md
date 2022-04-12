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
```
sh run_panda_robot_image.sh
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
