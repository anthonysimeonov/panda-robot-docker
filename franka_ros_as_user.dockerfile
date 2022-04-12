FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04

ARG USER_NAME
ARG USER_ID
ARG USER_GID

RUN apt update
RUN apt install sudo

# create a non-root user
RUN echo "user id: ${USER_ID}"
RUN echo "user name: ${USER_NAME}"
RUN useradd -m --no-log-init --system --uid ${USER_ID} ${USER_NAME} -g sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# running all build commands as the user who initiated this docker build
USER ${USER_NAME}

# set uid and gid to match those outside the container
RUN sudo usermod -u $USER_ID $USER_NAME 

WORKDIR /home/$USER_NAME
ENV USER_HOME_DIR=/home/$USER_NAME

# Replacing shell with bash for later docker build commands
RUN sudo mv /bin/sh /bin/sh-old && sudo ln -s /bin/bash /bin/sh

ENV LC_ALL C.UTF-8
ENV LANG C.UTF-8

# install basic system stuff
COPY ./install_scripts/install_basic.sh /tmp/install_basic.sh
RUN sudo sh /tmp/install_basic.sh

# install ROS stuff
ENV ROS_DISTRO noetic

COPY ./install_scripts/install_ros.sh /tmp/install_ros.sh
RUN sudo sh /tmp/install_ros.sh

# bootstrap rosdep
RUN sudo rosdep init && sudo rosdep update

# install panda
RUN sudo apt update && sudo apt install -y cmake git libpoco-dev libeigen3-dev ros-noetic-catkin python3-catkin-tools 

ENV PANDA_WS=${USER_HOME_DIR}/panda_ws
RUN source /opt/ros/noetic/setup.bash 
RUN mkdir -p ${PANDA_WS}/src && \
    cd ${PANDA_WS} && catkin init && \
    catkin config --extend /opt/ros/$ROS_DISTRO --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False

WORKDIR ${PANDA_WS}/src
RUN git clone --recursive https://github.com/frankaemika/libfranka && \
    cd libfranka && \
    git checkout faa0219f286d857f01fa2097389ae6a23a4d9350 && \
    git submodule update 

RUN cd libfranka && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && cmake --build .

WORKDIR ${PANDA_WS}/src
RUN git clone -b noetic-devel --recursive https://github.com/frankaemika/franka_ros && cd franka_ros && \
    git checkout 902fdbba0f7c6036a84a688712a454b9e622863b

WORKDIR ${PANDA_WS}
RUN rosdep update && rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
RUN catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=${PANDA_WS}/src/libfranka/build

RUN sudo apt update && sudo apt install -y \
    ros-noetic-rosparam-shortcuts \
    ros-noetic-graph-msgs

WORKDIR ${PANDA_WS}/src
RUN git clone -b melodic https://github.com/rachelholladay/franka_ros_interface.git && \
    source ${PANDA_WS}/devel/setup.bash && \
    catkin config -a --cmake-args -DFranka_DIR:PATH=${PANDA_WS}/src/libfranka/build && \
    catkin build 

WORKDIR ${PANDA_WS}
# install moveit
RUN wstool init src && \
    wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall && \
    wstool update -t src && \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} && \
    source devel/setup.bash && catkin build

# make virtualenv
WORKDIR ${USER_HOME_DIR}
RUN mkdir environments && cd environments && virtualenv -p `which python3` --system-site-packages panda-ros

# install LCM
RUN sudo apt update && sudo apt install libglib2.0-dev freeglut3-dev -y
RUN git clone https://github.com/lcm-proj/lcm.git && cd lcm && git checkout c22669c
WORKDIR /home/${USER_NAME}/lcm
RUN mkdir build && \
    cd build && cmake .. && make && sudo make install
    WORKDIR /home/${USER_NAME}
    RUN cd lcm/lcm-python && source ${USER_HOME_DIR}/environments/panda-ros/bin/activate && python setup.py install 

# install realsense
RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg 
RUN sudo apt-get install -y ros-noetic-realsense2-camera ros-noetic-rgbd-launch

# last bits
RUN sudo apt install -y \  
    ros-noetic-joint-trajectory-controller \
    ros-noetic-rospy-message-converter \
    ros-noetic-kdl-conversions \
    ros-noetic-kdl-parser-py \
    ros-noetic-kdl-parser 

# we need numpy-quaternion to use the franka ros interface
RUN source ${USER_HOME_DIR}/environments/panda-ros/bin/activate && pip install numpy-quaternion

# change ownership of everything to our user
RUN echo 'Group id' && echo ${USER_GID}
RUN sudo groupadd $USER_NAME && sudo groupmod -g $USER_GID $USER_NAME
RUN cd ${USER_HOME_DIR} && echo $(pwd) && sudo chown $USER_NAME:$USER_NAME -R .

# create realtime group
RUN sudo addgroup realtime
RUN sudo usermod -a -G realtime ${USER_NAME}
USER root
RUN echo -e "@realtime soft rtprio 99\n@realtime soft priority 99\n@realtime soft memlock 102400\n@realtime hard rtprio 99\n@realtime hard priority 99\n@realtime hard memlock 102400" >> /etc/security/limits.conf
RUN echo -e "@realtime soft rtprio 99\n@realtime soft priority 99\n@realtime soft memlock 102400\n@realtime hard rtprio 99\n@realtime hard priority 99\n@realtime hard memlock 102400" >> /etc/security/limits.d/realtime.conf
USER $USER_NAME

RUN echo "source ${USER_HOME_DIR}/panda_ws/devel/setup.bash" >> ${USER_HOME_DIR}/.bashrc
RUN echo "source ${USER_HOME_DIR}/panda_ws/devel/setup.bash" >> ~/.bashrc
ENTRYPOINT bash -c "/bin/bash"
