FROM osrf/ros:melodic-desktop-bionic

ARG ssh_prv_key

# install ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'

# install further ROS dependencies
RUN apt-get update && apt-get install -y \
        python-rosdep  \
        python-rosinstall  \
        python-rosinstall-generator  \
        python-wstool  \
        build-essential  \
        python-rosdep \
        ros-melodic-catkin  \
        python-catkin-tools  \
        ros-melodic-rotors-simulator

# install other packages
RUN apt-get update && apt-get install -y \
        nano  \
        git \
        openssh-server

# Authorize SSH Host
RUN mkdir -p /root/.ssh && \
    chmod 0700 /root/.ssh && \
    ssh-keyscan gitlab.lrz.de > /root/.ssh/known_hosts

# Add the keys and set permissions
RUN echo "$ssh_prv_key" > /root/.ssh/id_rsa && \
    chmod 600 /root/.ssh/id_rsa

# clone your git using SSH connection
WORKDIR /home
RUN git clone git@gitlab.lrz.de:ge23ged/autonomous-systems-2021-group-terminus.git
RUN cd autonomous-systems-2021-group-terminus/project/src &&\
    git submodule init &&\
    git submodule update

# Remove SSH keys from docker
RUN rm -rf /root/.ssh/

# set working directory to project
WORKDIR /home/autonomous-systems-2021-group-terminus/project

RUN catkin config --extend /opt/ros/melodic
RUN catkin build
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash'
RUN /bin/bash -c 'source devel/setup.bash'
