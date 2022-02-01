FROM ubuntu:18.04
SHELL [ "/bin/bash", "--login", "-c" ]

# install essential tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    curl \
    ca-certificates \
    libjpeg-dev \
    libpng-dev && \
    rm -rf /var/lib/apt/lists/*

# install ros
RUN apt-get update && apt-get install ros-melodic

# install catkin dependencies
RUN apt-get update && apt-get install python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev

# install catkin
RUN apt-get update && apt-get install ros-melodic-catkin

# create catkin workspace
RUN mkdir -p /home/catkin_ws/src
RUN cd ~/catkin_ws/
WORKDIR /home/catkin_ws
RUN catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# install packages/dependencies in catkin workspace
RUN apt-get update && apt-get install -y python3 ros-melodic-rtabmap ros-melodic-rtabmap-ros ros-$ROS_DISTRO-realsense2-camera rviz

RUN chmod -R a+w .
