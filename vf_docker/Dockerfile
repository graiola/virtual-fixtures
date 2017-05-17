FROM ubuntu:14.04
MAINTAINER Gennaro Raiola <gennaro.raiola@gmail.com>

# Add source...
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install ROS Indigo
RUN apt-get update && apt-get install -y wget git unzip nano
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install --no-install-recommends -y ros-indigo-ros-base python-wstool

# Eigen 3.2.2
RUN wget -P /tmp/ http://ftp.fr.debian.org/debian/pool/main/e/eigen3/libeigen3-dev_3.2.2-3_all.deb
RUN dpkg -i /tmp/libeigen3-dev_3.2.2-3_all.deb

# Yaml
RUN apt-get install -y libyaml-cpp-dev

# Install g++ to avoid "CMAKE_CXX_COMPILER-NOTFOUND was not found." error
RUN apt-get update && apt-get install -y g++ cmake make

# Install ros_control and real time tools (Optional)
RUN apt-get install -y ros-indigo-ros-control ros-indigo-realtime-tools

# Install QT (Optional)
#RUN apt-get install -y qt5-default

#RUN source /opt/ros/indigo/setup.bash
RUN mkdir -p /home/catkin_ws/src
#RUN /opt/ros/indigo/bin/catkin_init_workspace /home/catkin_ws/src
RUN git clone --recursive https://github.com/graiola/virtual-fixtures /home/catkin_ws/src/virtual-fixtures

# Use ccache to speed up c++ compilation
#RUN apt-get update && apt-get install -y ccache
#ENV USE_CCACHE 1
#ENV CCACHE_DIR /opt/roscpp_output/ccache
#ENV PATH /usr/lib/ccache:$PATH
