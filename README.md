Virtual-Fixtures
==============

A set of ROS packages to create and interact with a library of virtual guides.

------
## What ?

Virtual guides constrain the robot movement along task-relevant trajectories.
To do an analogy with the real world, a virtual guide can be compared to a ruler which can be used to draw straight lines with minimal effort.
These packages allow you to create a library of guides (i.e. multiple guides working in parallel) and to choose the guide to use through an intuitive haptic interaction. [video](https://www.youtube.com/watch?v=K8xCxh6U_yg)

![Multiple Guides](http://perso.ensta-paristech.fr/~raiola/img/virtual_guides_simple.png)

------
## Why ?

Virtual guides can be used in a co-manipulation context where humans and robots work together to complete different kind of tasks. As well known, industrial robots have a good accuracy and strength but they lack the capacity of abstract thinking and they don't (usually) posses a good perception.
By using the virtual guides it is possible to facilitate the joint completion of a task by combining the advantages of an industrial robot with the smartness of a human worker. 

------
## Prerequisites:
The code has been tested with ROS Indigo and Ubuntu 14.04.

### Necessary:

##### ROS Indigo
```bash
sudo apt-get update && sudo apt-get install -y wget git unzip nano
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install --no-install-recommends -y ros-indigo-ros-base python-wstool
```

##### Eigen 3.2.2
```bash
wget -P /tmp/ http://ftp.fr.debian.org/debian/pool/main/e/eigen3/libeigen3-dev_3.2.2-3_all.deb && sudo dpkg -i /tmp/libeigen3-dev_3.2.2-3_all.deb
```
##### Yaml
```bash
sudo apt-get install -y libyaml-cpp-dev
```
### Optional:
##### ros-control (Needed by vf_controller)
```bash
sudo apt-get install -y ros-indigo-ros-control
```
##### realtime_tools (Needed to have the real time publishers)
```bash
sudo apt-get install -y ros-indigo-realtime-tools 
```
##### Qt5 (Needed by vf_gui)
```bash
sudo apt-get install -y qt5-default 
```
##### RTAI for real time tests
Here is a good reference to install
[RTAI](https://www.rtai.org/userfiles/downloads/RTAICONTRIB/RTAI_Installation_Guide.pdf)

------
## Installation:
In your catkin/src:
```bash
sudo git clone --recursive https://github.com/graiola/virtual-fixtures
```
Compile and install
```bash
catkin_make_isolated --install
```
If you want to launch the tests
```bash
catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Debug --make-args run_tests
```
And that's it! :D

## References:
[1]  Gennaro Raiola, Xavier Lamy, and Freek Stulp. 
     Co-manipulation with Multiple Probabilistic Virtual Guides. In International Conference on       Intelligent Robots and Systems (IROS), 2015. [pdf](https://hal-cea.archives-ouvertes.fr/hal-01170974/document)

[2]   Gennaro Raiola, Pedro Rodriguez-Ayerbe, Xavier Lamy, Sami Tliba, and Freek Stulp. Parallel Guiding Virtual Fixtures: Control and Stability. In IEEE Multi-Conference on Systems and Control (MSC), 2015. [pdf](https://hal-cea.archives-ouvertes.fr/hal-01250101/document)
