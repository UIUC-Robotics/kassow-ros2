# ROS2 for Kassow Robots

## Overview

ROS2 for Kassow Robots allows you to monitor and control all KR models via the Robot Operating System 2 (ROS2). For using the ROS2 with real robot you need to install the ROS2 Interface CBun. See our wiki for detailed description. 

## Installation

ROS2 for Kassow Robots requires Linux setup with ROS2.  We recommend ROS2 Dashing with Ubuntu 18.04 and ROS2 Foxy with Ubuntu 20.04. Newer versions of ROS2 (Galactic) are unfortunately not supported. It is not oficially supported with any other ROS2 version or operating system as it was not tested in any other setup.  

```bash
cd ~/dew_ws/src
git clone https://gitlab.com/kassowrobots/orange-ros2.git
cd ~/dew_ws
colcon build --packages-select kr_msgs
catkin_make --pkg=kr_example_python install
```

## Usage

Requires ROS Control from ROS Interface CBun running on Kassow Robot. Following examples also require "kr" as CBun Control Namespace.

### kr_msgs

#### System State

```bash
rostopic echo /kr/system/state
```
#### Jogging

```bash
rostopic pub -r 20 /kr/motion/jog_joint kr_msgs/JogJoint "jsvel: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 
```
#### Services

```bash
rosservice call /kr/iob/set_digital_output 1 1
```

### kr_example

#### state_subscriber

```bash
rosrun kr_example_python state_subscriber.py
```

#### iob_control

```bash
rosrun kr_example_python iob_control.py
```

#### ps4_jogging

```bash
rosrun kr_example_python ps4_jogging.py
```
