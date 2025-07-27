# ROS2 for Kassow Robots

## Overview

ROS2 for Kassow Robots allows you to monitor and control all KR models via the Robot Operating System 2 (ROS2). For using the ROS2 with real robot you need to install the ROS2 Interface CBun. See our wiki for detailed description. 

## Installation

ROS2 for Kassow Robots requires Linux setup with ROS2.  We recommend ROS2 Dashing with Ubuntu 18.04 and ROS2 Foxy with Ubuntu 20.04. This repository also supports Humble(Ubuntu 22.04) and Jazzy (Ubuntu 24.04).Refer to the [ROS2 setup guide](https://community.boschrexroth.com/ctrlx-automation-how-tos-qmglrz33/post/kassow-robots---command-from-ros2-urSGIhvvYqZxZAN)

```bash
cd ~/dew_ws
source /opt/ros/dashing/setup.bash
cd ~/dew_ws/src
git clone https://gitlab.com/kassowrobots/orange-ros2.git
cd ~/dew_ws
colcon build
. install/setup.bash
```

The second line may differ according to your ROS2 distribution and path, where you installed your ROS2. 

## Usage

Requires ROS2 Control from ROS2 Interface CBun running on Kassow Robot. Following examples also require "kr" as CBun Control Namespace.


### kr_msgs

#### System State

```bash
ros2 topic echo /kr/system/state
```
#### Jogging

```bash
ros2 topic pub -r 20 /kr/motion/jog_joint kr_msgs/msg/JogJoint "{jsvel: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# for foxy or older distros
ros2 topic pub -r 20 /kr/motion/jog_joint kr_msgs/msg/JogJoint "jsvel: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```
#### Services

```bash
ros2 service call /kr/iob/set_digital_output  kr_msgs/SetDiscreteOutput "index: 1
value: 1"
```

### kr_example
kr_example package contains two versions of tests with the same functionality. One for python and one for c++. 

#### state_subscriber
First of all state_subscriber reads robot state and writes all received values into console. After kr_example compilation as described before, you can run state_subscriber test by:

```bash
ros2 run kr_example_cpp robot_state
```


```bash
ros2 run kr_example_python robot_state
```

#### move_joint
By this test it is possible to let the robot move to the joint configuration which is defined in function move_joint() in file move_joint.cpp or move_joint.py. The configuration is [0., 35., 9., 116., 0., 0., 0.].

```bash
ros2 run kr_example_cpp move_joint
```

```bash
ros2 run kr_example_python move_joint
```

#### follow_joint
By this test it is possible to let the robot follow the joint configuration [0, 0, 0, 0, 0, 0, 0], which is defined in file follow_joint.cpp or follow_joint.py.

```bash
ros2 run kr_example_cpp follow_joint
```

```bash
ros2 run kr_example_python follow_joint
```

### select_jogging_frame and jog_linear
To test these two topics, you can run command bellow and your robot wil start jogging with values defined in jog_linear.cpp or jog_linear.py files in sellected jogging frame.

```bash
ros2 run kr_example_cpp jog_linear
```

```bash
ros2 run kr_example_python jog_linear
```

### self_motion
By running the following command, your robot will start executing self-motion. 

```bash
ros2 run kr_example_cpp self_motion
```


```bash
ros2 run kr_example_python self_motion
```

For more details see our wiki page with detailed describtion of all available messages and services. 
