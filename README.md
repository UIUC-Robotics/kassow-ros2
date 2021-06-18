# ROS2 for Kassow Robots

## Overview

ROS2 for Kassow Robots allows you to monitor and control all KR models via the Robot Operating System 2 (ROS2). For using the ROS2 with real robot you need to install the ROS2 Interface CBun. See our wiki for detailed description. 

## Installation

ROS2 for Kassow Robots requires Linux setup with ROS2.  We recommend ROS2 Dashing with Ubuntu 18.04 and ROS2 Foxy with Ubuntu 20.04. Newer versions of ROS2 (Galactic) are unfortunately not supported. It is not oficially supported with any other ROS2 version or operating system as it was not tested in any other setup.  

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
ros2 topic pub -r 20 /kr/motion/jog_joint kr_msgs/msg/JogJoint "jsvel: [30.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```
#### Services

```bash
ros2 service call /kr/iob/set_digital_output  kr_msgs/SetDiscreteOutput "index: 1
value: 1"
```

### kr_example
To try the following examples, you will need to edit file kr_example.cpp located in kr_example/src folder. In the upper part of the file in constructor of class KrRobotROS2Subscriber, there are several testing scenarios prepared. 
#### state_subscriber
First of all state_subscriber which can be tested by uncommenting (removing //) line 50, saving the file, compiling the whole project and running the example. 

```bash
cd ~/dew_ws
colcon build
ros2 run kr_example kr_example
```

#### move_joint
By uncommening line 53, it is possible to let the robot move to the joint configuration which is defined in function move_joint().

```bash
cd ~/dew_ws
colcon build
ros2 run kr_example kr_example
```

#### follow_joint
By uncomening line 56 and running the example your robot will move between two joint configurations in 5s time intervals. 

```bash
cd ~/dew_ws
colcon build
ros2 run kr_example kr_example
```

### select_jogging_frame and jog_linear
To test these two topics, just uncomment lines 59 and 60 and your robot wil start jogging in sellecteg jogging frame.


```bash
cd ~/dew_ws
colcon build
ros2 run kr_example kr_example
```

### self_motion
By uncommenting line 63 and running the testing program, your robot will start executing selfmotion. 

```bash
cd ~/dew_ws
colcon build
ros2 run kr_example kr_example
```

For more details see our wiki page with detailed describtion of all available messages and services. 
