
// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
using namespace std::chrono_literals;

#include <string>

#include "rclcpp/rclcpp.hpp"

#include <kr_msgs/msg/system_state.hpp>
#include <kr_msgs/msg/number.hpp>
#include <kr_msgs/msg/robot_pose.hpp>

#include <kr_msgs/srv/move_joint.hpp>

#include <kr_msgs/srv/select_jogging_frame.hpp>

#include <kr_msgs/msg/jog_linear.hpp>
#include <kr_msgs/msg/self_motion.hpp>
#include <kr_msgs/msg/follow_joint.hpp>
#include <mutex>


using std::placeholders::_1;

std::mutex my_mutex;


class KrRobotROS2SelfMotion : public rclcpp::Node {
    
public:
    
    KrRobotROS2SelfMotion() : Node("kr_robot_ros2_subscriber") {
        // Publisher init
        self_motion_publisher_ = this->create_publisher<kr_msgs::msg::SelfMotion>("/kr/motion/self_motion", 10);
        
        // Self motion message example
        self_motion_timer_ = this->create_wall_timer(20ms, std::bind(&KrRobotROS2SelfMotion::self_motion_callback, this));
    }
  

private:

    // Sending message to do self-motion in positive direction with relative speed 0.5
    void self_motion_callback() {
        std::lock_guard<std::mutex> guard(my_mutex);
        auto message = kr_msgs::msg::SelfMotion();

        message.set__speed(0.5);

        std ::cout << "PUBLISHING SELF MOTION \n";
        self_motion_publisher_->publish(message);
    }

    rclcpp::Publisher<kr_msgs::msg::SelfMotion>::SharedPtr self_motion_publisher_;
    
    rclcpp::TimerBase::SharedPtr self_motion_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrRobotROS2SelfMotion>());
  rclcpp::shutdown();
  return 0;
}
