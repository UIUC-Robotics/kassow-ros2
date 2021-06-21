
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


class KrRobotROS2FollowJointPublisher : public rclcpp::Node {
    
public:
    
    KrRobotROS2FollowJointPublisher() : Node("kr_robot_ros2_subscriber") {
        // Publisher init
        follow_joint_publisher_ = this->create_publisher<kr_msgs::msg::FollowJoint>("/kr/motion/follow_joint", 10);

        // Follow joint message example
        follow_joint_timer_ = this->create_wall_timer(5000ms, std::bind(&KrRobotROS2FollowJointPublisher::follow_joint_callback, this));
    }
  

private:

    // Each time callback is called, one configuration is selected and request to follow this configuration is published
    // configurations are changing each call.
    void follow_joint_callback(){
        std::lock_guard<std::mutex> guard(my_mutex);
        auto message = kr_msgs::msg::FollowJoint();
        const std::array<double, 7> config_1 = {0., 35., 9., 116., 0., 0., 0.};
        const std::array<double, 7> config_2 = {54., 35., 9., 116., 0., 0., 0.};

        if (temp) { message.set__jsconf(config_1); } else { message.set__jsconf(config_2); }

        message.set__ttype(1);
        message.set__tvalue(3);
        message.set__bpoint(0);
        message.set__btype(1);
        message.set__bvalue(2);

        std ::cout << "PUBLISHING FOLLOW JOINT \n";
        follow_joint_publisher_->publish(message);
        temp = !temp;
    }

    rclcpp::Publisher<kr_msgs::msg::FollowJoint>::SharedPtr follow_joint_publisher_;

    
    rclcpp::TimerBase::SharedPtr follow_joint_timer_;
    
    bool temp = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrRobotROS2FollowJointPublisher>());
  rclcpp::shutdown();
  return 0;
}
