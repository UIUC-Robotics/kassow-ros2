
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


class KrRobotROS2JogJoint : public rclcpp::Node {
    
public:
    
    KrRobotROS2JogJoint() : Node("kr_robot_ros2_subscriber") {
        // Publisher init
        jog_linear_publisher_ = this->create_publisher<kr_msgs::msg::JogLinear>("/kr/motion/jog_linear", 10);

        // Select jogging frame service and jog linear mnessage example
        select_jogging_frame(1);
        jog_linear_timer_ = this->create_wall_timer(20ms, std::bind(&KrRobotROS2JogJoint::jog_linear_callback, this));

    }
  

private:

    // Sending message to jog X axis
    void jog_linear_callback(){
        std::lock_guard<std::mutex> guard(my_mutex);
        auto message = kr_msgs::msg::JogLinear();
        const std::array<double, 3> vel = { 100, 0., 0. };
        const std::array<double, 3> rot = { 0., 0., 0. };

        message.set__vel(vel);
        message.set__rot(rot);

        std ::cout << "PUBLISHING JOG LINEAR \n";
        jog_linear_publisher_->publish(message);
    }

    // Sending request to select jogging frame and waiting until result received
    void select_jogging_frame(int value)
    {
        std::lock_guard<std::mutex> guard(my_mutex);
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("select_jogging_node");
        rclcpp::Client<kr_msgs::srv::SelectJoggingFrame>::SharedPtr client = node->create_client<kr_msgs::srv::SelectJoggingFrame>("/kr/motion/select_jogging_frame");

        auto request = std::make_shared<kr_msgs::srv::SelectJoggingFrame::Request>();
        request->set__ref(value);

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        std ::cout << "SENDING REQUEST TO SELECT JOGGING FRAME \n";
        auto result = client->async_send_request(request);
        #if defined(FOXY) || defined(DASHING)
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfull selection.");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Selection failed.");
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "/kr/motion/select_jogging_frame");
        }
        # else
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfull selection.");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Selection failed.");
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "/kr/motion/select_jogging_frame");
        }
        #endif
    }


    rclcpp::Publisher<kr_msgs::msg::JogLinear>::SharedPtr jog_linear_publisher_;
    
    rclcpp::TimerBase::SharedPtr jog_linear_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrRobotROS2JogJoint>());
  rclcpp::shutdown();
  return 0;
}
