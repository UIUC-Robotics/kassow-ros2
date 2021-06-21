
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


class KrRobotROS2MoveJoint : public rclcpp::Node {
    
public:
    
    KrRobotROS2MoveJoint() : Node("kr_robot_ros2_subscriber") {
        // Move joint service example
        move_joint();
    }
  

private:

    // Move joint service example sending request to move to specified joint configuration
    void move_joint()
    {
        std::lock_guard<std::mutex> guard(my_mutex);
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_joint_node");
        rclcpp::Client<kr_msgs::srv::MoveJoint>::SharedPtr client = node->create_client<kr_msgs::srv::MoveJoint>("/kr/motion/move_joint");

        auto request = std::make_shared<kr_msgs::srv::MoveJoint::Request>();

        const std::array<double, 7> config_1 = {0., 35., 9., 116., 0., 0., 0.};

        request->set__jsconf(config_1);

        request->set__ttype(1);
        request->set__tvalue(5);
        request->set__bpoint(0);
        request->set__btype(1);
        request->set__bvalue(2);
        request->set__sync(0);
        request->set__chaining(1);

        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        std ::cout << "SENDING REQUEST FOR JOINT MOVE \n";

        auto result = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfull move joint request.");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Move joint failed.");
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "/kr/motion/move_joint");
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrRobotROS2MoveJoint>());
  rclcpp::shutdown();
  return 0;
}
