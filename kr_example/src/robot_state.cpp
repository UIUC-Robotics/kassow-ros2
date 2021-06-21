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


class KrRobotROS2StateSubscriber : public rclcpp::Node {
    
public:
    
    KrRobotROS2StateSubscriber() : Node("kr_robot_ros2_subscriber") {
        // Robot state subscriber example
        robot_state_subscription_ = this->create_subscription<kr_msgs::msg::SystemState>("/kr/system/state", 10, std::bind(&KrRobotROS2StateSubscriber::robot_state_callback, this, _1));
    }
  

private:

    // Writing received data from system state message to standard output 
    void robot_state_callback(const kr_msgs::msg::SystemState::SharedPtr msg) const
    {
        std::cout.precision(2);

        std::cout << "System state info recveived: " << std::endl;
        std::cout << "  robot_mode : " << msg->robot_mode.str << std::endl;
        std::cout << "  robot_state : " << msg->robot_state.str << std::endl;

        std::cout << "  sensed_pos : " << msg->sensed_pos[0] << " "
                                    << msg->sensed_pos[1] << " "
                                    << msg->sensed_pos[2] << " "
                                    << msg->sensed_pos[3] << " "
                                    << msg->sensed_pos[4] << " "
                                    << msg->sensed_pos[5] << " "
                                    << msg->sensed_pos[6] << std::endl;

        std::cout << "  sensed_vel : " << msg->sensed_vel[0] << " "
                                    << msg->sensed_vel[1] << " "
                                    << msg->sensed_vel[2] << " "
                                    << msg->sensed_vel[3] << " "
                                    << msg->sensed_vel[4] << " "
                                    << msg->sensed_vel[5] << " "
                                    << msg->sensed_vel[6] << std::endl;
                                    
        std::cout << "  sensed_trq : " << msg->sensed_trq[0] << " "
                                    << msg->sensed_trq[1] << " "
                                    << msg->sensed_trq[2] << " "
                                    << msg->sensed_trq[3] << " "
                                    << msg->sensed_trq[4] << " "
                                    << msg->sensed_trq[5] << " "
                                    << msg->sensed_trq[6] << std::endl;

        std::cout << "  target_pos : " << msg->target_pos[0] << " "
                                    << msg->target_pos[1] << " "
                                    << msg->target_pos[2] << " "
                                    << msg->target_pos[3] << " "
                                    << msg->target_pos[4] << " "
                                    << msg->target_pos[5] << " "
                                    << msg->target_pos[6] << std::endl;                                   
                                    
        std::cout << "  target_vel : " << msg->target_vel[0] << " "
                                    << msg->target_vel[1] << " "
                                    << msg->target_vel[2] << " "
                                    << msg->target_vel[3] << " "
                                    << msg->target_vel[4] << " "
                                    << msg->target_vel[5] << " "
                                    << msg->target_vel[6] << std::endl;
                                    
        std::cout << "  target_trq : " << msg->target_trq[0] << " "
                                    << msg->target_trq[1] << " "
                                    << msg->target_trq[2] << " "
                                    << msg->target_trq[3] << " "
                                    << msg->target_trq[4] << " "
                                    << msg->target_trq[5] << " "
                                    << msg->target_trq[6] << std::endl;          
                                    
        std::cout << "  pos : " << msg->pos[0] << " "
                                << msg->pos[1] << " "
                                << msg->pos[2] << " " << std::endl; 
                                                                
        std::cout << "  rot : " << msg->rot[0] << " "
                                << msg->rot[1] << " "
                                << msg->rot[2] << " " << std::endl; 
                                
        std::cout << "  iob_digital_out : " ; for (auto val : msg->iob_digital_out) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  iob_relays : " ; for (auto val : msg->iob_relays) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  iob_current_out : " ; for (auto val : msg->iob_current_out) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  iob_voltage_out : " ; for (auto val : msg->iob_voltage_out) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  iob_safe_in : " ; for (auto val : msg->iob_safe_in) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  iob_current_in : " ; for (auto val : msg->iob_current_in) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  iob_voltage_in : " ; for (auto val : msg->iob_voltage_in) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  tio_digital_out : " ; for (auto val : msg->tio_digital_out) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  tio_power_supply : " ; for (auto val : msg->tio_power_supply) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  tio_analog_out : " ; for (auto val : msg->tio_analog_out) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  tio_current_in : " ; for (auto val : msg->tio_current_in) std::cout << val << " ";
        std::cout << std::endl;
        
        std::cout << "  tio_voltage_in : " ; for (auto val : msg->tio_voltage_in) std::cout << val << " ";
        std::cout << std::endl;
    }

    rclcpp::Subscription<kr_msgs::msg::SystemState>::SharedPtr robot_state_subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrRobotROS2StateSubscriber>());
  rclcpp::shutdown();
  return 0;
}
