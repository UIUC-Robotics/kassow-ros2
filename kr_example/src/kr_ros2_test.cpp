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

#include <kr_msg_ros2/msg/system_state.hpp>
#include <kr_msg_ros2/msg/number.hpp>
#include <kr_msg_ros2/msg/robot_pose.hpp>

#include <kr_msg_ros2/srv/move_joint.hpp>

#include <kr_msg_ros2/srv/select_jogging_frame.hpp>

#include <kr_msg_ros2/msg/jog_linear.hpp>
#include <kr_msg_ros2/msg/self_motion.hpp>
#include <kr_msg_ros2/msg/follow_joint.hpp>


using std::placeholders::_1;

class KrRobotROS2Subscriber : public rclcpp::Node {
    
public:
    
    KrRobotROS2Subscriber() : Node("kr_robot_ros2_subscriber") {
        // Publishers init
        follow_joint_publisher_ = this->create_publisher<kr_msg_ros2::msg::FollowJoint>("/kr/motion/follow_joint", 10);
        jog_linear_publisher_ = this->create_publisher<kr_msg_ros2::msg::JogLinear>("/kr/motion/jog_linear", 10);
        self_motion_publisher_ = this->create_publisher<kr_msg_ros2::msg::SelfMotion>("/kr/motion/self_motion", 10);

        // UNCOMMENT ONE OF OPTIONS TO START THREAD

        // Robot state subscriber example
        // robot_state_subscription_ = this->create_subscription<kr_msg_ros2::msg::SystemState>("/kr/system/state", 10, std::bind(&KrRobotROS2Subscriber::robot_state_callback, this, _1));

        // Move joint service example
        // move_joint();

        // Follow joint message example
        // follow_joint_timer_ = this->create_wall_timer(5000ms, std::bind(&KrRobotROS2Subscriber::follow_joint_callback, this));

        // Select jogging frame service and jog linear mnessage example
        // select_jogging_frame(1);
        // jog_linear_timer_ = this->create_wall_timer(20ms, std::bind(&KrRobotROS2Subscriber::jog_linear_callback, this));

        // Self motion message example
        // self_motion_timer_ = this->create_wall_timer(20ms, std::bind(&KrRobotROS2Subscriber::self_motion_callback, this));
    }
  

private:

    // Writing received data from system state message to standard output 
    void robot_state_callback(const kr_msg_ros2::msg::SystemState::SharedPtr msg) const
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

    // Move joint service example sending request to move to specified joint configuration
    void move_joint()
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("move_joint_node");
        rclcpp::Client<kr_msg_ros2::srv::MoveJoint>::SharedPtr client = node->create_client<kr_msg_ros2::srv::MoveJoint>("/kr/motion/move_joint");

        auto request = std::make_shared<kr_msg_ros2::srv::MoveJoint::Request>();

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

    // Each time callback is called, one configuration is selected and request to follow this configuration is published
    // configurations are changing each call.
    void follow_joint_callback(){
        auto message = kr_msg_ros2::msg::FollowJoint();
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

    // Sending message to jog X axis
    void jog_linear_callback(){
        auto message = kr_msg_ros2::msg::JogLinear();
        const std::array<double, 3> vel = { 100., 0., 0. };
        const std::array<double, 3> rot = { 0., 0., 0. };

        message.set__vel(vel);
        message.set__rot(rot);

        std ::cout << "PUBLISHING JOG LINEAR \n";
        jog_linear_publisher_->publish(message);
    }

    // Sending message to do self-motion in positive direction with relative speed 0.5
    void self_motion_callback(){
        auto message = kr_msg_ros2::msg::SelfMotion();

        message.set__speed(0.5);

        std ::cout << "PUBLISHING SELF MOTION \n\n";
        self_motion_publisher_->publish(message);
    }

    // Sending request to select jogging frame and waiting until result received
    void select_jogging_frame(int value)
    {
        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("select_jogging_node");
        rclcpp::Client<kr_msg_ros2::srv::SelectJoggingFrame>::SharedPtr client = node->create_client<kr_msg_ros2::srv::SelectJoggingFrame>("/kr/motion/select_jogging_frame");

        auto request = std::make_shared<kr_msg_ros2::srv::SelectJoggingFrame::Request>();
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

        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfull selection.");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Selection failed.");
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "/kr/motion/select_jogging_frame");
        }
    }



    rclcpp::Subscription<kr_msg_ros2::msg::SystemState>::SharedPtr robot_state_subscription_;
  
    rclcpp::Publisher<kr_msg_ros2::msg::FollowJoint>::SharedPtr follow_joint_publisher_;
    rclcpp::Publisher<kr_msg_ros2::msg::JogLinear>::SharedPtr jog_linear_publisher_;
    rclcpp::Publisher<kr_msg_ros2::msg::SelfMotion>::SharedPtr self_motion_publisher_;
    
    rclcpp::TimerBase::SharedPtr follow_joint_timer_;
    rclcpp::TimerBase::SharedPtr jog_linear_timer_;
    rclcpp::TimerBase::SharedPtr self_motion_timer_;
    
    bool temp = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KrRobotROS2Subscriber>());
  rclcpp::shutdown();
  return 0;
}
