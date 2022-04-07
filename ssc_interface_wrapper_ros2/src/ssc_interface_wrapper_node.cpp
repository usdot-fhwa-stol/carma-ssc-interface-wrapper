/*
 * Copyright (C) 2022 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include "ssc_interface_wrapper/ssc_interface_wrapper_node.hpp"

namespace ssc_interface_wrapper{
    
    Node::Node(const rclcpp::NodeOptions &options)
        : carma_ros2_utils::CarmaLifecycleNode(options)
    {
        // Create initial config
        config_ = Config();

        // Declare parameters
        config_.controller_timeout = declare_parameter<double>("controller_timeout", config_.controller_timeout);

    }

    void Node::reengage_robotic_control()
    {
        if(!worker_.get_current_ssc_state().compare("ready")) {
            std_msgs::msg::Bool engage_cmd;
            engage_cmd.data = true;
            vehicle_engage_pub_->publish(engage_cmd);
            reengage_state_ = false;
        }
    }

    void Node::publish_robot_status()
    {
        robotic_status_msg_.robot_active = worker_.is_engaged();
        robot_status_pub_->publish(robotic_status_msg_);
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State& )
    {
        // Reset config
        config_ = Config();

        // Create object for worker class
        worker_ = Worker();

        //Load parameters
        get_parameter<double>("controller_timeout", config_.controller_timeout);

        reengage_state_ = false;

        //setup subscribers
        ssc_state_sub_ = create_subscription<automotive_navigation_msgs::msg::ModuleState>("module_states", 5,
                                                                                        std::bind(&Node::ssc_state_cb, this, std::placeholders::_1));
        steer_sub_ = create_subscription<pacmod3_msgs::msg::SystemRptFloat>("parsed_tx/steer_rpt", 5,
                                                                            std::bind(&Node::steer_cb, this, std::placeholders::_1));
        brake_sub_ = create_subscription<pacmod3_msgs::msg::SystemRptFloat>("parsed_tx/brake_rpt", 5, 
                                                                            std::bind(&Node::brake_cb, this, std::placeholders::_1));
        shift_sub_ = create_subscription<pacmod3_msgs::msg::SystemRptInt>("parsed_tx/shift_rpt", 5, 
                                                                        std::bind(&Node::shift_cb, this, std::placeholders::_1));

        //Initialize all publishers
        steering_wheel_angle_pub_ = create_publisher<std_msgs::msg::Float64>("can/steering_wheel_angle", 1);
        brake_position_pub_ = create_publisher<std_msgs::msg::Float64>("can/brake_position", 1);
        transmission_pub_ = create_publisher<j2735_v2x_msgs::msg::TransmissionState>("can/transmission_state",1);
        robot_status_pub_   = create_publisher<carma_driver_msgs::msg::RobotEnabled>("controller/robot_status", 1);
        vehicle_engage_pub_ = create_publisher<std_msgs::msg::Bool>("vehicle/engage", 5);                                                                                                                                                                                        
        
        // initialize services
        enable_robotic_control_srv_ = create_service<carma_driver_msgs::srv::SetEnableRobotic>("controller/enable_robotic", 
                                                                                    std::bind(&Node::enable_robotic_control_cb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));                                                                  
        
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_activate(const rclcpp_lifecycle::State& )
    {
        //Timer setup
        timer_ = this->create_wall_timer(std::chrono::duration<double>(config_.controller_timeout/2.0), 
        std::bind(&Node::check_driver_timeout, this));

        // Initialize timeout check
        worker_.last_vehicle_status_time_ = this->now();

        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_deactivate(const rclcpp_lifecycle::State &prev_state) 
    {
        reengage_state_ = false; // Do not allow this flag to remain set if the node is deactivated. 
                                 // An additional re-engagement should be required in that case.
        return CallbackReturn::SUCCESS;
    }

    void Node::check_driver_timeout()
    {
        worker_.check_driver_status(this->now(), config_.controller_timeout);
    }

    bool Node::enable_robotic_control_cb(const std::shared_ptr<rmw_request_id_t> header,
                                        const std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Request> req, 
                                        const std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Response> resp)
    {
        std_msgs::msg::Bool engage_cmd;
        std::string current_state = worker_.get_current_ssc_state();
        
        if(!current_state.compare("ready")) {
            if(req->set == carma_driver_msgs::srv::SetEnableRobotic::Request::ENABLE) {
                engage_cmd.data = true;
                vehicle_engage_pub_->publish(engage_cmd);
            }
        } else if(!current_state.compare("active")) {
            if(req->set == carma_driver_msgs::srv::SetEnableRobotic::Request::DISABLE) {
                engage_cmd.data = false;
                vehicle_engage_pub_->publish(engage_cmd);
            }
        } else if(!current_state.compare("failure")) {
            engage_cmd.data = false;
            vehicle_engage_pub_->publish(engage_cmd);
            if(req->set == carma_driver_msgs::srv::SetEnableRobotic::Request::ENABLE) {
                // This flag is required because the SSC needs to be sent a negative engagement after operator override 
                // before it will accept a positive engagement.
                // In the lines above we send the negative engagement, but we need to wait for the status to change again
                // before we send the re-engagement. This flag tells us of the need to do that. 
                reengage_state_ = true;
            }
        }
        return true;

    }
 
    void Node::ssc_state_cb(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg)
    {
        // message needs to go to library. Unique ptr cant be sent
        automotive_navigation_msgs::msg::ModuleState module_state = *msg;
        worker_.on_new_status_msg(module_state, this->now());
        // robot_enabled field is true once the lower level controller is running
	    robotic_status_msg_.robot_enabled = true;
        publish_robot_status();

        // When we get a new status message we need to check if the system is trying to renegage and do so if it is. 
        if(reengage_state_)
        {
            reengage_robotic_control();
        }

    }

    void Node::steer_cb(const pacmod3_msgs::msg::SystemRptFloat::UniquePtr msg)
    {
        std_msgs::msg::Float64 steer_msg;
        steer_msg.data = msg->output;
        steering_wheel_angle_pub_->publish(steer_msg);
    }

    void Node::brake_cb(const pacmod3_msgs::msg::SystemRptFloat::UniquePtr msg)
    {
        std_msgs::msg::Float64 brake_msg;
        brake_msg.data = msg->output;
        brake_position_pub_->publish(brake_msg);
    }

    void Node::shift_cb(const pacmod3_msgs::msg::SystemRptInt::UniquePtr msg)
    {
        pacmod3_msgs::msg::SystemRptInt shift_msg = *msg;
        j2735_v2x_msgs::msg::TransmissionState transmission_msg;
        transmission_msg.transmission_state = worker_.convert_shift_state_to_J2735(shift_msg);
        transmission_pub_->publish(transmission_msg);
    }

} //namespace ssc_interface_wrapper

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(ssc_interface_wrapper::Node)