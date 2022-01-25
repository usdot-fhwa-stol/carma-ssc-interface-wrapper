/*
 * Copyright (C) 2021 LEIDOS.
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include "ssc_interface_wrapper/ssc_interface_wrapper_config.hpp"
#include "ssc_interface_wrapper/ssc_interface_wrapper_worker.hpp"

#include <automotive_navigation_msgs/msg/module_state.hpp>
#include <j2735_v2x_msgs/msg/transmission_state.hpp>
#include <pacmod_msgs/msg/system_rpt_float.hpp>
#include <pacmod_msgs/msg/system_rpt_int.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <carma_driver_msgs/msg/robot_enabled.hpp>
#include <carma_driver_msgs/srv/set_enable_robotic.hpp>


namespace ssc_interface_wrapper
{
    class Node : public carma_ros2_utils::CarmaLifecycleNode
    {
        private:

            // one subscribers for ssc status
            rclcpp::Subscription<automotive_navigation_msgs::msg::ModuleState>::SharedPtr ssc_state_sub_;

            // subscribers for reading CAN data
            rclcpp::Subscription<pacmod_msgs::msg::SystemRptFloat>::SharedPtr steer_sub_;
            rclcpp::Subscription<pacmod_msgs::msg::SystemRptFloat>::SharedPtr brake_sub_;
            rclcpp::Subscription<pacmod_msgs::msg::SystemRptInt>::SharedPtr shift_sub_;

            // publishers for sending CAN data to CARMA
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> steering_wheel_angle_pub_;
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> brake_position_pub_;
            std::shared_ptr<rclcpp::Publisher<j2735_v2x_msgs::msg::TransmissionState>> transmission_pub_;

            // one publisher for SSC Interface node
            std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> vehicle_engage_pub_;

            // one publisher and one service for controller status to CARMA Platform
            std::shared_ptr<rclcpp::Publisher<carma_driver_msgs::msg::RobotEnabled>> robot_status_pub_;
            std::shared_ptr<rclcpp::Service<carma_driver_msgs::srv::SetEnableRobotic>> enable_robotic_control_srv_;

            // delegate logic implementation to Config class
            Config config_;
            // delegate logic implementation to worker class
            Worker worker_;

            // robotic status message as a local variable
            carma_driver_msgs::msg::RobotEnabled robotic_status_msg_;

            // bool flag indicates the wrapper is in the reengage mode
            bool reengage_state_;

            rclcpp::TimerBase::SharedPtr timer_;

        public:
            /**
             * \brief Node constructor 
             */
            explicit Node(const rclcpp::NodeOptions &);

            //  service callback to engage robotic control
            bool enable_robotic_control_cb(const std::shared_ptr<rmw_request_id_t> header,
                                        const std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Request> request, 
                                        const std::shared_ptr<carma_driver_msgs::srv::SetEnableRobotic::Response> response);

            // callback functions to handle CAN messages from PACMOD driver
            void ssc_state_cb(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg);
            void steer_cb(const pacmod_msgs::msg::SystemRptFloat::UniquePtr msg);
            void brake_cb(const pacmod_msgs::msg::SystemRptFloat::UniquePtr msg);
            void shift_cb(const pacmod_msgs::msg::SystemRptInt::UniquePtr msg);

            // process the latest vehicle status message and publish as robot status topic
            void publish_robot_status();

            // check controller health status
            void update_controller_health_status();

            // reengage robotic control if the reengage_state_ flag is set
            void reengage_robotic_control();

            void check_driver_timeout();

            ////
            // Overrides
            ////
            carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
            carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
    };
} // ssc_interface_wrapper