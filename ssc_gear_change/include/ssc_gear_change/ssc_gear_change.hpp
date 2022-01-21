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

#ifndef GEAR_CHANGE_H
#define GEAR_CHANGE_H

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <std_msgs/msg/bool.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/gear_command.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>

#include "ssc_gear_change/ssc_gear_change_config.hpp"


static const std::string BASE_FRAME_ID = "base_link";

namespace ssc_gear_change
{
    class Node : public carma_ros2_utils::CarmaLifecycleNode
    {
        private:
        
            // Subscribers
            rclcpp::Subscription<automotive_platform_msgs::msg::GearFeedback>::SharedPtr gear_feedback_sub_;
            rclcpp::Subscription<automotive_platform_msgs::msg::SpeedMode>::SharedPtr speed_mode_sub_;
            rclcpp::Subscription<automotive_platform_msgs::msg::GearCommand>::SharedPtr gear_sub_;

            // Publishers
            std::shared_ptr<rclcpp::Publisher<automotive_platform_msgs::msg::SpeedMode>> speed_mode_pub_;
            std::shared_ptr<rclcpp::Publisher<automotive_platform_msgs::msg::GearCommand>> gear_pub_;

            // delegate logic implementation to Config class
            Config config_;

            // variables
            rclcpp::Time gear_change_stamp_;
            // initialize duration
            rclcpp::Duration delay_duration_= rclcpp::Duration(0.0);
            int current_gear_;
            int desired_gear_;
            int previous_desired_gear_;
            automotive_platform_msgs::msg::GearCommand gear_msg_;
            bool gear_cmd_subscribed_;

        public:
            /**
             * \brief Node constructor 
             */
            explicit Node(const rclcpp::NodeOptions &);

            // callbacks
            void callbackFromSSCGearFeedback(const automotive_platform_msgs::msg::GearFeedback::UniquePtr msg);
            void callbackFromGearSelect(const automotive_platform_msgs::msg::GearCommand::UniquePtr msg);
            void callbackFromSpeedCommand(const automotive_platform_msgs::msg::SpeedMode::UniquePtr msg);

            ////
            // Overrides
            ////
            carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    };
}

#endif  // GEAR_CHANGE_H