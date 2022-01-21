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

#include <ssc_gear_change/ssc_gear_change.hpp>

namespace ssc_gear_change
{
    Node::Node(const rclcpp::NodeOptions &options) :
        carma_ros2_utils::CarmaLifecycleNode(options),
        current_gear_(automotive_platform_msgs::msg::Gear::NONE),
        desired_gear_(automotive_platform_msgs::msg::Gear::NONE),
        previous_desired_gear_(automotive_platform_msgs::msg::Gear::NONE),
        gear_change_stamp_(this->now()),
        gear_cmd_subscribed_(false)
    {
        // Create initial config
        config_ = Config();
        config_.delay_time_ = declare_parameter<double>("delay_time", config_.delay_time_);

    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State&)
    {
        // Reset config
        config_ = Config();

        //Load parameters
        get_parameter<double>("delay_time", config_.delay_time_);
        delay_duration_ = rclcpp::Duration(config_.delay_time_, 0.0);

        // subscriber from ssc
        gear_feedback_sub_ = create_subscription<automotive_platform_msgs::msg::GearFeedback>("as/gear_feedback", 10, 
                                                std::bind(&Node::callbackFromSSCGearFeedback, this, std::placeholders::_1));
        // subscribers from ssc_interface                                                
        speed_mode_sub_ = create_subscription<automotive_platform_msgs::msg::SpeedMode>("ssc_interface/arbitrated_speed_commands", 10, 
                                                std::bind(&Node::callbackFromSpeedCommand, this, std::placeholders::_1));
        gear_sub_ = create_subscription<automotive_platform_msgs::msg::GearCommand>("ssc_interface/gear_select", 10, 
                                                std::bind(&Node::callbackFromGearSelect, this, std::placeholders::_1));

        //Initialize all publishers
        speed_mode_pub_ = create_publisher<automotive_platform_msgs::msg::SpeedMode>("as/arbitrated_speed_commands", 10);
        gear_pub_ = create_publisher<automotive_platform_msgs::msg::GearCommand>("as/gear_select", 1);

        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;
    }

    void Node::callbackFromSSCGearFeedback(const automotive_platform_msgs::msg::GearFeedback::UniquePtr msg)
    {
        current_gear_ = msg->current_gear.gear;
    }

    void Node::callbackFromGearSelect(const automotive_platform_msgs::msg::GearCommand::UniquePtr msg)
    {
        gear_cmd_subscribed_ = true;
        gear_msg_ = *msg.get();
        desired_gear_ = msg->command.gear;
        if(desired_gear_ != previous_desired_gear_)
        {
            gear_change_stamp_ = this->now();
        }
        previous_desired_gear_ = desired_gear_;
    }

    void Node::callbackFromSpeedCommand(const automotive_platform_msgs::msg::SpeedMode::UniquePtr msg)
    {
        if(!gear_cmd_subscribed_)
        {
            return;
        }

        bool wait_for_brake = this->now() - gear_change_stamp_ < delay_duration_;
        bool is_stopping_gear = (current_gear_ == automotive_platform_msgs::msg::Gear::NONE
                                || current_gear_ == automotive_platform_msgs::msg::Gear::PARK
                                || current_gear_ == automotive_platform_msgs::msg::Gear::NEUTRAL);

        bool is_gear_desired_value = (current_gear_ == desired_gear_);

        // speed command
        automotive_platform_msgs::msg::SpeedMode speed_mode = *msg.get();
        if(is_stopping_gear || wait_for_brake || !is_gear_desired_value)
        {
            speed_mode.speed = 0;
        }

        automotive_platform_msgs::msg::GearCommand gear_cmd = gear_msg_;
        if( wait_for_brake )
        {
            // gear command
            gear_cmd.command.gear = current_gear_;
        }

        // publish
        speed_mode_pub_->publish(speed_mode);
        gear_pub_->publish(gear_cmd);

        RCLCPP_INFO_STREAM(get_logger(), "CurrentGear: " << (int)current_gear_ << ", "
                                << "DesiredGear: " << (int)desired_gear_ << ", "
                                << "wait_for_brake: " << wait_for_brake << ", "
                                << "SpeedCommand: " << speed_mode.speed);
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(ssc_gear_change::Node)