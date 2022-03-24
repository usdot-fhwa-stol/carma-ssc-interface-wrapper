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
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <automotive_platform_msgs/msg/gear.hpp>


/* This is a temporary class for the carma ssc wrapper that supports communication with the ssc.
This node is responsible for converting information published and subscribed to by the autoware.auto ssc package to the message format
accepted by the ssc while the ros2 conversion is ongoing */


namespace ssc_interface_wrapper{

static const std::string BASE_FRAME_ID = "base_link";

class Convertor : public carma_ros2_utils::CarmaLifecycleNode
{
    private:
    //Subscribers
    // From autoware

    carma_ros2_utils::PubPtr<autoware_msgs::msg::VehicleStatus> vehicle_status_pub_; 
    carma_ros2_utils::PubPtr<geometry_msgs::msg::TwistStamped> current_twist_pub_;

    // Callbacks
    void auto_gear_cb(const automotive_platform_msgs::msg::GearCommand& m_gear_msg);
    void auto_speed_cb(const automotive_platform_msgs::msg::SpeedMode& m_speed_msg);
    void auto_steer_cb(const automotive_platform_msgs::msg::SteerMode& m_steer_msg);
    void auto_turn_signal_cb(const automotive_platform_msgs::msg::TurnSignalCommand& m_turn_signal_msg);
    void auto_kinematic_state_cb(const autoware_auto_msgs::msg::VehicleKinematicState& kinematic_state_msg);


    //autoware.ai publishes a vehicle status topic after subscribing to ssc topics, that needs to be created here
    //Create vehicle status message after subscribing to ssc topics
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::VelocityAccelCov> velocity_accel_sub_;
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::CurvatureFeedback> curvature_feedback_sub_;
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::ThrottleFeedback> throttle_feedback_sub_;
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::BrakeFeedback> brake_feedback_sub_;
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::GearFeedback> gear_feedback_sub_;
    carma_ros2_utils::SubPtr<automotive_platform_msgs::msg::SteeringFeedback> steering_wheel_sub_;
    //Callbacks for vehicle_status
    void velocity_accel_cb(const automotive_platform_msgs::msg::VelocityAccelCov::UniquePtr msg_velocity);
    void curvature_feedback_cb(const automotive_platform_msgs::msg::CurvatureFeedback::UniquePtr msg_curvature);
    void throttle_feedback_cb(const automotive_platform_msgs::msg::ThrottleFeedback::UniquePtr msg_throttle);
    void brake_feedback_cb(const automotive_platform_msgs::msg::BrakeFeedback::UniquePtr msg_brake);
    void gear_feedback_cb(const automotive_platform_msgs::msg::GearFeedback::UniquePtr msg_gear);
    void steering_feedback_cb(const automotive_platform_msgs::msg::SteeringFeedback::UniquePtr msg_steering_wheel);
    // Global variables to store ssc feedback msgs
    automotive_platform_msgs::msg::VelocityAccelCov velocity_feedback_;
    automotive_platform_msgs::msg::CurvatureFeedback curvature_feedback_;
    automotive_platform_msgs::msg::ThrottleFeedback throttle_feedback_;
    automotive_platform_msgs::msg::BrakeFeedback brake_feedback_;
    automotive_platform_msgs::msg::GearFeedback gear_feedback_;
    automotive_platform_msgs::msg::SteeringFeedback steering_feedback_;
    // Consolidated Feedback for vehicle status
    void callback_from_ssc_feedbacks(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                    const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                    const automotive_platform_msgs::msg::ThrottleFeedback& msg_throttle,
                                    const automotive_platform_msgs::msg::BrakeFeedback& msg_brake,
                                    const automotive_platform_msgs::msg::GearFeedback& msg_gear,
                                    const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel);
    // Consolidated Feedback for twist
    void callback_for_twist_update(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                  const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                  const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel);

    //Things autoware.ai is subscribing to
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::VehicleCmd> vehicle_cmd_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::Bool> engage_sub_;
    carma_ros2_utils::SubPtr<automotive_navigation_msgs::msg::ModuleState> module_states_sub_;
    // Callbacks
    void callback_from_guidance_state(const carma_planning_msgs::msg::GuidanceState::UniqPtr msg);
    void callback_from_vehicle_cmd(const autoware_msgs::msg::VehicleCmd::UniqPtr msg);
    void callback_from_engage(const std_msgs::msg::Bool::UniquePtr msg);
    void callback_from_ssc_module_states(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg);
    
    //Timer callback
    void publish_vehicle_status();

    rclcpp::TimerBase::SharedPtr example_timer_;


    //Constants from autoware.ai as package config - ros params 
    // Check values in config
    bool use_adaptive_gear_ratio_ =true;
    double status_pub_rate = 30.0;

    double agr_coef_a_ = 15.713;
    double agr_coef_b_ = 0.053;
    double agr_coef_c_ = 0.042;
    double wheel_base_ = 2.79;

    // Other global variables
    bool engage_;
    bool command_initialized_; 
    double current_velocity_;
    double adaptive_gear_ratio_;
    rclcpp::Time command_time_;
    autoware_msgs::msg::VehicleCmd vehicle_cmd_;
    automotive_navigation_msgs::msg::ModuleState module_states_;

    // rclcpp::rate status_pub_rate_;
    // Flag to indicate whether the ssc should shift the vehicle to park
    bool shift_to_park_{false};

    bool have_vehicle_status_ = false; // Flag to show if the vehicle status messages have been populated with new information
    bool have_twist_ = false;
    geometry_msgs::msg::TwistStamped current_twist_msg_; // Current twist message recieved from ssc
    autoware_msgs::msg::VehicleStatus current_status_msg_; // Current status message recieved from ssc

    public:

    /**
     * \brief constructor 
     */
    explicit Convertor(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);


};
} //namespace ssc_interface_wrapper