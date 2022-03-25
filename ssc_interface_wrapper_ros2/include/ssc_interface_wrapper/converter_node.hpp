/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 
 /**
  * Modification (C) Leidos 2022
  * Updated ssc_interface to ROS2 as new converter_node
  */
 
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <automotive_platform_msgs/msg/velocity_accel_cov.hpp>
#include <automotive_platform_msgs/msg/curvature_feedback.hpp>
#include <automotive_platform_msgs/msg/throttle_feedback.hpp>
#include <automotive_platform_msgs/msg/brake_feedback.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/steering_feedback.hpp>
#include <autoware_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <automotive_navigation_msgs/msg/module_state.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <autoware_msgs/msg/vehicle_cmd.hpp>

#include <automotive_platform_msgs/msg/steer_mode.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>
#include <automotive_platform_msgs/msg/gear_command.hpp>

#include <automotive_platform_msgs/msg/gear.hpp>


namespace ssc_interface_wrapper{

static const std::string BASE_FRAME_ID = "base_link";

  struct ConverterConfig
  {
   
    bool use_adaptive_gear_ratio_ = true;
    double loop_rate_ = 30.0;
    int command_timeout_ = 1000;
    double status_pub_rate_ = 30.0;
    double wheel_base_ = 2.79;
    double tire_radius_ = 0.39;
    double ssc_gear_ratio_ = 16.135;
    double acceleration_limit_ = 3.0;
    double deceleration_limit_ = 3.0;
    double max_curvature_rate_ = 0.15;
    double agr_coef_a_ = 15.713;
    double agr_coef_b_ = 0.053;
    double agr_coef_c_ = 0.042;

    // Stream operator for this config
    
    friend std::ostream &operator<<(std::ostream &output, const ConverterConfig &c)
    {
      output << "ssc_interface_wrapper::ConverterConfig { " << std::endl
           << "use_adaptive_gear_ratio_: " << c.use_adaptive_gear_ratio_ << std::endl
           << "loop_rate:" << c.loop_rate_ <<std::endl
           << "command_timeout:" << c.command_timeout_ <<std::endl
           << "status_pub_rate:" << c.status_pub_rate_<<std::endl
           << "wheel_base_: " << c.wheel_base_ << std::endl
           << "tire_radius: " << c.tire_radius_ << std::endl
           << "ssc_gear_ratio: " << c.ssc_gear_ratio_ << std::endl
           << "acceleration_limit: " << c.acceleration_limit_ << std::endl
           << "deceleration_limit: " << c.deceleration_limit_ << std::endl
           << "max_curvature_rate: "<< c.max_curvature_rate_ <<std::endl
           << "agr_coef_a_: " << c.agr_coef_a_ << std::endl
           << "agr_coef_b_: " << c.agr_coef_b_ << std::endl
           << "agr_coef_c_: " << c.agr_coef_c_ << std::endl
           << "}" << std::endl;
      return output;
    }
  };

class Converter : public carma_ros2_utils::CarmaLifecycleNode
{
    private:
    
    carma_ros2_utils::PubPtr<autoware_msgs::msg::VehicleStatus> vehicle_status_pub_; 
    carma_ros2_utils::PubPtr<geometry_msgs::msg::TwistStamped> current_twist_pub_;

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

    //autoware.ai subscriptions from autoware
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;
    carma_ros2_utils::SubPtr<autoware_msgs::msg::VehicleCmd> vehicle_cmd_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::Bool> engage_sub_;
    carma_ros2_utils::SubPtr<automotive_navigation_msgs::msg::ModuleState> module_states_sub_;
    // Callbacks
    void callback_from_guidance_state(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg);
    void callback_from_vehicle_cmd(const autoware_msgs::msg::VehicleCmd::UniquePtr msg);
    void callback_from_engage(const std_msgs::msg::Bool::UniquePtr msg);
    void callback_from_ssc_module_states(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg);

    void publish_vehicle_state(const autoware_msgs::msg::VehicleStatus& vehicle_status);

    //Timer callback
    void publish_vehicle_status();
    void publish_command();

    carma_ros2_utils::PubPtr<automotive_platform_msgs::msg::SteerMode> steer_mode_pub_;
    carma_ros2_utils::PubPtr<automotive_platform_msgs::msg::SpeedMode> speed_mode_pub_;
    carma_ros2_utils::PubPtr<automotive_platform_msgs::msg::TurnSignalCommand> turn_signal_pub_;
    carma_ros2_utils::PubPtr<automotive_platform_msgs::msg::GearCommand> gear_pub_;

    rclcpp::TimerBase::SharedPtr status_pub_timer_;
    rclcpp::TimerBase::SharedPtr command_pub_timer_;

    
    ConverterConfig config_;

    bool engage_;
    bool command_initialized_; 
    double current_velocity_;
    double adaptive_gear_ratio_;
    rclcpp::Time command_time_;
    autoware_msgs::msg::VehicleCmd vehicle_cmd_;
    automotive_navigation_msgs::msg::ModuleState module_states_;

    // Flag to indicate whether the ssc should shift the vehicle to park
    bool shift_to_park_{false};

    bool have_vehicle_status_ = false; // Flag to show if the vehicle status messages have been populated with new information
    bool have_twist_ = false;
    geometry_msgs::msg::TwistStamped current_twist_msg_; // Current twist message recieved from ssc
    autoware_msgs::msg::VehicleStatus current_status_msg_; // Current status message recieved from ssc

     //A small static value for comparing doubles
    static constexpr double epsilon_ = 0.001;

    public:

    /**
     * \brief constructor 
     */
    explicit Converter(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);


};
} //namespace ssc_interface_wrapper