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

#include "ssc_interface_wrapper/converter_node.hpp"


namespace ssc_interface_wrapper{

    namespace std_ph = std::placeholders;

    Converter::Converter(const rclcpp::NodeOptions &options)
        : carma_ros2_utils::CarmaLifecycleNode(options)
    {
        // Create initial config
        config_ = ConverterConfig();

        // Declare parameters
        config_.use_adaptive_gear_ratio_ = declare_parameter<bool>("use_adaptive_gear_ratio", config_.use_adaptive_gear_ratio_);
        config_.loop_rate_ = declare_parameter<double>("loop_rate", config_.loop_rate_);
        config_.command_timeout_ = declare_parameter<int>("command_timeout", config_.command_timeout_);
        config_.status_pub_rate_ = declare_parameter<double>("status_pub_rate", config_.status_pub_rate_);
        config_.wheel_base_ = declare_parameter<double>("wheel_base", config_.wheel_base_);
        config_.tire_radius_ = declare_parameter<double>("tire_radius", config_.tire_radius_);
        config_.ssc_gear_ratio_ = declare_parameter<double>("ssc_gear_ratio", config_.ssc_gear_ratio_);
        config_.acceleration_limit_ = declare_parameter<double>("acceleration_limit", config_.acceleration_limit_);
        config_.deceleration_limit_ = declare_parameter<double>("deceleration_limit", config_.deceleration_limit_);
        config_.max_curvature_rate_ = declare_parameter<double>("max_curvature_rate", config_.max_curvature_rate_);
        config_.agr_coef_a_ = declare_parameter<double>("agr_coef_a", config_.agr_coef_a_);
        config_.agr_coef_b_ = declare_parameter<double>("agr_coef_b", config_.agr_coef_b_);
        config_.agr_coef_c_ = declare_parameter<double>("agr_coef_c", config_.agr_coef_c_);
    }

    carma_ros2_utils::CallbackReturn Converter::handle_on_configure(const rclcpp_lifecycle::State &)
    {
       // Load parameters
        get_parameter<bool>("use_adaptive_gear_ratio", config_.use_adaptive_gear_ratio_);
        get_parameter<double>("loop_rate", config_.loop_rate_);
        get_parameter<int>("command_timeout", config_.command_timeout_);
        get_parameter<double>("status_pub_rate", config_.status_pub_rate_);
        get_parameter<double>("wheel_base", config_.wheel_base_);
        get_parameter<double>("tire_radius", config_.tire_radius_);
        get_parameter<double>("ssc_gear_ratio", config_.ssc_gear_ratio_);
        get_parameter<double>("acceleration_limit", config_.acceleration_limit_);
        get_parameter<double>("deceleration_limit", config_.deceleration_limit_);
        get_parameter<double>("max_curvature_rate", config_.max_curvature_rate_);
        get_parameter<double>("agr_coef_a", config_.agr_coef_a_);
        get_parameter<double>("agr_coef_b", config_.agr_coef_b_);
        get_parameter<double>("agr_coef_c", config_.agr_coef_c_);

        //Setup Subscribers

        //subscribers from CARMA
        guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("state", 1,
                                                                    std::bind(&Converter::callback_from_guidance_state, this, std_ph::_1));
        //subscribers from autoware
        vehicle_cmd_sub_ = create_subscription<autoware_msgs::msg::VehicleCmd>("vehicle_cmd", 1,
                                                                    std::bind(&Converter::callback_from_vehicle_cmd, this, std_ph::_1));
        engage_sub_ = create_subscription<std_msgs::msg::Bool>("vehicle/engage", 1,
                                                        std::bind(&Converter::callback_from_engage, this, std_ph::_1));

        // ssc topic subscribers
        module_states_sub_ = create_subscription<automotive_navigation_msgs::msg::ModuleState> ("as/module_states", 1, 
                                                                    std::bind(&Converter::callback_from_ssc_module_states, this, std_ph::_1));
        velocity_accel_sub_ = std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::VelocityAccelCov>>
            (this->get_node_base_interface(), "as/velocity_accel_cov", rmw_qos_profile_default);
        curvature_feedback_sub_ = std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::CurvatureFeedback>>
            (this->get_node_base_interface(), "as/curvature_feedback", rmw_qos_profile_default);
        throttle_feedback_sub_ = std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::ThrottleFeedback>>
            (this->get_node_base_interface(), "as/throttle_feedback", rmw_qos_profile_default);
        brake_feedback_sub_ = std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::BrakeFeedback>>
            (this->get_node_base_interface(), "as/brake_feedback", rmw_qos_profile_default);
        gear_feedback_sub_ = std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::GearFeedback>>
            (this->get_node_base_interface(), "as/gear_feedback", rmw_qos_profile_default);
        steering_wheel_sub_ = std::make_unique<message_filters::Subscriber<automotive_platform_msgs::msg::SteeringFeedback>>
            (this->get_node_base_interface(), "as/steering_feedback", rmw_qos_profile_default);
        ssc_feedbacks_sync_ = std::make_unique<message_filters::Synchronizer<SSCFeedbacksSyncPolicy>>(
            SSCFeedbacksSyncPolicy(10), *velocity_accel_sub_, *curvature_feedback_sub_, *throttle_feedback_sub_,
            *brake_feedback_sub_, *gear_feedback_sub_, *steering_wheel_sub_);
        ssc_feedbacks_sync_->registerCallback(
            std::bind(&Converter::callback_from_ssc_feedbacks, this, 
            std_ph::_1, std_ph::_2, std_ph::_3, std_ph::_4, std_ph::_5, std_ph::_6));

        ssc_twist_sync_ = std::make_unique<message_filters::Synchronizer<SSCTwistSyncPolicy>>(
            SSCTwistSyncPolicy(10), *velocity_accel_sub_, *curvature_feedback_sub_, *steering_wheel_sub_);

        ssc_twist_sync_->registerCallback(
            std::bind(&Converter::callback_for_twist_update, this, std_ph::_1, std_ph::_2, std_ph::_3));

        // Setup publishers
        // To autoware
        vehicle_status_pub_ = create_publisher<autoware_msgs::msg::VehicleStatus>("vehicle_status",10);
        current_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vehicle/twist",10);
        // To SSC
        steer_mode_pub_ = create_publisher<automotive_platform_msgs::msg::SteerMode>("as/arbitrated_steering_commands", 10);
        speed_mode_pub_ = create_publisher<automotive_platform_msgs::msg::SpeedMode>("as/arbitrated_speed_commands",10);
        turn_signal_pub_ = create_publisher<automotive_platform_msgs::msg::TurnSignalCommand>("as/turn_signal_command",10);

        rclcpp::QoS qos(1);
        qos.transient_local();
        gear_pub_ = create_publisher<automotive_platform_msgs::msg::GearCommand>("as/gear_select", qos);


        command_pub_timer_ = create_timer(get_clock(),std::chrono::duration<double>(1.0/config_.loop_rate_),
                                        std::bind(&Converter::publish_command, this));

        status_pub_timer_ = create_timer(get_clock(),std::chrono::duration<double>(1.0/config_.status_pub_rate_),
                             std::bind(&Converter::publish_vehicle_status, this));

        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;
    }

    void Converter::publish_vehicle_status(){
          if (have_vehicle_status_) {
            vehicle_status_pub_->publish(current_status_msg_);
            have_vehicle_status_ = false; // Reset the status flag to ensure we only publish new messages
        }
        if (have_twist_) {
            current_twist_pub_->publish(current_twist_msg_);
            have_twist_ = false; // Reset the status flag to ensure we only publish new messages
        }
    }

    void Converter::callback_from_guidance_state(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg){
        if (msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED)
        {
            shift_to_park_ = false;
        }
        else if (msg->state == carma_planning_msgs::msg::GuidanceState::ENTER_PARK)
        {
            shift_to_park_ = true;
        }
    }

    void Converter::callback_from_vehicle_cmd(const autoware_msgs::msg::VehicleCmd::UniquePtr msg){
        // If the current state is not active we will return to avoid comparing timestamps from different time sources (ie. states)
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            return;
        }
        command_time_ = this->now();
        vehicle_cmd_ = *msg;
        command_initialized_ = true;

    }

    void Converter::callback_from_engage(const std_msgs::msg::Bool::UniquePtr msg){
        engage_ = msg->data;
    }

    void Converter::callback_from_ssc_module_states(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg){
        if (msg->name.find("veh_controller") != std::string::npos)
        {
            module_states_ = *msg;
        }
    }

    void Converter::callback_from_ssc_feedbacks(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                                const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                                const automotive_platform_msgs::msg::ThrottleFeedback& msg_throttle,
                                                const automotive_platform_msgs::msg::BrakeFeedback& msg_brake,
                                                const automotive_platform_msgs::msg::GearFeedback& msg_gear,
                                                const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel)
    {
        builtin_interfaces::msg::Time stamp = msg_velocity.header.stamp;
        // update adaptive gear ratio (avoiding zero division)
        adaptive_gear_ratio_ =
            std::max(1e-5, config_.agr_coef_a_ + config_.agr_coef_b_ * msg_velocity.velocity * msg_velocity.velocity - config_.agr_coef_c_ * msg_steering_wheel.steering_wheel_angle);

        // current steering curvature
        double curvature = !config_.use_adaptive_gear_ratio_ ?
                        (msg_curvature.curvature) :
                        std::tan(msg_steering_wheel.steering_wheel_angle/ adaptive_gear_ratio_) / config_.wheel_base_;

        // Set current_velocity_ variable [m/s]
        current_velocity_ = msg_velocity.velocity;

        // vehicle_status (autoware_msgs::msg::VehicleStatus)
        autoware_msgs::msg::VehicleStatus vehicle_status;
        vehicle_status.header.frame_id = BASE_FRAME_ID;
        vehicle_status.header.stamp = stamp;

        // drive/steeringmode
        vehicle_status.drivemode = (module_states_.state == "active") ? autoware_msgs::msg::VehicleStatus::MODE_AUTO :
                                                                autoware_msgs::msg::VehicleStatus::MODE_MANUAL;

        vehicle_status.steeringmode = vehicle_status.drivemode;

        // speed [km/h]
        vehicle_status.speed = msg_velocity.velocity * 3.6;

        // drive/brake pedal [0,1000] (TODO: Scaling)
        vehicle_status.drivepedal = (int)(1000 * msg_throttle.throttle_pedal);
        vehicle_status.brakepedal = (int)(1000 * msg_brake.brake_pedal);

        // steering angle [rad]
        vehicle_status.angle = std::atan(curvature * config_.wheel_base_);

        // gearshift
        if (msg_gear.current_gear.gear == automotive_platform_msgs::msg::Gear::NONE)
        {
            vehicle_status.current_gear.gear = automotive_platform_msgs::msg::Gear::NONE;
        }
        else if (msg_gear.current_gear.gear == automotive_platform_msgs::msg::Gear::PARK)
        {
            vehicle_status.current_gear.gear = automotive_platform_msgs::msg::Gear::PARK;
        }
        else if (msg_gear.current_gear.gear == automotive_platform_msgs::msg::Gear::REVERSE)
        {
            vehicle_status.current_gear.gear= automotive_platform_msgs::msg::Gear::REVERSE;
        }
        else if (msg_gear.current_gear.gear == automotive_platform_msgs::msg::Gear::NEUTRAL)
        {
            vehicle_status.current_gear.gear = automotive_platform_msgs::msg::Gear::NEUTRAL;
        }
        else if (msg_gear.current_gear.gear == automotive_platform_msgs::msg::Gear::DRIVE)
        {
            vehicle_status.current_gear.gear = automotive_platform_msgs::msg::Gear::DRIVE;
        }

        // lamp/light cannot be obtain from SSC
        // vehicle_status.lamp
        // vehicle_status.light

        current_status_msg_ = vehicle_status;
        have_vehicle_status_ = true;

    }

    void Converter::callback_for_twist_update(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                  const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                  const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel)
    {
        
        // current steering curvature
        double curvature = !config_.use_adaptive_gear_ratio_ ?
                        (msg_curvature.curvature) :
                        std::tan(msg_steering_wheel.steering_wheel_angle/ adaptive_gear_ratio_) / config_.wheel_base_;

        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = BASE_FRAME_ID;
        twist.header.stamp = msg_velocity.header.stamp;
        twist.twist.linear.x = msg_velocity.velocity;               // [m/s]
        twist.twist.angular.z = curvature * msg_velocity.velocity;  // [rad/s]
        current_twist_msg_ = twist;

        have_twist_ = true;
    }

    void Converter::publish_command()
    {
        //
        // This method will publish 0 commands until the vehicle_cmd has actually been populated with non-zeros
        // When the vehicle_cmd_ becomes populated it will start to forward that instead

        rclcpp::Time stamp = this->now();

        // Desired values
        // Driving mode (If autonomy mode should be active, mode = 1)
        unsigned char desired_mode = engage_ ? 1 : 0;

        // Speed for SSC speed_model
        double desired_speed = vehicle_cmd_.ctrl_cmd.linear_velocity;

        // Curvature for SSC steer_model
        double desired_steering_angle = !config_.use_adaptive_gear_ratio_ ?
                                      vehicle_cmd_.ctrl_cmd.steering_angle :
                                      vehicle_cmd_.ctrl_cmd.steering_angle * config_.ssc_gear_ratio_ / adaptive_gear_ratio_;

        double desired_curvature = std::tan(desired_steering_angle) / config_.wheel_base_;

        // Gear (TODO: Use vehicle_cmd.gear)
        unsigned char desired_gear = automotive_platform_msgs::msg::Gear::NONE;

        if (engage_)
        {
            if (shift_to_park_)
            {
                if (current_velocity_ < epsilon_){
                    desired_gear = automotive_platform_msgs::msg::Gear::PARK;
                }
            }
            else
            {
                desired_gear = automotive_platform_msgs::msg::Gear::DRIVE;
            }
        }
        else
        {
            desired_gear = automotive_platform_msgs::msg::Gear::NONE;
        }

        // Turn signal
        unsigned char desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;

        if (vehicle_cmd_.lamp_cmd.l == 0 && vehicle_cmd_.lamp_cmd.r == 0)
        {
            desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::NONE;
        }
        else if (vehicle_cmd_.lamp_cmd.l == 1 && vehicle_cmd_.lamp_cmd.r == 0)
        {
            desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::LEFT;
        }
        else if (vehicle_cmd_.lamp_cmd.l == 0 && vehicle_cmd_.lamp_cmd.r == 1)
        {
            desired_turn_signal = automotive_platform_msgs::msg::TurnSignalCommand::RIGHT;
        }
        else if (vehicle_cmd_.lamp_cmd.l == 1 && vehicle_cmd_.lamp_cmd.r == 1)
        {
            // NOTE: HAZARD signal cannot be used in automotive_platform_msgs::msg::TurnSignalCommand
        }

        // Override desired speed to ZERO by emergency/timeout
        bool emergency = (vehicle_cmd_.emergency == 1);
        bool timeouted = command_initialized_ && (((this->now() - command_time_).seconds() * 1000) > config_.command_timeout_);

        if (emergency || timeouted)
        {
            // RCLCPP_ERROR_STREAM(get_logger(), "Emergency Stopping, emergency = %d, timeouted = %d", emergency, timeouted);
            desired_speed = 0.0;
        }

        // speed command
        automotive_platform_msgs::msg::SpeedMode speed_mode;
        speed_mode.header.frame_id = BASE_FRAME_ID;
        speed_mode.header.stamp = stamp;
        speed_mode.mode = desired_mode;
        speed_mode.speed = desired_speed;
        speed_mode.acceleration_limit = config_.acceleration_limit_;
        speed_mode.deceleration_limit = config_.deceleration_limit_;

        // steer command
        automotive_platform_msgs::msg::SteerMode steer_mode;
        steer_mode.header.frame_id = BASE_FRAME_ID;
        steer_mode.header.stamp = stamp;
        steer_mode.mode = desired_mode;
        steer_mode.curvature = desired_curvature;
        steer_mode.max_curvature_rate = config_.max_curvature_rate_;

        // turn signal command
        automotive_platform_msgs::msg::TurnSignalCommand turn_signal;
        turn_signal.header.frame_id = BASE_FRAME_ID;
        turn_signal.header.stamp = stamp;
        turn_signal.mode = desired_mode;
        turn_signal.turn_signal = desired_turn_signal;

        // gear command
        automotive_platform_msgs::msg::GearCommand gear_cmd;
        gear_cmd.header.frame_id = BASE_FRAME_ID;
        gear_cmd.header.stamp = stamp;
        gear_cmd.command.gear = desired_gear;

        // publish
        speed_mode_pub_->publish(speed_mode);
        steer_mode_pub_->publish(steer_mode);
        turn_signal_pub_->publish(turn_signal);
        gear_pub_->publish(gear_cmd);

        RCLCPP_INFO_STREAM(get_logger(), "Mode: " << (int)desired_mode << ", "
                           << "Speed: " << speed_mode.speed << ", "
                           << "Curvature: " << steer_mode.curvature << ", "
                           << "Gear: " << (int)gear_cmd.command.gear << ", "
                           << "TurnSignal: " << (int)turn_signal.turn_signal);

    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(ssc_interface_wrapper::Converter)
