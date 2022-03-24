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
        config_.wheel_base_ = declare_parameter<double>("wheel_base", config_.wheel_base_);
        config_.status_pub_rate_ = declare_parameter<double>("status_pub_rate", config_.status_pub_rate_);
        config_.agr_coef_a_ = declare_parameter<double>("agr_coef_a", config_.agr_coef_a_);
        config_.agr_coef_b_ = declare_parameter<double>("agr_coef_b", config_.agr_coef_b_);
        config_.agr_coef_c_ = declare_parameter<double>("agr_coef_c", config_.agr_coef_c_);
    }

    carma_ros2_utils::CallbackReturn Converter::handle_on_configure(const rclcpp_lifecycle::State &)
   {
       // Load parameters
        get_parameter<bool>("use_adaptive_gear_ratio", config_.use_adaptive_gear_ratio_);
        get_parameter<double>("wheel_base", config_.wheel_base_);
        get_parameter<double>("status_pub_rate", config_.status_pub_rate_);
        get_parameter<double>("agr_coef_a", config_.agr_coef_a_);
        get_parameter<double>("agr_coef_b", config_.agr_coef_b_);
        get_parameter<double>("agr_coef_c", config_.agr_coef_c_);

        // status_pub_rate_ = rclcpp::rate::GenericRate(config_.status_pub_rate_);

        //Setup Subscribers

        //subscribers from CARMA
        guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("/state", 1, 
                                                                    std::bind(&Converter::callback_from_guidance_state, this, std_ph::_1));
        // autoware.ai subscribers from autoware                                                                  
        vehicle_cmd_sub_ = create_subscription<autoware_msgs::msg::VehicleCmd>("vehicle_cmd", 1,
                                                                    std::bind(&Converter::callback_from_vehicle_cmd, this, std_ph::_1));
        engage_sub_ = create_subscription<std_msgs::msg::Bool>("vehicle/engage", 1,
                                                        std::bind(&Converter::callback_from_engage, this, std_ph::_1));                                                                    
                                                                                                                                                             
        // ssc topic subscribers
        module_states_sub_ = create_subscription<automotive_navigation_msgs::msg::ModuleState> ("as/module_states", 1, 
                                                                    std::bind(&Converter::callback_from_ssc_module_states, this, std_ph::_1));

        velocity_accel_sub_ = create_subscription<automotive_platform_msgs::msg::VelocityAccelCov>("as/velocity_accel_cov",10,
                                                                    std::bind(&Converter::velocity_accel_cb, this, std_ph::_1));
        curvature_feedback_sub_ = create_subscription<automotive_platform_msgs::msg::CurvatureFeedback>("as/curvature_feedback", 10,
                                                                    std::bind(&Converter::curvature_feedback_cb, this, std_ph::_1));
        throttle_feedback_sub_ = create_subscription<automotive_platform_msgs::msg::ThrottleFeedback>("as/throttle_feedback", 10,
                                                                    std::bind(&Converter::throttle_feedback_cb, this, std_ph::_1));
        brake_feedback_sub_ = create_subscription<automotive_platform_msgs::msg::BrakeFeedback>("as/gear_feedback", 10,
                                                                    std::bind(&Converter::brake_feedback_cb, this, std_ph::_1));
        gear_feedback_sub_ = create_subscription<automotive_platform_msgs::msg::GearFeedback>("as/steering_feedback", 10, 
                                                                    std::bind(&Converter::gear_feedback_cb, this, std_ph::_1));
        steering_wheel_sub_ = create_subscription<automotive_platform_msgs::msg::SteeringFeedback>("as/steering_feedback", 10,
                                                                    std::bind(&Converter::steering_feedback_cb, this, std_ph::_1));
                                                                                                                                                                                                                                                                                                                                                                                                                       
        callback_from_ssc_feedbacks(velocity_feedback_, curvature_feedback_, throttle_feedback_, brake_feedback_, gear_feedback_, steering_feedback_);
        callback_for_twist_update(velocity_feedback_,curvature_feedback_,steering_feedback_);

        // Setup publishers
        // To autoware system
        vehicle_status_pub_ = create_publisher<autoware_msgs::msg::VehicleStatus>("vehicle_status",10);
        current_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vehicle/twist",10);
        // To autoware.auto
        m_state_pub_ = create_publisher<autoware_auto_msgs::msg::VehicleStateCommand>("",10);
        m_command_pub_ = create_publisher<autoware_auto_msgs::msg::VehicleControlCommand>("vehicle_command",10);


        // These are published in a timed callback
        // Using 30Hz as callback rate - TO DO: Update this to use status_pub_rate
        //std::chrono::milliseconds(33.333333333333)
        status_pub_timer = create_timer(
        get_clock(),
        std::chrono::milliseconds(33),    
        std::bind(&Converter::publish_vehicle_status, this));

        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;
   }
   
   void Converter::publish_vehicle_state(const autoware_msgs::msg::VehicleStatus& vehicle_status){
       //Publishes autoware_auto_msgs::msg::VehicleStateCommand which is subscribed to by autoware.auto
       autoware_auto_msgs::msg::VehicleStateCommand vehicle_state_cmd;
       vehicle_state_cmd.stamp = vehicle_status.header.stamp;

       //Get info from vehicle_status

        //Get blinker - autoware_msgs doesn't have BLINKER_OFF type which exists in autoware_auto_msgs
        if(vehicle_status.lamp == autoware_msgs::msg::VehicleStatus::LAMP_LEFT){
            vehicle_state_cmd.blinker = autoware_auto_msgs::msg::VehicleStateCommand::BLINKER_LEFT;
        }
        else if(vehicle_status.lamp == autoware_msgs::msg::VehicleStatus::LAMP_RIGHT){
            vehicle_state_cmd.blinker = autoware_auto_msgs::msg::VehicleStateCommand::BLINKER_RIGHT;
        }
        else if(vehicle_status.lamp == autoware_msgs::msg::VehicleStatus::LAMP_HAZARD){
            vehicle_state_cmd.blinker = autoware_auto_msgs::msg::VehicleStateCommand::BLINKER_HAZARD;
        }
        else{
            vehicle_state_cmd.blinker = autoware_auto_msgs::msg::VehicleStateCommand::BLINKER_NO_COMMAND;
        }

        //autoware_msgs doesn't define light types
        // Get gear
       if (vehicle_status.current_gear.gear == autoware_msgs::msg::Gear::NONE)
        {
            vehicle_state_cmd.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_NO_COMMAND;
        }
        else if (vehicle_status.current_gear.gear == autoware_msgs::msg::Gear::PARK)
        {
            vehicle_state_cmd.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_PARK;
        }
        else if (vehicle_status.current_gear.gear == autoware_msgs::msg::Gear::REVERSE)
        {
            vehicle_state_cmd.gear= autoware_auto_msgs::msg::VehicleStateCommand::GEAR_REVERSE;
        }
        else if (vehicle_status.current_gear.gear == autoware_msgs::msg::Gear::NEUTRAL)
        {
            vehicle_state_cmd.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_NEUTRAL;
        }
        else if (vehicle_status.current_gear.gear == autoware_msgs::msg::Gear::DRIVE)
        {
            vehicle_state_cmd.gear = autoware_auto_msgs::msg::VehicleStateCommand::GEAR_DRIVE;
        }

        // Get Mode
        if(vehicle_status.drivemode == autoware_msgs::msg::VehicleStatus::MODE_MANUAL){
            vehicle_state_cmd.mode = autoware_auto_msgs::msg::VehicleStateCommand::MODE_MANUAL;
        }
        else if(vehicle_status.drivemode == autoware_msgs::msg::VehicleStatus::MODE_AUTO){
            vehicle_state_cmd.mode = autoware_auto_msgs::msg::VehicleStateCommand::MODE_AUTONOMOUS;
        }
        else{
            vehicle_state_cmd.mode = autoware_auto_msgs::msg::VehicleStateCommand::MODE_NO_COMMAND;
        }

        /*Note: headlight, wiper, hand_brake, horn not defined in autoware_msgs (but exist in autoware_auto_msgs) */
        m_state_pub_->publish(vehicle_state_cmd);

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
        command_time_ = this->now();
        vehicle_cmd_ = *msg;
        command_initialized_ = true;

        autoware_auto_msgs::msg::VehicleControlCommand vehicle_control_cmd;
        vehicle_control_cmd.stamp = msg->header.stamp;
        vehicle_control_cmd.long_accel_mps2 = msg->ctrl_cmd.linear_acceleration;
        vehicle_control_cmd.velocity_mps = msg->ctrl_cmd.linear_velocity;
        vehicle_control_cmd.front_wheel_angle_rad = msg->ctrl_cmd.steering_angle; 
        vehicle_control_cmd.rear_wheel_angle_rad; //***Assumption - steering_angle defined in autoware_msgs is only front_wheel_angle

        m_command_pub_->publish(vehicle_control_cmd);
    }

    void Converter::callback_from_engage(const std_msgs::msg::Bool::UniquePtr msg){
        engage_ = msg->data;
    }

    void Converter::callback_from_ssc_module_states(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg){
        if (msg->name.find("veh_controller") != std::string::npos)
        {
            module_states_ = *msg;  // *_veh_controller status is used for 'drive/steeringmode'
        }
    }

    void Converter::velocity_accel_cb(const automotive_platform_msgs::msg::VelocityAccelCov::UniquePtr msg_velocity){
        velocity_feedback_ = *msg_velocity;
    }

    void Converter::curvature_feedback_cb(const automotive_platform_msgs::msg::CurvatureFeedback::UniquePtr msg_curvature){
        curvature_feedback_ = *msg_curvature;
    }

    void Converter::throttle_feedback_cb(const automotive_platform_msgs::msg::ThrottleFeedback::UniquePtr msg_throttle){
        throttle_feedback_ = *msg_throttle;
    }

    void Converter::brake_feedback_cb(const automotive_platform_msgs::msg::BrakeFeedback::UniquePtr msg_brake){
        brake_feedback_ = *msg_brake;
    }

    void Converter::gear_feedback_cb(const automotive_platform_msgs::msg::GearFeedback::UniquePtr msg_gear){
        gear_feedback_ = *msg_gear;
    }

    void Converter::steering_feedback_cb(const automotive_platform_msgs::msg::SteeringFeedback::UniquePtr msg_steering_wheel){
        steering_feedback_ = *msg_steering_wheel;
    }

    void Converter::callback_from_ssc_feedbacks(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                                const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                                const automotive_platform_msgs::msg::ThrottleFeedback& msg_throttle,
                                                const automotive_platform_msgs::msg::BrakeFeedback& msg_brake,
                                                const automotive_platform_msgs::msg::GearFeedback& msg_gear,
                                                const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel)
    {
        builtin_interfaces::msg::Time stamp = msg_velocity.header.stamp;
        // update adaptive gear ratio (avoiding zero divizion)
        adaptive_gear_ratio_ =
            std::max(1e-5, config_.agr_coef_a_ + config_.agr_coef_b_ * msg_velocity.velocity * msg_velocity.velocity - config_.agr_coef_c_ * msg_steering_wheel.steering_wheel_angle);

        // current steering curvature
        double curvature = !config_.use_adaptive_gear_ratio_ ?
                         (msg_curvature.curvature) :
                         std::tan(msg_steering_wheel.steering_wheel_angle/ adaptive_gear_ratio_) / wheel_base_;

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
        vehicle_status.angle = std::atan(curvature * wheel_base_);

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
        have_vehicle_status_ = true; // Set vehicle status message flag to true

    }                                    

    void Converter::callback_for_twist_update(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                  const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                  const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel)
    {
        // current steering curvature
        double curvature = !config_.use_adaptive_gear_ratio_ ?
                         (msg_curvature.curvature) :
                         std::tan(msg_steering_wheel.steering_wheel_angle/ adaptive_gear_ratio_) / wheel_base_;

        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = BASE_FRAME_ID;
        twist.header.stamp = msg_velocity.header.stamp;
        twist.twist.linear.x = msg_velocity.velocity;               // [m/s]
        twist.twist.angular.z = curvature * msg_velocity.velocity;  // [rad/s]
        current_twist_msg_ = twist;

        have_twist_ = true;
    }                                  



}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(ssc_interface_wrapper::Converter)