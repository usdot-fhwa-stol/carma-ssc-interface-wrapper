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

#include "ssc_interface_wrapper_ros2/temp_convertor_node.hpp"


namespace ssc_interface_wrapper{

    namespace std_ph = std::placeholders;

    Convertor::Convertor(const rclcpp::NodeOptions &options)
        : carma_ros2_utils::CarmaLifecycleNode(options)
    {

    }

    carma_ros2_utils::CallbackReturn Convertor::handle_on_configure(const rclcpp_lifecycle::State &)
   {
        // status_pub_rate_ = rclcpp::rate::GenericRate(status_pub_rate);

        //Setup Subscribers
        guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("/state", 1, 
                                                                    std::bind(&Convertor::callback_from_guidance_state, this, std_ph::_1));
        vehicle_cmd_sub_ = create_subscription<autoware_msgs::msg::VehicleCmd>("vehicle_cmd", 1,
                                                                    std::bind(&Convertor::callback_from_engage, this, std_ph::_1));
        engage_sub_ = create_subscription<std_msgs::msg::Bool>("vehicle/engage", 1,
                                                        std::bind(&Convertor::callback_from_engage, this, std_ph::_1));                                                                    
        module_states_sub_ = create_subscription<automotive_navigation_msgs::msg::ModuleState> ("as/module_states", 1, 
                                                                    std::bind(&Convertor::callback_from_ssc_module_states, this, std_ph::_1));                                                                                                                                                     


        // Subscribers for vehicle status
        velocity_accel_sub_ = create_subscription<autoware_platform_msgs::msg::VelocityAccelCov>("as/velocity_accel_cov",10,
                                                                    std::bind(&Convertor::velocity_accel_cb, this, std_ph::_1));
        curvature_feedback_sub_ = create_subscription<autoware_platform_msgs::msg::CurvatureFeedback>("as/curvature_feedback", 10,
                                                                    std::bind(&Convertor::curvature_feedback_cb, this, std_ph::_1));
        throttle_feedback_sub_ = create_subscription<autoware_platform_msgs::msg::ThrottleFeedback>("as/throttle_feedback", 10,
                                                                    std::bind(&Convertor::throttle_feedback_cb, this, std_ph::_1));
        brake_feedback_sub_ = create_subscription<autoware_platform_msgs::msg::BrakeFeedback>("as/gear_feedback", 10,
                                                                    std::bind(&Convertor::brake_feedback_cb, this, std_ph::_1));
        gear_feedback_sub_ = create_subscription<autoware_platform_msgs::msg::GearFeedback>("as/steering_feedback", 10, 
                                                                    std::bind(&Convertor::gear_feedback_cb, this, std_ph::_1));
        steering_wheel_sub_ = create_subscription<autoware_platform_msgs::msg::SteeringFeedback>("as/steering_feedback", 10,
                                                                    std::bind(&Convertor::steering_feedback_cb, this, std_ph::_1));
                                                                                                                                                                                                                                                                                                                                                                                                                       
        callback_from_ssc_feedbacks(velocity_feedback_, curvature_feedback_, throttle_feedback_, brake_feedback_, gear_feedback_, steering_feedback_);
        callback_from_twist_feedbacks(velocity_feedback_,curvature_feedback_,steering_feedback_);

        // Setup publishers
        vehicle_status_pub_ = create_publisher<autoware_msgs::msg::VehicleStatus>("vehicle_status",10);
        current_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("vehicle/twist",10);

        // These are published in a timed callback
        // Using 30Hz as callback rate - TO DO: Update this to use status_pub_rate
        status_pub_timer = create_timer(
        get_clock(),
        std::chrono::milliseconds(33.333333333333),    
        std::bind(&Convertor::publish_vehicle_status, this))



        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;
   }

    void Convertor::publish_vehicle_status(){
          if (have_vehicle_status_) {
            vehicle_status_pub_->publish(current_status_msg_);
            have_vehicle_status_ = false; // Reset the status flag to ensure we only publish new messages
        }
        if (have_twist_) {
            current_twist_pub_->publish(current_twist_msg_);
            have_twist_ = false; // Reset the status flag to ensure we only publish new messages
        }
    }

    void Convertor::callback_from_guidance_state(const carma_planning_msgs::msg::GuidanceState::UniqPtr msg){
            if (msg->state == cav_msgs::msg::GuidanceState::ENGAGED)
            {
                shift_to_park_ = false;
            }
            else if (msg->state == cav_msgs::msg::GuidanceState::ENTER_PARK)
            {
                shift_to_park_ = true;
            }
    }

    void Convertor::callback_from_vehicle_cmd(const autoware_msgs::msg::VehicleCmd::UniqPtr msg){
        command_time_ = this->now();
        vehicle_cmd_ = *msg;
        command_initialized_ = true;
    }

    void callback_from_engage(const std_msgs::msg::Bool::UniquePtr msg){
        engage_ = msg->data;
    }

    void callback_from_ssc_module_states(const automotive_navigation_msgs::msg::ModuleState::UniquePtr msg){
          if (msg->name.find("veh_controller") != std::string::npos)
        {
            module_states_ = *msg;  // *_veh_controller status is used for 'drive/steeringmode'
        }
    }

    void Convertor::velocity_accel_cb(const automotive_platform_msgs::msg::VelocityAccelCov::UniquePtr& msg_velocity){
        velocity_feedback_ = *msg_velocity;
    }

    void Convertor::curvature_feedback_cb(const automotive_platform_msgs::msg::CurvatureFeedback::UniquePtr& msg_curvature){
        curvature_feedback_ = *msg_curvature;
    }

    void Convertor::throttle_feedback_cb(const automotive_platform_msgs::msg::ThrottleFeedback::UniquePtr& msg_throttle){
        throttle_feedback_ = *msg_throttles;
    }

    void Convertor::brake_feedback_cb(const automotive_platform_msgs::msg::BrakeFeedback::UniquePtr& msg_brake){
        brake_feedback_ = *msg_brake;
    }

    void Convertor::gear_feedback_cb(const automotive_platform_msgs::msg::GearFeedback::UniquePtr& msg_gear){
        gear_feedback_ = *msg_gear;
    }

    void Convertor::steering_feedback_cb(const automotive_platform_msgs::msg::SteeringFeedback::UniquePtr& msg_steering_wheel){
        steering_feedback_ = *msg_steering_wheel;
    }

    void Convertor::callback_from_ssc_feedbacks(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                                const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                                const automotive_platform_msgs::msg::ThrottleFeedback& msg_throttle,
                                                const automotive_platform_msgs::msg::BrakeFeedback& msg_brake,
                                                const automotive_platform_msgs::msg::GearFeedback& msg_gear,
                                                const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel)
    {
        builtin_interfaces::msg::Time stamp = msg_velocity.header.stamp;
        // update adaptive gear ratio (avoiding zero divizion)
        adaptive_gear_ratio_ =
            std::max(1e-5, agr_coef_a_ + agr_coef_b_ * msg_velocity.velocity * msg_velocity.velocity - agr_coef_c_ * msg_steering_wheel.steering_wheel_angle);

        // current steering curvature
        double curvature = !use_adaptive_gear_ratio_ ?
                         (msg_curvature.curvature) :
                         std::tan(msg_steering_wheel.steering_wheel_angle/ adaptive_gear_ratio_) / wheel_base_;

        // Set current_velocity_ variable [m/s]
        current_velocity_ = msg_velocity.velocity;

        // vehicle_status (autoware_msgs::msg::VehicleStatus)
        autoware_msgs::msg::VehicleStatus vehicle_status;
        vehicle_status.header.frame_id = BASE_FRAME_ID;
        vehicle_status.header.stamp = stamp;

        // drive/steeringmode
        ehicle_status.drivemode = (module_states_.state == "active") ? autoware_msgs::msg::VehicleStatus::MODE_AUTO :
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

    void Convertor::callback_for_twist_update(const automotive_platform_msgs::msg::VelocityAccelCov& msg_velocity,
                                  const automotive_platform_msgs::msg::CurvatureFeedback& msg_curvature,
                                  const automotive_platform_msgs::msg::SteeringFeedback& msg_steering_wheel)
    {
        // current steering curvature
        double curvature = !use_adaptive_gear_ratio_ ?
                         (msg_curvature.curvature) :
                         std::tan(msg_steering_wheel.steering_wheel_angle/ adaptive_gear_ratio_) / wheel_base_;

        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = BASE_FRAME_ID;
        twist.header.stamp = msg_velocity->header.stamp;
        twist.twist.linear.x = msg_velocity->velocity;               // [m/s]
        twist.twist.angular.z = curvature * msg_velocity->velocity;  // [rad/s]
        current_twist_msg_ = twist;

        have_twist_ = true;
    }                                  



}