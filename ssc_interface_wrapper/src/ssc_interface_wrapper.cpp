/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "ssc_interface_wrapper.h"

SSCInterfaceWrapper::SSCInterfaceWrapper(int argc, char **argv, const std::string &name) : DriverWrapper (argc, argv, name) {}

SSCInterfaceWrapper::~SSCInterfaceWrapper() {}

void SSCInterfaceWrapper::initialize() {

    // Set driver type
    status_.controller = true;
    status_.can = true;

    // Initialize all subscribers
    ssc_state_sub_ = nh_->subscribe("module_states", 5, &SSCInterfaceWrapper::ssc_state_cb, this);
    steer_sub_ = nh_->subscribe("parsed_tx/steer_rpt", 5, &SSCInterfaceWrapper::steer_cb, this);
    brake_sub_ = nh_->subscribe("parsed_tx/brake_rpt", 5, &SSCInterfaceWrapper::brake_cb, this);
    shift_sub_ = nh_->subscribe("parsed_tx/shift_rpt", 5, &SSCInterfaceWrapper::shift_cb, this);
    hazard_light_sub_ = nh_->subscribe("hazard_light_status", 5, &SSCInterfaceWrapper::hazard_light_cb, this);

    // Initialize all publishers
    steering_wheel_angle_pub_ = nh_->advertise<std_msgs::Float64>("can/steering_wheel_angle", 1);
    brake_position_pub_ = nh_->advertise<std_msgs::Float64>("can/brake_position", 1);
    transmission_pub_ = nh_->advertise<j2735_msgs::TransmissionState>("can/transmission_state", 1);
    robot_status_pub_   = nh_->advertise<cav_msgs::RobotEnabled>("controller/robot_status", 1);
    vehicle_engage_pub_ = nh_->advertise<std_msgs::Bool>("vehicle/engage", 5);
    hazard_light_cmd_pub_ = nh_->advertise<pacmod_msgs::SystemCmdBool>("as_rx/hazard_lights", 5);
    
    // initialize services
    enable_robotic_control_srv_ = nh_->advertiseService("controller/enable_robotic", &SSCInterfaceWrapper::enable_robotic_control_cb, this);

    // Load parameters
    private_nh_->param<double>("controller_timeout", controller_timeout_, 1);

    // initialize member variables
    reengage_state_ = false;

}

void SSCInterfaceWrapper::pre_spin()
{
    update_controller_health_status();
    if(reengage_state_)
    {
        reengage_robotic_control();
    }
}

void SSCInterfaceWrapper::post_spin() {}

void SSCInterfaceWrapper::shutdown() {}

void SSCInterfaceWrapper::update_controller_health_status()
{
    status_.status = worker_.get_driver_status(ros::Time::now(), controller_timeout_);
}

void SSCInterfaceWrapper::reengage_robotic_control()
{
    if(!worker_.get_current_ssc_state().compare("ready")) {
        std_msgs::Bool engage_cmd;
        engage_cmd.data = true;
        vehicle_engage_pub_.publish(engage_cmd);
        reengage_state_ = false;
    }
}

void SSCInterfaceWrapper::hazard_light_cb(const std_msgs::Bool& status)
{
    pacmod_msgs::SystemCmdBool cmd;
    if (status.data == true)
    {   
        cmd.enable = true;
        cmd.command = true;
    }
    else
    {
        cmd.enable = false;
        cmd.command = false;
    }
    hazard_light_cmd_pub_.publish(cmd);
}

void SSCInterfaceWrapper::publish_robot_status()
{
    robotic_status_msg_.robot_active = worker_.is_engaged();
    robot_status_pub_.publish(robotic_status_msg_);
}

bool SSCInterfaceWrapper::enable_robotic_control_cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticResponse &resp)
{
    std_msgs::Bool engage_cmd;
    std::string current_state = worker_.get_current_ssc_state();
    if(!current_state.compare("ready")) {
        if(req.set == cav_srvs::SetEnableRoboticRequest::ENABLE) {
            engage_cmd.data = true;
            vehicle_engage_pub_.publish(engage_cmd);
        }
    } else if(!current_state.compare("active")) {
        if(req.set == cav_srvs::SetEnableRoboticRequest::DISABLE) {
            engage_cmd.data = false;
            vehicle_engage_pub_.publish(engage_cmd);
        }
    } else if(!current_state.compare("failure")) {
        engage_cmd.data = false;
        vehicle_engage_pub_.publish(engage_cmd);
        if(req.set == cav_srvs::SetEnableRoboticRequest::ENABLE) {
            // This flag is required because the SSC needs to be sent a negative engagement after operator override 
            // before it will accept a positive engagement.
            // In the lines above we send the negative engagement, but we need to wait for the status to change again
            // before we send the re-engagement. This flag tells us of the need to do that. 
            reengage_state_ = true;
        }
    }
    return true;
}

void SSCInterfaceWrapper::ssc_state_cb(const automotive_navigation_msgs::ModuleStateConstPtr& msg)
{
	worker_.on_new_status_msg(msg, ros::Time::now());
	// robot_enabled field is true once the lower level controller is running
	robotic_status_msg_.robot_enabled = true;
    publish_robot_status();
}

void SSCInterfaceWrapper::steer_cb(const pacmod_msgs::SystemRptFloatConstPtr& msg)
{
    std_msgs::Float64 steer_msg;
    steer_msg.data = msg -> output;
    steering_wheel_angle_pub_.publish(steer_msg);
}

void SSCInterfaceWrapper::brake_cb(const pacmod_msgs::SystemRptFloatConstPtr& msg)
{
	std_msgs::Float64 brake_msg;
	brake_msg.data = msg -> output;
	brake_position_pub_.publish(brake_msg);
}

void SSCInterfaceWrapper::shift_cb(const pacmod_msgs::SystemRptIntConstPtr& msg)
{
    j2735_msgs::TransmissionState transmission_msg;
    transmission_msg.transmission_state = worker_.convert_shift_state_to_J2735(msg);
    transmission_pub_.publish(transmission_msg);
}
