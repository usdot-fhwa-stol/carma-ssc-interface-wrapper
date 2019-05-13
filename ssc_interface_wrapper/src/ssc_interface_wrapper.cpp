/*
 * Copyright (C) 2019 LEIDOS.
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
    status_.lon_controller = true;
    status_.lat_controller = true;

    // Initilize all subscribers
    vehicle_status_sub_ = nh_->subscribe("/vehicle_status", 5, &SSCInterfaceWrapper::vehicle_status_cb, this);

    // Initilize all publishers
    robot_status_pub_   = nh_->advertise<cav_msgs::RobotEnabled>("/controller/robot_status", 1);
    vehicle_engage_pub_ = nh_->advertise<std_msgs::Bool>("/vehicle/engage", 5);
    
    // initialize services
    enable_robotic_control_srv_ = nh_->advertiseService("/controller/enable_robotic", &SSCInterfaceWrapper::enable_robotic_control_cb, this);

    // Load parameters
    private_nh_->param<double>("controller_timeout", controller_timeout_, 1);

}

void SSCInterfaceWrapper::pre_spin()
{
    update_controller_health_status();
}

void SSCInterfaceWrapper::post_spin() {}

void SSCInterfaceWrapper::shutdown() {}

void SSCInterfaceWrapper::update_controller_health_status()
{
    status_.status = worker_.get_driver_status(ros::Time::now(), controller_timeout_);
}

void SSCInterfaceWrapper::publish_robot_status()
{
    robotic_status_msg_.robot_active = worker_.is_engaged();
    robot_status_pub_.publish(robotic_status_msg_);
}

void SSCInterfaceWrapper::vehicle_status_cb(const autoware_msgs::VehicleStatusConstPtr& msg)
{
    worker_.on_new_status_msg(msg, ros::Time::now());
    // robot_enabled field is true once the lower level controller is running
    robotic_status_msg_.robot_enabled = true;
    publish_robot_status();
}

bool SSCInterfaceWrapper::enable_robotic_control_cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticResponse &resp)
{
    std_msgs::Bool engage_cmd;
    engage_cmd.data = req.set == cav_srvs::SetEnableRoboticRequest::ENABLE;
    vehicle_engage_pub_.publish(engage_cmd);
    return true;
}
