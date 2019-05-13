#pragma once

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

#include <driver_wrapper/driver_wrapper.h>
#include <autoware_msgs/VehicleStatus.h>
#include <cav_srvs/SetEnableRobotic.h>
#include <std_msgs/Bool.h>
#include "ssc_interface_wrapper_worker.h"

class SSCInterfaceWrapper : public cav::DriverWrapper
{

public:
    SSCInterfaceWrapper(int argc, char **argv, const std::string &name = "ssc_interface_wrapper");
    virtual ~SSCInterfaceWrapper();

private:
    
    // one subscriber and one publisher to SSC Interface node
    ros::Subscriber vehicle_status_sub_;
    ros::Publisher  vehicle_engage_pub_;
    
    // one publisher and one service to CARMA
    ros::Publisher  robot_status_pub_;
    ros::ServiceServer enable_robotic_control_srv_;

    // delegate logic implementation to worker class
    SSCInterfaceWrapperWorker worker_;
    
    // ROS parameter for determining controller timeout
    double controller_timeout_;

    // robotic status message as a local variable
    cav_msgs::RobotEnabled robotic_status_msg_;

    // callbacks to handle incoming vehicle status messages and enable robotic control request
    void vehicle_status_cb(const autoware_msgs::VehicleStatusConstPtr& msg);
    bool enable_robotic_control_cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticResponse &resp);

    // process the latest vehicle status message and publish as robot status topic
    void publish_robot_status();

    // check controller timeout and send disengage command if necessary
    void update_controller_health_status();

    //cav::DriverWrapper members
    virtual void initialize();
    virtual void pre_spin();
    virtual void post_spin();
    virtual void shutdown();



};
