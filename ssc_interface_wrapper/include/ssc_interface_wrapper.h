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
#include <cav_srvs/SetEnableRobotic.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <j2735_msgs/TransmissionState.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <automotive_navigation_msgs/ModuleState.h>
#include "ssc_interface_wrapper_worker.h"

class SSCInterfaceWrapper : public cav::DriverWrapper
{

public:
    SSCInterfaceWrapper(int argc, char **argv, const std::string &name = "ssc_interface_wrapper");
    virtual ~SSCInterfaceWrapper();

private:
    
    // one subscribers for ssc status
    ros::Subscriber ssc_state_sub_;

    // subscribers for reading CAN data
    ros::Subscriber steer_sub_;
    ros::Subscriber brake_sub_;
    ros::Subscriber shift_sub_;

    // publishers for sending CAN data to CARMA
    ros::Publisher steering_wheel_angle_pub_;
    ros::Publisher brake_position_pub_;
    ros::Publisher transmission_pub_;

    // one publisher for SSC Interface node
    ros::Publisher  vehicle_engage_pub_;
    
    // one publisher and one service for controller status to CARMA platform
    ros::Publisher  robot_status_pub_;
    ros::ServiceServer enable_robotic_control_srv_;

    // delegate logic implementation to worker class
    SSCInterfaceWrapperWorker worker_;
    
    // ROS parameter for determining controller timeout
    double controller_timeout_;

    // bool flag indicates the wrapper is in the reengage mode
    bool reengage_state_;

    // robotic status message as a local variable
    cav_msgs::RobotEnabled robotic_status_msg_;

    // service callback to engage robotic control
    bool enable_robotic_control_cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticResponse &resp);

    // callback functions to handle CAN messages from PACMOD driver
    void ssc_state_cb(const automotive_navigation_msgs::ModuleStateConstPtr& msg);
    void steer_cb(const pacmod_msgs::SystemRptFloatConstPtr& msg);
    void brake_cb(const pacmod_msgs::SystemRptFloatConstPtr& msg);
    void shift_cb(const pacmod_msgs::SystemRptIntConstPtr& msg);

    // process the latest vehicle status message and publish as robot status topic
    void publish_robot_status();

    // check controller health status
    void update_controller_health_status();

    // reengage robotic control if the reengage_state_ flag is set
    void reengage_robotic_control();

    // cav::DriverWrapper members
    virtual void initialize();
    virtual void pre_spin();
    virtual void post_spin();
    virtual void shutdown();

};
