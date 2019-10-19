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

#include <cav_msgs/DriverStatus.h>
#include <cav_msgs/RobotEnabled.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <automotive_navigation_msgs/ModuleState.h>
#include <j2735_msgs/TransmissionState.h>

class SSCInterfaceWrapperWorker
{

public:

    SSCInterfaceWrapperWorker() : robotic_control_engaged_(false), can_bus_timeout_(false), controller_fault_(false) {}
    ~SSCInterfaceWrapperWorker() {}

    bool is_engaged();
    uint8_t get_driver_status(const ros::Time& current_time, double timeout);
    void on_new_status_msg(const automotive_navigation_msgs::ModuleStateConstPtr& msg, const ros::Time& current_time);
    int convert_shift_state_to_J2735(const pacmod_msgs::SystemRptIntConstPtr shift_state);

private:
    
    bool robotic_control_engaged_;
    bool can_bus_timeout_;
    bool controller_fault_;
    ros::Time last_vehicle_status_time_;
    
    void update_control_status(const automotive_navigation_msgs::ModuleStateConstPtr& msg);

};
