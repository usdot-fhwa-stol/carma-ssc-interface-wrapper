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
#include <autoware_msgs/VehicleStatus.h>

class SSCInterfaceWrapperWorker
{

public:

    SSCInterfaceWrapperWorker() : robotic_control_engaged_(false) {}
    ~SSCInterfaceWrapperWorker() {}

    bool is_engaged();
    uint8_t get_driver_status(const ros::Time& current_time, double timeout);
    void on_new_status_msg(const autoware_msgs::VehicleStatusConstPtr& msg, const ros::Time& current_time);

private:
    
    bool robotic_control_engaged_;
    ros::Time last_vehicle_status_time_;
    
    void update_control_status(const autoware_msgs::VehicleStatusConstPtr& msg);

};
