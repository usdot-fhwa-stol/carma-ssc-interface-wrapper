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

#include "ssc_interface_wrapper_worker.h"

uint8_t SSCInterfaceWrapperWorker::get_driver_status(const ros::Time& current_time, double timeout)
{
    if(last_vehicle_status_time_.isZero())
    {
        return cav_msgs::DriverStatus::OFF;
    }
    else if(current_time - last_vehicle_status_time_ > ros::Duration(timeout)) {
        return cav_msgs::DriverStatus::FAULT;
    }
    return cav_msgs::DriverStatus::OPERATIONAL;
}

void SSCInterfaceWrapperWorker::on_new_status_msg(const autoware_msgs::VehicleStatusConstPtr& msg, const ros::Time& current_time)
{
    last_vehicle_status_time_ = current_time;
    update_control_status(msg);
}

void SSCInterfaceWrapperWorker::update_control_status(const autoware_msgs::VehicleStatusConstPtr& msg)
{
    robotic_control_engaged_ = msg->drivemode || msg->steeringmode;
}

bool SSCInterfaceWrapperWorker::is_engaged()
{
    return robotic_control_engaged_;
}
