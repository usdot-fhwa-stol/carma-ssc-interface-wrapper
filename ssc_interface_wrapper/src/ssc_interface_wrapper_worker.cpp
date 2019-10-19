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
    else if(current_time - last_vehicle_status_time_ > ros::Duration(timeout) || controller_fault_) {
        return cav_msgs::DriverStatus::FAULT;
    }
    return cav_msgs::DriverStatus::OPERATIONAL;
}

void SSCInterfaceWrapperWorker::on_new_status_msg(const automotive_navigation_msgs::ModuleStateConstPtr& msg, const ros::Time& current_time)
{
	const std::string controller_tag("veh_controller");
	
    if(msg->name.length() >= controller_tag.length() 
		&& (0 == msg->name.compare (msg->name.length() - controller_tag.length(), controller_tag.length(), controller_tag)))
	{
        last_vehicle_status_time_ = current_time;
		update_control_status(msg);
	}
}

void SSCInterfaceWrapperWorker::update_control_status(const automotive_navigation_msgs::ModuleStateConstPtr& msg)
{
	if(msg->state.compare("fatal") == 0)
	{
		controller_fault_ = true;
	}
	else if (msg->state.compare("active") == 0 || msg->state.compare("engaged") == 0)
	{
		robotic_control_engaged_ = true;
	}

	// TODO will move the following logic into controller specific CAN driver
	//can_bus_timeout_ = msg -> user_can_timeout || msg -> brake_can_timeout   || msg -> steering_can_timeout
	//		                                   || msg -> vehicle_can_timeout || msg -> subsystem_can_timeout;
	can_bus_timeout_ = true;
}

int SSCInterfaceWrapperWorker::convert_shift_state_to_J2735(const pacmod_msgs::SystemRptIntConstPtr shift_state)
{
	switch(shift_state -> output)
	{
	case pacmod_msgs::SystemRptInt::SHIFT_PARK:
		return j2735_msgs::TransmissionState::PARK;
	case pacmod_msgs::SystemRptInt::SHIFT_REVERSE:
		return j2735_msgs::TransmissionState::REVERSEGEARS;
	case pacmod_msgs::SystemRptInt::SHIFT_NEUTRAL:
		return j2735_msgs::TransmissionState::NEUTRAL;
	case pacmod_msgs::SystemRptInt::SHIFT_FORWARD:
		return j2735_msgs::TransmissionState::FORWARDGEARS;
	default:
		return j2735_msgs::TransmissionState::UNAVAILABLE;
	}
}

bool SSCInterfaceWrapperWorker::is_engaged()
{
    return robotic_control_engaged_;
}
