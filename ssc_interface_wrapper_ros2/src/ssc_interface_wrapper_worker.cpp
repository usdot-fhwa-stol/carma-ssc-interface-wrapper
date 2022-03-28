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

#include "ssc_interface_wrapper/ssc_interface_wrapper_worker.hpp"

namespace ssc_interface_wrapper
{
    void Worker::check_driver_status(const rclcpp::Time& current_time, double timeout){
        
        rclcpp::Duration duration_since_last_update = current_time - last_vehicle_status_time_;

        if(duration_since_last_update.seconds() > timeout || latest_ssc_status_.compare("fatal") == 0 
        || (latest_ssc_status_.compare("failure") == 0 && latest_ssc_status_info_.compare("Operator Override") != 0))
        {
            throw std::invalid_argument("SSC driver error");
        }

    }

    void Worker::on_new_status_msg(const automotive_navigation_msgs::msg::ModuleState& msg, const rclcpp::Time& current_time)
    {
        static const std::string controller_tag{"veh_controller"};
        
        if(msg.name.length() >= controller_tag.length() 
            && (0 == msg.name.compare (msg.name.length() - controller_tag.length(), controller_tag.length(), controller_tag)))
            {
                last_vehicle_status_time_ = current_time;
                latest_ssc_status_ = msg.state;
                latest_ssc_status_info_ = msg.info;
            }
    }

    std::string Worker::get_current_ssc_state()
    {
        return this->latest_ssc_status_;
    }

    int Worker::convert_shift_state_to_J2735(const pacmod3_msgs::msg::SystemRptInt& shift_state)
    {
        switch(shift_state.output)
        {
        case pacmod3_msgs::msg::SystemRptInt::SHIFT_PARK:
            return j2735_v2x_msgs::msg::TransmissionState::PARK;
        case pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE:
            return j2735_v2x_msgs::msg::TransmissionState::REVERSEGEARS;
        case pacmod3_msgs::msg::SystemRptInt::SHIFT_NEUTRAL:
            return j2735_v2x_msgs::msg::TransmissionState::NEUTRAL;
        case pacmod3_msgs::msg::SystemRptInt::SHIFT_FORWARD:
            return j2735_v2x_msgs::msg::TransmissionState::FORWARDGEARS;
        default:
            return j2735_v2x_msgs::msg::TransmissionState::UNAVAILABLE;
        }
    }

    bool Worker::is_engaged()
    {
        return !latest_ssc_status_.compare("active");
    }

}//namespace ssc_interface_wrapper