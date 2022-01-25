/*
 * Copyright (C) 2021 LEIDOS.
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <automotive_navigation_msgs/msg/module_state.hpp>
#include <pacmod_msgs/msg/system_rpt_int.hpp>
#include <j2735_v2x_msgs/msg/transmission_state.hpp>
#include <carma_driver_msgs/msg/driver_status.hpp>


namespace ssc_interface_wrapper
{
    class Worker
    {
    public:
        Worker() : latest_ssc_status_("not_ready") {}

        bool is_engaged();
        void check_driver_status(const rclcpp::Time& current_time, double timeout);
        void on_new_status_msg(const automotive_navigation_msgs::msg::ModuleState& msg, const rclcpp::Time& current_time);
        int convert_shift_state_to_J2735(const pacmod_msgs::msg::SystemRptInt& shift_state);
        std::string get_current_ssc_state();

    private:
        std::string latest_ssc_status_info_;
        std::string latest_ssc_status_;
        rclcpp::Time last_vehicle_status_time_;

    };
}