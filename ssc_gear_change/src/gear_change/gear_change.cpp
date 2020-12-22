
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gear_change.h"

GearChange::GearChange() :
  nh_(),
  private_nh_("~"),
  current_gear_(automotive_platform_msgs::Gear::NONE),
  desired_gear_(automotive_platform_msgs::Gear::NONE),
  previous_desired_gear_(automotive_platform_msgs::Gear::NONE),
  gear_change_stamp_(ros::Time::now()),
  gear_cmd_subscribed_(false)
{
  // setup parameters
  private_nh_.param<double>("delay_time", delay_time_, 1.0);
  delay_duration_ = ros::Duration(delay_time_);

  // subscribers from ssc
  gear_feedback_sub_ =
    nh_.subscribe("as/gear_feedback", 10, &GearChange::callbackFromSSCGearFeedback, this);

  // subscribers from ssc_interface
  speed_mode_sub_ = nh_.subscribe("ssc_interface/arbitrated_speed_commands", 10, &GearChange::callbackFromSpeedCommand, this);
  gear_sub_ = nh_.subscribe("ssc_interface/gear_select", 10, &GearChange::callbackFromGearSelect, this);

  speed_mode_pub_ = nh_.advertise<automotive_platform_msgs::SpeedMode>("as/arbitrated_speed_commands", 1, true);
  gear_pub_ = nh_.advertise<automotive_platform_msgs::GearCommand>("as/gear_select", 1, true);
}

GearChange::~GearChange()
{
}

void GearChange::callbackFromSSCGearFeedback(const automotive_platform_msgs::GearFeedbackConstPtr& msg)
{
  current_gear_ = msg->current_gear.gear;
}


void GearChange::callbackFromGearSelect(const automotive_platform_msgs::GearCommandConstPtr& msg)
{
  gear_cmd_subscribed_ = true;
  gear_msg_ = *msg;
  desired_gear_ = msg->command.gear;
  if(desired_gear_ != previous_desired_gear_)
  {
    gear_change_stamp_ = ros::Time::now();
  }
  previous_desired_gear_ = desired_gear_;
}


void GearChange::callbackFromSpeedCommand(const automotive_platform_msgs::SpeedModeConstPtr& msg)
{
  if(!gear_cmd_subscribed_)
    return;

  bool wait_for_brake = ros::Time::now() - gear_change_stamp_ < delay_duration_;
  bool is_stopping_gear = (current_gear_ == automotive_platform_msgs::Gear::NONE
                           || current_gear_ == automotive_platform_msgs::Gear::PARK
                           || current_gear_ == automotive_platform_msgs::Gear::NEUTRAL);
  bool is_gear_desired_value = (current_gear_ == desired_gear_);

  // speed command
  automotive_platform_msgs::SpeedMode speed_mode = *msg;
  if(is_stopping_gear || wait_for_brake || !is_gear_desired_value)
  {
    speed_mode.speed = 0;
  }

  automotive_platform_msgs::GearCommand gear_cmd = gear_msg_;
  if( wait_for_brake )
  {
     // gear command
     gear_cmd.command.gear = current_gear_;
  }

  // publish
  speed_mode_pub_.publish(speed_mode);
  gear_pub_.publish(gear_cmd);

  ROS_INFO_STREAM("CurrentGear: " << (int)current_gear_ << ", "
                                  << "DesiredGear: " << (int)desired_gear_ << ", "
                                  << "wait_for_brake: " << wait_for_brake << ", "
                                  << "SpeedCommand: " << speed_mode.speed);
}
