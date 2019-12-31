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

#ifndef GEAR_CHANGE_H
#define GEAR_CHANGE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <automotive_platform_msgs/GearCommand.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <automotive_platform_msgs/VelocityAccelCov.h>
#include <automotive_platform_msgs/CurvatureFeedback.h>
#include <automotive_platform_msgs/ThrottleFeedback.h>
#include <automotive_platform_msgs/BrakeFeedback.h>
#include <automotive_platform_msgs/GearFeedback.h>
#include <automotive_navigation_msgs/ModuleState.h>
#include <pacmod_msgs/WheelSpeedRpt.h>
#include <pacmod_msgs/SystemRptFloat.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>

static const std::string BASE_FRAME_ID = "base_link";

class GearChange
{
public:
  GearChange();
  ~GearChange();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber gear_feedback_sub_;
  ros::Subscriber speed_mode_sub_;
  ros::Subscriber gear_sub_;

  // publishers
  ros::Publisher speed_mode_pub_;
  ros::Publisher gear_pub_;

  // ros param
  double delay_time_; // [sec]

  // variables
  ros::Duration delay_duration_;
  ros::Time gear_change_stamp_;
  int current_gear_;
  int desired_gear_;
  int previous_desired_gear_;
  automotive_platform_msgs::GearCommand gear_msg_;
  bool gear_cmd_subscribed_;

  // callbacks
  void callbackFromSSCGearFeedback(const automotive_platform_msgs::GearFeedbackConstPtr& msg);
  void callbackFromGearSelect(const automotive_platform_msgs::GearCommandConstPtr& msg);
  void callbackFromSpeedCommand(const automotive_platform_msgs::SpeedModeConstPtr& msg);

};

#endif  // GEAR_CHANGE_H
