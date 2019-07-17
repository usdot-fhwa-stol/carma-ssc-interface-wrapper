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
#include <gtest/gtest.h>

TEST(SSCInterfaceWrapperWorkerTest, testControlStatusChangeOne)
{
    SSCInterfaceWrapperWorker worker;
    EXPECT_EQ(false, worker.is_engaged());
    pacmod_msgs::GlobalRpt msg;
    msg.enabled = true;
    pacmod_msgs::GlobalRptConstPtr status_msg(new pacmod_msgs::GlobalRpt(msg));
    ros::Time current_time(10, 10);
    worker.on_new_status_msg(status_msg, current_time);
    EXPECT_EQ(true, worker.is_engaged());
}


TEST(SSCInterfaceWrapperWorkerTest, testControlStatusChangeTwo)
{
    SSCInterfaceWrapperWorker worker;
    EXPECT_EQ(false, worker.is_engaged());
    pacmod_msgs::GlobalRpt msg;
    msg.enabled = false;
    pacmod_msgs::GlobalRptConstPtr status_msg(new pacmod_msgs::GlobalRpt(msg));
    ros::Time current_time(10, 10);
    worker.on_new_status_msg(status_msg, current_time);
    EXPECT_EQ(false, worker.is_engaged());
}

TEST(SSCInterfaceWrapperWorkerTest, testGetDriverStatusOne)
{
    SSCInterfaceWrapperWorker worker;
    ros::Time current_time(10, 10);
    EXPECT_EQ(cav_msgs::DriverStatus::OFF, worker.get_driver_status(current_time, 2));
}


TEST(SSCInterfaceWrapperWorkerTest, testGetDriverStatusTwo)
{
    SSCInterfaceWrapperWorker worker;
    pacmod_msgs::GlobalRpt msg;
    msg.header.stamp.sec = 10;
    msg.header.stamp.nsec = 10;
    pacmod_msgs::GlobalRptConstPtr status_msg(new pacmod_msgs::GlobalRpt(msg));
    ros::Time current_time1(10, 20);
    worker.on_new_status_msg(status_msg, current_time1);
    ros::Time current_time2(11, 20);
    EXPECT_EQ(cav_msgs::DriverStatus::OPERATIONAL, worker.get_driver_status(current_time2, 2));
}


TEST(SSCInterfaceWrapperWorkerTest, testGetDriverStatusThree)
{
    SSCInterfaceWrapperWorker worker;
    pacmod_msgs::GlobalRpt msg;
    msg.header.stamp.sec = 10;
    msg.header.stamp.nsec = 10;
    pacmod_msgs::GlobalRptConstPtr status_msg(new pacmod_msgs::GlobalRpt(msg));
    ros::Time current_time1(10, 20);
    worker.on_new_status_msg(status_msg, current_time1);
    ros::Time current_time2(12, 30);
    EXPECT_EQ(cav_msgs::DriverStatus::FAULT, worker.get_driver_status(current_time2, 2));
}

TEST(SSCInterfaceWrapperWorkerTest, testGetDriverStatusFour)
{
    SSCInterfaceWrapperWorker worker;
    pacmod_msgs::GlobalRpt msg;
    msg.header.stamp.sec = 10;
    msg.header.stamp.nsec = 10;
    msg.fault_active = true;
    pacmod_msgs::GlobalRptConstPtr status_msg(new pacmod_msgs::GlobalRpt(msg));
    ros::Time current_time1(10, 20);
    worker.on_new_status_msg(status_msg, current_time1);
    ros::Time current_time2(11, 20);
    EXPECT_EQ(cav_msgs::DriverStatus::FAULT, worker.get_driver_status(current_time2, 2));
}

TEST(SSCInterfaceWrapperWorkerTest, testGetDriverStatusFive)
{
    SSCInterfaceWrapperWorker worker;
    pacmod_msgs::GlobalRpt msg;
    msg.header.stamp.sec = 10;
    msg.header.stamp.nsec = 10;
    msg.vehicle_can_timeout = true;
    pacmod_msgs::GlobalRptConstPtr status_msg(new pacmod_msgs::GlobalRpt(msg));
    ros::Time current_time1(10, 20);
    worker.on_new_status_msg(status_msg, current_time1);
    ros::Time current_time2(11, 20);
    EXPECT_EQ(cav_msgs::DriverStatus::FAULT, worker.get_driver_status(current_time2, 2));
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
