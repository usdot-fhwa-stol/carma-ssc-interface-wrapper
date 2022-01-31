/*
 * Copyright (C) <SUB><year> LEIDOS.
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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include <rclcpp/rclcpp.hpp>

#include "ssc_interface_wrapper/ssc_interface_wrapper_worker.hpp"

TEST(TEST_ssc_interface_wrapper, testControlledStatusChangeOne)
{
    ssc_interface_wrapper::Worker worker;
    EXPECT_EQ(false, worker.is_engaged());
    automotive_navigation_msgs::msg::ModuleState msg;
    msg.name = "veh_controller";
    msg.state = "active";
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto clock = node->get_clock();
    worker.on_new_status_msg(msg, clock->now());
    EXPECT_EQ(true, worker.is_engaged());

}

TEST(TEST_ssc_interface_wrapper, testWrongControllerName)
{
    ssc_interface_wrapper::Worker worker;
    EXPECT_EQ(false, worker.is_engaged());
    automotive_navigation_msgs::msg::ModuleState msg;
    msg.name = "veh_controller1";
    msg.state = "active";
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto clock = node->get_clock();
    worker.on_new_status_msg(msg, clock->now());
    EXPECT_EQ(false, worker.is_engaged());

}

TEST(TEST_ssc_interface_wrapper, testControlledStatusChangeTwo)
{
    ssc_interface_wrapper::Worker worker;
    EXPECT_EQ(false, worker.is_engaged());
    automotive_navigation_msgs::msg::ModuleState msg;
    msg.name = "veh_controller";
    msg.state = "off";
    auto node = std::make_shared<rclcpp::Node>("test_node");
    auto clock = node->get_clock();
    worker.on_new_status_msg(msg, clock->now());
    EXPECT_EQ(false, worker.is_engaged());
}

TEST(Test_ssc_interface_wrapper, testAsync)
{
    auto node1 = std::make_shared<rclcpp::Node>("test_node_1");
    auto node2 = std::make_shared<rclcpp::Node>("test_node_2");

    rclcpp::executors::SingleThreadedExecutor executor1;
    executor1.add_node(node1);
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);

    auto spin_executor2 = [&executor2](){
        std::cout<<"Print from spin"<<std::endl;
        executor2.spin();
    };

    // Multiple single threaded executors in async?
    std::async execution_thread(spin_executor2);
    executor1.spin();
    execution_thread.join();

    rclcpp::Duration test;
    test.seconds();
    
}


int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);
    
    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 