# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    vehicle_calibration_dir = LaunchConfiguration("vehicle_calibration_dir")
    declare_vehicle_calibration_dir = DeclareLaunchArgument(name="vehicle_calibration_dir", default_value='ssc_vehicle_calibration_dir')
    
    ssc_package_name = LaunchConfiguration('ssc_package_name')
    declare_ssc_package_name = DeclareLaunchArgument(name = 'ssc_package_name', default_value = 'ssc_pm_lexus')

    return LaunchDescription([
        declare_vehicle_calibration_dir,
        declare_ssc_package_name,
        GroupAction(
            actions = [
                SetEnvironmentVariable('RLM_LICENSE', PathJoinSubstitution([vehicle_calibration_dir, '/' , ssc_package_name ,'/as_licenses'])),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare(ssc_package_name),
                            'launch', 
                            'speed_steering_control.launch.xml'
                        ])
                    ])
                )
            ]
        )
    ])