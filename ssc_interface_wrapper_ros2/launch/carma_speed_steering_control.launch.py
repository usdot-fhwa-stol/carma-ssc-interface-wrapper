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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os

from launch.substitutions import TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    
    ssc_package_name = LaunchConfiguration('ssc_package_name')
    
    license_folder = TextSubstitution([vehicle_calibration_dir ,ssc_package_name,'as_licenses'])
    
    #Set environment variable
    env_var = SetEnvironmentVariable('RLM_LICENSE', license_folder)

    ssc_package_dir = get_package_share_directory(ssc_package_name)
    carma_speed_steering_control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/', ssc_package_dir, '/launch','/speed_steering_control.launch.xml']),
            
    )

    return LaunchDescription([
        env_var,
        carma_speed_steering_control_node
    ])