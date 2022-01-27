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

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import set_remap
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    ssc_package_name = LaunchConfiguration('ssc_package_name')
    declare_ssc_package_name = DeclareLaunchArgument(name = 'ssc_package_name', default_value='ssc_pm_lexus')

    param_dir = LaunchConfiguration('param_dir')
    declare_param_dir = DeclareLaunchArgument(name='param_dir', default_value= get_package_share_directory('ssc_pm_lexus'))

    license_folder = LaunchConfiguration('license_folder')
    declare_license_folder = DeclareLaunchArgument(name = 'license_folder', default_value= os.path.join(
        get_package_share_directory('ssc_pm_lexus'), 'as_licenses'))
    
    speed_model_json = LaunchConfiguration('speed_model_json')
    declare_speed_model_json = DeclareLaunchArgument(name = 'speed_model_json', default_value= os.path.join(
        get_package_share_directory('ssc_pm_lexus'), 'json/speed_model.json'))

    steering_model_json = LaunchConfiguration('steering_model_json')
    declare_steering_model_json = DeclareLaunchArgument(name = 'steering_model_json', default_value= os.path.join(
        get_package_share_directory('ssc_pm_lexus'), 'json/steering_model.json'))
    
    veh_controller_json = LaunchConfiguration('veh_controller_json')
    declare_veh_controller_json = DeclareLaunchArgument(name = 'veh_controller_json', default_value= os.path.join(
        get_package_share_directory('ssc_pm_lexus'), 'json/veh_controller.json'))

    veh_interface_json = LaunchConfiguration('veh_interface_json')
    declare_veh_interface_json = DeclareLaunchArgument(name = 'veh_interface_json', default_value= os.path.join(
        get_package_share_directory('ssc_pm_lexus'), 'json/veh_interface.json'))


    #Set environment variable
    env_var = SetEnvironmentVariable('RLM_LICENSE', license_folder)


    #TO DO: replace with generated module
    # carma_speed_steering_control_group = GroupAction(
    #     actions = [
            
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(['/', <package_name>, '/launch','<launch>']),
    #             launch_arguments={'speed_model_json':speed_model_json}.items(),
    #         ),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(['/', <package_name>, '/launch','<launch>']),
    #             launch_arguments={'steering_model_json':steering_model_json}.items(),
    #         ),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(['/', <package_name>, '/launch','<launch>']),
    #             launch_arguments={'steering_model_json':steering_model_json}.items(),
    #         ),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(['/', <package_name>, '/launch','<launch>']),
    #             launch_arguments={'steering_model_json':steering_model_json}.items(),
    #         ),
    #     ]
    # )


    return LaunchDescription([
        env_var,
        declare_ssc_package_name,
        declare_param_dir,
        declare_license_folder,
        declare_speed_model_json,
        declare_steering_model_json,
        declare_veh_controller_json,
        declare_veh_interface_json,
        # carma_speed_steering_control_group
    ])