# Copyright (C) 2022 LEIDOS.
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

def generate_launch_description():

    use_adaptive_gear_ratio = LaunchConfiguration('use_adaptive_gear_ratio')
    declare_use_adaptive_gear_ratio = DeclareLaunchArgument(name ='use_adaptive_gear_ratio', default_value='true')

    command_timeout = LaunchConfiguration('command_timeout')
    declare_command_timeout = DeclareLaunchArgument(name ='command_timeout', default_value='1000')

    loop_rate = LaunchConfiguration('loop_rate')
    declare_loop_rate = DeclareLaunchArgument(name ='loop_rate', default_value='30.0')

    ssc_interface_group = GroupAction(
        actions = [
            # Launch Wrapper
            set_remap.SetRemap('as/arbitrated_speed_commands', 'arbitrated_speed_commands'),
            set_remap.SetRemap('as/arbitrated_steering_commands', 'arbitrated_steering_commands'),
            set_remap.SetRemap('as/brake_feedback', 'brake_feedback'),
            set_remap.SetRemap('as/steering_feedback', 'steering_feedback'),
            set_remap.SetRemap('as/curvature_feedback', 'curvature_feedback'),
            set_remap.SetRemap('as/gear_feedback', 'gear_feedback'),
            set_remap.SetRemap('as/gear_select', 'gear_select'),
            set_remap.SetRemap('as/module_states','module_states'),
            set_remap.SetRemap('as/throttle_feedback','throttle_feedback'),
            set_remap.SetRemap('as/turn_signal_command', 'turn_signal_command'),
            set_remap.SetRemap('as/velocity_accel','velocity_accel'),
            set_remap.SetRemap('as/velocity_accel_cov', 'velocity_accel_cov'),

            set_remap.SetRemap('wheel_base','/vehicle_wheel_base'),
            set_remap.SetRemap('tire_radius','/vehicle_tire_radius'),
            set_remap.SetRemap('acceleration_limit','/vehicle_acceleration_limit'),
            set_remap.SetRemap('deceleration_limit','/vehicle_deceleration_limit'),
            set_remap.SetRemap('max_curvature_rate','/vehicle_max_curvature_rate'),
            set_remap.SetRemap('max_curvature_rate','/vehicle_max_curvature_rate'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('ssc_interface'), '/launch','/ssc_interface.launch.py']),
                launch_arguments={'use_adaptive_gear_ratio':use_adaptive_gear_ratio, 'command_timeout': command_timeout, 'loop_rate': loop_rate}.items()
            )

        ]
    )

    return LaunchDescription([
        declare_use_adaptive_gear_ratio,
        declare_command_timeout,
        declare_loop_rate,
        ssc_interface_group
    ])