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
    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    declare_vehicle_calibration_dir = DeclareLaunchArgument(name='vehicle_calibration_dir', default_value='ssc_vehicle_calibration_dir')

    ssc_package_name = LaunchConfiguration('ssc_package_name')
    declare_ssc_package_name = DeclareLaunchArgument(name = 'ssc_package_name', default_value = 'ssc_pm_lexus')

    # Launch wrapper
    ssc_interface_wrapper_pkg = get_package_share_directory('ssc_interface_wrapper_ros2')
    ssc_interface_wrapper_group = GroupAction(
        actions = [
            # Launch Wrapper
            set_remap.SetRemap('parsed_tx/global_rpt','pacmod/parsed_tx/global_rpt'),
            set_remap.SetRemap('parsed_tx/steer_rpt', 'pacmod/parsed_tx/steer_rpt'),
            set_remap.SetRemap('parsed_tx/brake_rpt', 'pacmod/parsed_tx/brake_rpt'),
            set_remap.SetRemap('parsed_tx/shift_rpt', 'pacmod/parsed_tx/shift_rpt'),
                        
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', ssc_interface_wrapper_pkg, '/launch','/ssc_interface_wrapper.launch.py']),
            )
        ]
    )

    #Launch drive by wire
    ssc_lexus_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/', vehicle_calibration_dir, '/ssc_pm_lexus/pacmod3_connection.launch.py']),
    )

    #Launch SSC
    carma_speed_steering_control_group = GroupAction(
        actions = [
            # Launch SSC
            set_remap.SetRemap('brake_cmd','pacmod/as_rx/brake_cmd'),
            set_remap.SetRemap('brake_report', 'pacmod/parsed_tx/brake_rpt'),
            set_remap.SetRemap('enable_cmd', 'pacmod/as_rx/enable'),
            set_remap.SetRemap('steering_cmd', 'pacmod/as_rx/steer_cmd'),
            set_remap.SetRemap('steering_report','pacmod/parsed_tx/steer_rpt'),
            set_remap.SetRemap('throttle_cmd', 'pacmod/as_rx/accel_cmd'),
            set_remap.SetRemap('throttle_report','pacmod/parsed_tx/accel_rpt'),
            set_remap.SetRemap('transmission_cmd','pacmod/as_rx/shift_cmd'),
            set_remap.SetRemap('transmission_report','pacmod/parsed_tx/shift_rpt'),
            set_remap.SetRemap('turn_signal_cmd','pacmod/as_rx/turn_cmd'),
            set_remap.SetRemap('speed','pacmod/parsed_tx/vehicle_speed_rpt'),
            set_remap.SetRemap('wheel_speed', 'pacmod/parsed_tx/wheel_speed_rpt'),
            set_remap.SetRemap('global_report', 'pacmod/parsed_tx/global_rpt'),

                        
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', ssc_interface_wrapper_pkg, '/launch','/carma_speed_steering_control.launch.py']),
                launch_arguments={
                    'ssc_package_name': ssc_package_name,
                    'vehicle_calibration_dir' : vehicle_calibration_dir
                    }.items()
            ),
        ]
    )

    #SSC Interface
    autoware_ssc_interface_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/', ssc_interface_wrapper_pkg,'/launch', '/remapped_ssc_interface.launch.py'])
    )

    return LaunchDescription([
        declare_vehicle_calibration_dir,
        declare_ssc_package_name,
        ssc_interface_wrapper_group,
        # ssc_lexus_node,
        carma_speed_steering_control_group,
        autoware_ssc_interface_node
    ])