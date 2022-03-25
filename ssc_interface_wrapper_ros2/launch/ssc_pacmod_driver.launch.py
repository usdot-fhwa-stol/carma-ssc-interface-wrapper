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

from launch_ros.substitutions import FindPackageShare
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
    declare_vehicle_calibration_dir = DeclareLaunchArgument(name='vehicle_calibration_dir', default_value='/opt/carma/vehicle/calibration')

    ssc_package_name = LaunchConfiguration('ssc_package_name')
    declare_ssc_package_name = DeclareLaunchArgument(name = 'ssc_package_name', default_value = 'ssc_pm_lexus')

    use_adaptive_gear_ratio = LaunchConfiguration('use_adaptive_gear_ratio')
    declare_use_adaptive_gear_ratio = DeclareLaunchArgument(name ='use_adaptive_gear_ratio', default_value='true')

    wheel_base = LaunchConfiguration('wheel_base')
    declare_wheel_base = DeclareLaunchArgument(name='wheel_base', default_value='2.79')

    tire_radius = LaunchConfiguration('tire_radius')
    declare_tire_base = DeclareLaunchArgument(name='tire_radius', default_value='0.39')

    acceleration_limit = LaunchConfiguration('acceleration_limit')
    declare_acceleration_limit =DeclareLaunchArgument(name='acceleration_limit', default_value='3.0')
    
    deceleration_limit = LaunchConfiguration('deceleration_limit')
    declare_deceleration_limit =DeclareLaunchArgument(name='deceleration_limit', default_value='3.0')

    max_curvature_rate = LaunchConfiguration('max_curvature_rate')
    declare_max_curvature_rate = DeclareLaunchArgument(name='max_curvature_rate', default_value='0.15')

    # Launch wrapper
    ssc_interface_wrapper_pkg = FindPackageShare('ssc_interface_wrapper_ros2')
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
        PythonLaunchDescriptionSource([vehicle_calibration_dir, '/ssc_pm_lexus/pacmod3_connection.launch.py']),
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
                PythonLaunchDescriptionSource([ssc_interface_wrapper_pkg, '/launch','/carma_speed_steering_control.launch.py']),
                launch_arguments={
                    'ssc_package_name': ssc_package_name,
                    'vehicle_calibration_dir' : vehicle_calibration_dir
                    }.items()
            ),
        ]
    )

    #SSC Interface
    autoware_ssc_interface_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ssc_interface_wrapper_pkg,'/launch', '/remapped_ssc_interface.launch.py']),
        launch_arguments={
                'use_adaptive_gear_ratio':use_adaptive_gear_ratio, 
                'wheel_base' : wheel_base,
                'tire_radius' : tire_radius,
                'acceleration_limit' : acceleration_limit,
                'deceleration_limit' : deceleration_limit,
                'max_curvature_rate' : max_curvature_rate
        }.items()
    )

    return LaunchDescription([
        declare_vehicle_calibration_dir,
        declare_ssc_package_name,
        declare_use_adaptive_gear_ratio,
        declare_wheel_base,
        declare_tire_base,
        declare_acceleration_limit,
        declare_deceleration_limit,
        declare_max_curvature_rate,
        carma_speed_steering_control_group,
        ssc_interface_wrapper_group,
        ssc_lexus_node,
        autoware_ssc_interface_node
    ])