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
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    use_adaptive_gear_ratio = LaunchConfiguration('use_adaptive_gear_ratio')
    declare_use_adaptive_gear_ratio = DeclareLaunchArgument(name ='use_adaptive_gear_ratio', default_value='true')

    command_timeout = LaunchConfiguration('command_timeout')
    declare_command_timeout = DeclareLaunchArgument(name ='command_timeout', default_value='1000')

    status_pub_rate = LaunchConfiguration('status_pub_rate')
    declare_status_pub_rate = DeclareLaunchArgument(name ='status_pub_rate', default_value='30.0')

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

    vehicle_config_param_file = LaunchConfiguration('vehicle_config_param_file')
    declare_vehicle_config_param_file_arg = DeclareLaunchArgument(
        name = 'vehicle_config_param_file',
        default_value = ["/opt/carma/vehicle/config/VehicleConfigParams.yaml"],
        description = "Path to file contain vehicle configuration parameters"
    )

    # Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('ssc_interface_wrapper_ros2'), 'config/converter_params.yaml')


    ssc_interface_group = GroupAction(
        actions = [
            # Launch Wrapper

            # Remap parameters to match vehicle config
            set_remap.SetRemap('wheel_base','vehicle_wheel_base'),
            set_remap.SetRemap('tire_radius','vehicle_tire_radius'),
            set_remap.SetRemap('acceleration_limit','vehicle_acceleration_limit'),
            set_remap.SetRemap('deceleration_limit','vehicle_deceleration_limit'),
            set_remap.SetRemap('max_curvature_rate','vehicle_max_curvature_rate'),
            # Remap topics used in CARMA Web UI to match other vehicles
            set_remap.SetRemap('/hardware_interface/as/brake_feedback', '/hardware_interface/brake_feedback'),
            set_remap.SetRemap('/hardware_interface/as/throttle_feedback', '/hardware_interface/throttle_feedback'),
            set_remap.SetRemap('/hardware_interface/as/steering_feedback', '/hardware_interface/steering_feedback'),
            

            # Launch conveter node as container
            ComposableNodeContainer(
                name='ssc_interface_wrapper_converter_container',
                namespace= GetCurrentNamespace(),
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='ssc_interface_wrapper_ros2',
                        plugin='ssc_interface_wrapper::Converter',
                        name='ssc_converter_node',
                        parameters=[ param_file_path,
                                    vehicle_config_param_file],
                    ),
                ]
            )

        ]
    )

    return LaunchDescription([
        declare_use_adaptive_gear_ratio,
        declare_command_timeout,
        declare_status_pub_rate,
        declare_wheel_base,
        declare_tire_base,
        declare_acceleration_limit,
        declare_deceleration_limit,
        declare_max_curvature_rate,
        declare_vehicle_config_param_file_arg,
        ssc_interface_group
    ])