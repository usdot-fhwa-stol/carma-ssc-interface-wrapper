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

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch.substitutions import TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import GroupAction
from launch.actions import SetEnvironmentVariable

from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import set_remap

def generate_launch_description():

    vehicle_calibration_dir = LaunchConfiguration('vehicle_calibration_dir')
    declare_vehicle_calibration_dir = DeclareLaunchArgument(name='vehicle_calibration_dir', default_value= '/opt/carma/vehicle/calibration')
    
    ssc_package_name = LaunchConfiguration('ssc_package_name')
    declare_ssc_package_name = DeclareLaunchArgument(name = 'ssc_package_name', default_value = 'ssc_pm_lexus')

    return LaunchDescription([
        declare_vehicle_calibration_dir,
        declare_ssc_package_name,
        GroupAction(
            actions = [

                # NOTE: In past versions of the system the RLM_LICENSE env variable was set here however it has been moved to the config due to this issue https://github.com/usdot-fhwa-stol/carma-platform/issues/2002

                set_remap.SetRemap('/hardware_interface/as/speed_model/get_state', '/hardware_interface/as/speed_model/false_get_state'),
                set_remap.SetRemap('/hardware_interface/as/speed_model/change_state', '/hardware_interface/as/speed_model/false_change_state'),

                set_remap.SetRemap('/hardware_interface/as/steering_model/get_state', '/hardware_interface/as/steering_model/false_get_state'),
                set_remap.SetRemap('/hardware_interface/as/steering_model/change_state', '/hardware_interface/as/steering_model/false_change_state'),
                
                set_remap.SetRemap('/hardware_interface/as/veh_controller/get_state', '/hardware_interface/as/veh_controller/false_get_state'),
                set_remap.SetRemap('/hardware_interface/as/veh_controller/change_state', '/hardware_interface/as/veh_controller/false_change_state'),

                set_remap.SetRemap('/hardware_interface/as/veh_interface/get_state', '/hardware_interface/as/veh_interface/false_get_state'),
                set_remap.SetRemap('/hardware_interface/as/veh_interface/change_state', '/hardware_interface/as/veh_interface/false_change_state'),

            
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource([
                        PathJoinSubstitution([ThisLaunchFileDir(),'ssc_pm_lexus.launch.xml'])
                    ]),
                    launch_arguments = {
                        'json_dir' : PathJoinSubstitution([vehicle_calibration_dir,ssc_package_name, 'json']),
                        }.items()
                ),
            ]
        )
    ])