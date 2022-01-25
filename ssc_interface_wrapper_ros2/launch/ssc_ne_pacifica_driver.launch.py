# # Copyright (C) <SUB><year> LEIDOS.
# #
# # Licensed under the Apache License, Version 2.0 (the "License"); you may not
# # use this file except in compliance with the License. You may obtain a copy of
# # the License at
# #
# #   http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# # WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# # License for the specific language governing permissions and limitations under
# # the License.

# from ament_index_python import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

# import os

# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.actions import GroupAction
# from launch_ros.actions import set_remap

# '''
# This file is used to launch the CARMA compatible controller driver for the pacmod using the ssc interface.
# '''

# def generate_launch_description():

#     # Get parameter file path
#     ssc_param_file_path = os.path.join(
#         get_package_share_directory('ssc_ne_pacifica'), 'config/parameters.yaml')

    

    
#     ssc_interface_wrapper_pkg = get_package_share_directory('ssc_interface_wrapper')
#     ssc_ne_pacifica_pkg = get_package_share_directory('ssc_ne_pacifica')
#     ssc_interface_wrapper_group = GroupAction(
#         actions = [
#             # Launch Wrapper
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(['/', ssc_interface_wrapper_pkg, '/launch','/ssc_interface_wrapper.launch.py']),
#             ),

#             #set_remap.SetRemap('parsed_tx/global_rpt','pacmod/parsed_tx/global_rpt')

#             # Drive by wire
#             IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource(['/', ssc_ne_pacifica_pkg, '/launch','/ne_dbw_connection.launch.py']),
#             )
#         ]
#     )

    


#     return LaunchDescription([
#         ssc_interface_wrapper_group
#     ])
