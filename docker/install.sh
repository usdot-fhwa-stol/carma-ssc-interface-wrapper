#!/bin/bash

#  Copyright (C) 2018-2022 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.


# ROS1 build and install
cd ~/workspace_ros1
echo "ROS1 build"
source /home/carma/catkin/setup.bash
source /opt/autoware.ai/ros/install/setup.bash
sudo apt-get install -y apt-utils
source /opt/autoware.ai/ros/install/setup.bash

rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-noetic-pacmod-msgs

sudo apt install python3-catkin-pkg
colcon build --packages-ignore ssc_interface_wrapper_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/carma/ros/install
chmod -R ugo+x /opt/carma/ros/install
unset ROS_LANG_DISABLE

# Get the exit code from the ROS1 build so we can skip the ROS2 build if the ROS1 build failed
status=$?

if [[ $status -ne 0 ]]; then
    echo "ssc interface wrapper ros1 build failed."
    exit $status
fi
