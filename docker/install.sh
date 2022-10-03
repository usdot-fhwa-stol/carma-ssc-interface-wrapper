#!/bin/bash

#  Copyright (C) 2022 LEIDOS.
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

declare -i false=0 true=1

# This script installs the ssc_pm_lexus package using the access id and secret key as arguments
# These tokens are required to run the script
access_id=""
secret_key=""

while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -ros1|--ros1_build)
                  dir=~/workspace_ros1
                  echo "Install and build ros1 packages"
                  build_ros1_pkgs="$true"
                  build_ros2_pkgs="$false"
                  shift
                ;;
            -ros2|--ros2_build)
                  dir=~/workspace_ros2
                  echo "Install and build ros2 packages"
                  build_ros1_pkgs="$false"
                  build_ros2_pkgs="$true"
                  shift
                ;;
            *) ##Arguments for ssc_pm_lexus
                access_id=${arg}
                secret_key="$2"
                shift
                shift
                ;;
      esac
done

# Check if valid arguments are passed
if [ -z $access_id ];
    then 
        echo "No argument provided for access_id, this script needs to be run with <ACCESS_ID> <SECRET_KEY>"
        exit 1
fi

if [ -z $secret_key ];
    then 
        echo "No argument provided for secret_key, this script needs to be run with <ACCESS_ID> <SECRET_KEY>"
        exit 1
fi

if [ $build_ros1_pkgs -eq 1 ]; then
    # ROS1 build and install
    cd ~/workspace_ros1
    echo "ROS1 build"
    source /home/carma/catkin/setup.bash
    source /opt/autoware.ai/ros/install/setup.bash
    sudo apt-get install -y apt-utils

    rosdep install --from-paths src --ignore-src -r -y
    sudo apt-get install ros-noetic-pacmod-msgs

    sudo apt-get install python3-catkin-pkg
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/carma/install
    chmod -R ugo+x /opt/carma/install
    unset ROS_LANG_DISABLE

    # Get the exit code from the ROS1 build so we can skip the ROS2 build if the ROS1 build failed
    status=$?

    if [[ $status -ne 0 ]]; then
        echo "ssc interface wrapper ros1 build failed."
        exit $status
    fi

    exit #Success building ros1 pkgs

elif [ $build_ros2_pkgs -eq 1 ]; then

    cd ~/workspace_ros2
    source /opt/autoware.ai/ros/install_ros2/setup.bash
    sudo apt-get update
    sudo apt-get install -y apt-utils
    sudo apt-get install ros-foxy-pacmod-msgs
    sudo apt-get install ros-foxy-pacmod3-msgs
    sudo apt-get install ros-foxy-pacmod3
    sudo apt-get install ros-foxy-kvaser-interface

    colcon build --packages-up-to ssc_interface_wrapper_ros2 pacmod3 kvaser_interface --build-base ./build_ssc_interface_wrapper --install-base /opt/carma/install_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
    
    # Get the exit code from the ROS2 build
    status=$?

    if [[ $status -ne 0 ]]; then
        echo "ssc interface wrapper ros2 build failed."
        exit $status
    fi

    exit #Success building ros2 pkgs
fi

cd ~
source /opt/autoware.ai/ros/install_ros2/setup.bash
sudo apt-get update
sudo apt-get -qq install apt-transport-s3

sudo sh -c 'echo "AccessKeyId = '$access_id'" > /etc/apt/s3auth.conf'
sudo sh -c 'echo "SecretAccessKey = '$secret_key'" >> /etc/apt/s3auth.conf'
sudo sh -c 'echo "Token = \"\"" >> /etc/apt/s3auth.conf'
sudo sh -c 'echo "Region = \"us-east-1\"" >> /etc/apt/s3auth.conf'

sudo sh -c 'echo "deb [trusted=yes] s3://autonomoustuff-ssc $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-ssc.list'
sudo apt-get update
sudo apt-get -y install ros-foxy-ssc-pm-lexus ros-foxy-ssc-joystick && exit 0 || echo "Installation failed for ssc_pm_lexus check access_key and secret_id" && exit 1
