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
token=""

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
                token="$1"
                shift
                shift
                ;;
      esac
done

if [ $build_ros1_pkgs -eq 1 ]; then
    # ROS1 build and install
    cd ~/workspace_ros1
    echo "ROS1 build"
    source /home/carma/catkin/setup.bash
    source /opt/ros/noetic/install/setup.bash
    sudo apt-get update
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

    if [ -z $token ];
        then
            echo "No argument provided for token for ROS2 build, this script needs to be run with <TOKEN>"
            exit 1
    fi
    cd ~
    source /opt/ros/humble/install/setup.bash
    git clone https://$token@github.com/usdot-fhwa-stol/CARMASensitive.git --branch arc-199-humble-lexus-ssc-deb-files

    sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
    sudo apt-add-repository ppa:astuff/kvaser-linux
    sudo apt-get update

    # Prerequisite for kvaser-interface
    sudo apt install kvaser-canlib-dev kvaser-drivers-dkms

    # Install necessary dependencies for ssc-pm-lexus
    sudo apt-get install -y libas-common \
        ros-humble-automotive-autonomy-msgs \
        ros-humble-automotive-navigation-msgs \
        ros-humble-automotive-platform-msgs \
        ros-humble-pacmod3-msgs \
        ros-humble-pacmod3 \
        ros-humble-kvaser-interface \
        apt-utils

    cd CARMASensitive
    sudo dpkg -i ros-humble-ssc-joystick_2004.0.0-0jammy_amd64.deb
    sudo dpkg -i ros-humble-ssc-pm-lexus_1.1.0-0jammy_amd64.deb

    # Build ssc_interface_wrapper_ros2
    cd ~/workspace_ros2
    source /opt/ros/humble/setup.bash

    colcon build --packages-up-to ssc_interface_wrapper_ros2 --build-base ./build_ssc_interface_wrapper --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release

    # Get the exit code from the ROS2 build
    status=$?

    if [[ $status -ne 0 ]]; then
        echo "ssc interface wrapper ros2 build failed."
        exit $status
    fi

    exit #Success building ros2 pkgs
fi
