#!/bin/bash

#  Copyright (C) 2021 LEIDOS.
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

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

set -exo pipefail

dir=~/workspace_ros2
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -d|--develop)
                  BRANCH=foxy/develop
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

# Checkout ROS2 dependencies
cd ${dir}
if [[ "$BRANCH" = "develop" ]]; then
      sudo git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ./src/CARMAMsgs --branch $BRANCH
      sudo git clone https://github.com/usdot-fhwa-stol/carma-utils.git ./src/CARMAUtils --branch $BRANCH
else
      sudo git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ./src/CARMAMsgs --branch foxy/develop
      sudo git clone https://github.com/usdot-fhwa-stol/carma-utils.git ./src/CARMAUtils --branch foxy/develop
fi

sudo git clone https://github.com/NewEagleRaptor/raptor-dbw-ros2.git ./src/raptor-dbw-ros2 --branch foxy 
cd ${dir}/src/raptor-dbw-ros2
sudo git reset --hard 4ad958dd07bb9c7128dc75bc7397bc8f5be30a3c
cd ${dir}

# TO DO: Add ros2 version for Ford fusion drive by wire
# Required for ford fusion drive by wire
# sudo git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ${dir}/src/dbw-mkz-ros --branch 1.2.4

#Pacmod3
sudo git clone https://github.com/astuff/pacmod3.git ./src/pacmod3_ros2 --branch ros2_master
cd ./src/pacmod3_ros2
sudo git reset --hard 159ef36f26726cf8d7f58e67add8c8319a67ae85
cd ${dir}

# kvaser
sudo git clone https://github.com/astuff/kvaser_interface.git ${dir}/src/kvaser_interface --branch ros2_master
cd ./src/kvaser_interface
sudo git reset --hard 89a6293ac0229c2c82a1fb33f72311e46f81c85b
cd ${dir}

# Install automotive_autonomy_msgs
sudo git clone https://github.com/astuff/automotive_autonomy_msgs.git ${dir}/src/automotive_autonomy_msgs --branch master
cd ./src/automotive_autonomy_msgs 
sudo git reset --hard 191dce1827023bef6d69b31e8c2514cf82bf10c5
cd ${dir}
