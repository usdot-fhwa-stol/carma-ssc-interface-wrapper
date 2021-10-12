#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
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

dir=~
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -d|--develop)
                  BRANCH=noetic/develop
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

if [[ "$BRANCH" = "noetic/develop" ]]; then
      git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch $BRANCH
      git clone https://github.com/usdot-fhwa-stol/carma-utils.git ${dir}/src/CARMAUtils --branch $BRANCH
else
      git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch noetic/develop
      git clone https://github.com/usdot-fhwa-stol/carma-utils.git ${dir}/src/CARMAUtils --branch noetic/develop
fi

# Required to build the dbw_pacifica_msgs message set.
git clone https://github.com/NewEagleRaptor/raptor-dbw-ros.git ${dir}/src/raptor-dbw-ros --branch master 
cd ${dir}/src/raptor-dbw-ros
git reset --hard f50f91cd88ad27b2ce05bab1f8ff780931c41475
cd ${dir}

# Required for ford fusion drive by wire
git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ${dir}/src/dbw-mkz-ros --branch 1.2.4

git clone https://github.com/astuff/pacmod3.git --branch ros1_master

git clone https://github.com/astuff/kvaser_interface.git --branch ros1_master

sudo apt-get install -y apt-utils
source /opt/autoware.ai/ros/install/setup.bash
cd ${dir}/
rosdep install --from-paths src --ignore-src -r -y
