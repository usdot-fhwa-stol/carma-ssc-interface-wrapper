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

if [[ "$BRANCH" = "develop" ]]; then
      git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch $BRANCH
      git clone https://github.com/usdot-fhwa-stol/carma-utils.git ${dir}/src/CARMAUtils --branch $BRANCH
else
      git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch foxy/develop
      git clone https://github.com/usdot-fhwa-stol/carma-utils.git ${dir}/src/CARMAUtils --branch foxy/develop
fi

# Required to build the dbw_pacifica_msgs message set.
git clone https://github.com/NewEagleRaptor/raptor-dbw-ros2.git ${dir}/src/raptor-dbw-ros2 --branch foxy 
cd ${dir}/src/raptor-dbw-ros2
git reset --hard 4ad958dd07bb9c7128dc75bc7397bc8f5be30a3c
cd ${dir}

# TO DO: Add ros2 version for Ford fusion drive by wire
# Required for ford fusion drive by wire
# git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ${dir}/src/dbw-mkz-ros --branch 1.2.4

#Pacmod3
git clone https://github.com/astuff/pacmod3.git ${dir}/src/pacmod3 --branch ros2_master
cd ${dir}/src/pacmod3
git reset --hard 159ef36f26726cf8d7f58e67add8c8319a67ae85
cd ${dir}

# kvaser
git clone https://github.com/astuff/kvaser_interface.git ${dir}/src/kvaser_interface --branch ros2_master
cd ${dir}/src/kvaser_interface
git reset --hard 89a6293ac0229c2c82a1fb33f72311e46f81c85b
cd ${dir}

# Install automotive_autonomy_msgs
git clone https://github.com/astuff/automotive_autonomy_msgs.git ${dir}/src/automotive_autonomy_msgs --branch master
cd ${dir}/src/automotive_autonomy_msgs 
git reset --hard 191dce1827023bef6d69b31e8c2514cf82bf10c5
cd ${dir}
