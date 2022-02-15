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

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

set -exo pipefail

dir=~/workspace_ros1
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -d|--develop)
                  BRANCH=develop
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

cd ${dir}
if [[ "$BRANCH" = "develop" ]]; then
      sudo git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ./src/CARMAMsgs --branch $BRANCH
      sudo git clone https://github.com/usdot-fhwa-stol/carma-utils.git ./src/CARMAUtils --branch $BRANCH
else
      sudo git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ./src/CARMAMsgs --branch develop
      sudo git clone https://github.com/usdot-fhwa-stol/carma-utils.git ./src/CARMAUtils --branch develop
fi

# Required to build the dbw_pacifica_msgs message set.
sudo git clone https://github.com/NewEagleRaptor/raptor-dbw-ros.git ./src/raptor-dbw-ros --branch master 
cd ./src/raptor-dbw-ros
sudo git reset --hard f50f91cd88ad27b2ce05bab1f8ff780931c41475
cd ${dir}

# Required for ford fusion drive by wire
sudo git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ./src/dbw-mkz-ros --branch 1.2.4

sudo git clone https://github.com/astuff/pacmod3.git ./src/pacmod3 --branch ros1_master
cd ./src/pacmod3
sudo git reset --hard 4e5e9cd5e821f4f19e31e10ba42f20449860b940
cd ${dir}

sudo git clone https://github.com/astuff/kvaser_interface.git ./src/kvaser_interface --branch ros1_master
cd ./src/kvaser_interface
sudo git reset --hard e2aa169e32577f2468993b89edf7a0f67d1e7f0e
cd ${dir}

