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

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

set -exo pipefail
declare -i false=0 true=1
BRANCH=develop
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -b|--branch)
                  BRANCH=$2
                  shift
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
            -ros1|--ros1_build)
                  dir=~/workspace_ros1
                  echo "Checkout ros1 dependencies"
                  build_ros1_pkgs="$true"
                  build_ros2_pkgs="$false"
                  shift
            ;;
            -ros2|--ros2_build)
                  dir=~/workspace_ros2
                  echo "Checkout ros2 dependencies"
                  build_ros1_pkgs="$false"
                  build_ros2_pkgs="$true"
                  shift
            ;;
      esac
done

cd ${dir}
git clone --depth 1 https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/src/CARMAMsgs --branch "$BRANCH"
git clone --depth 1 https://github.com/usdot-fhwa-stol/carma-utils.git ${dir}/src/CARMAUtils --branch "$BRANCH"

# Sparse checkout to only get the messages we need
if [[ "$BRANCH" == "develop" ]] || [[ "$BRANCH" == "master" ]]; then
      git clone --depth 1 --filter=blob:none --sparse https://github.com/usdot-fhwa-stol/autoware.ai.git ${dir}/src/autoware.ai --branch carma-"$BRANCH"
else
      git clone --depth 1 --filter=blob:none --sparse https://github.com/usdot-fhwa-stol/autoware.ai.git ${dir}/src/autoware.ai --branch "$BRANCH"
fi

cd ${dir}/src/autoware.ai
git sparse-checkout init --cone
git sparse-checkout set messages/autoware_msgs jsk_recognition/jsk_recognition_msgs
cd ${dir}

if [ $build_ros1_pkgs -eq 1 ]; then

    # Required to build the dbw_pacifica_msgs message set.
    sudo git clone https://github.com/NewEagleRaptor/raptor-dbw-ros.git ${dir}/src/raptor-dbw-ros --branch master
    cd ${dir}/src/raptor-dbw-ros
    sudo git reset --hard f50f91cd88ad27b2ce05bab1f8ff780931c41475
    cd ${dir}

    # Required for ford fusion drive by wire
    sudo git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ${dir}/src/dbw-mkz-ros --branch 1.2.4

    sudo git clone https://github.com/astuff/pacmod3.git ${dir}/src/pacmod3 --branch ros1_master
    cd ${dir}/src/pacmod3
    sudo git reset --hard 4e5e9cd5e821f4f19e31e10ba42f20449860b940
    cd ${dir}

    sudo git clone https://github.com/astuff/kvaser_interface.git ${dir}/src/kvaser_interface --branch ros1_master
    cd ${dir}/src/kvaser_interface
    sudo git reset --hard e2aa169e32577f2468993b89edf7a0f67d1e7f0e
    cd ${dir}

fi
