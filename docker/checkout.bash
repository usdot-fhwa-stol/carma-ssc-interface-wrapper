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
build_ros1_pkgs="$false"
build_ros2_pkgs="$false"
dir=~
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
                  dir_ros1=$dir/$2
                  echo "Checkout ros1 dependencies to specified folder"
                  build_ros1_pkgs="$true"
                  shift
            ;;
            -ros2|--ros2_build)
                  dir_ros2=$dir/$2
                  echo "Checkout ros2 dependencies to specified folder"
                  build_ros2_pkgs="$true"
                  shift
            ;;

      esac
done


cd ${dir}

# Clone the shared dependency only if it's not already present
if [ ! -d "${dir}/shared_deps/CARMAMsgs" ]; then
  git clone https://github.com/usdot-fhwa-stol/carma-msgs.git ${dir}/shared_deps/CARMAMsgs --branch $BRANCH
fi

if [ ! -d "${dir}/shared_deps/CARMAUtils" ]; then
  git clone https://github.com/usdot-fhwa-stol/carma-utils.git ${dir}/shared_deps/CARMAMsgs --branch arc-146-remove-ros1
fi

if [ $build_ros1_pkgs -eq 1 ]; then
    mkdir -p  ${dir_ros1}/src/
    # symlink the shared dependency
    ln -sf ${dir}/shared_deps/CARMAMsgs ${dir_ros1}/src/
    ln -sf ${dir}/shared_deps/CARMAUtils ${dir_ros1}/src/

    # Required to build the dbw_pacifica_msgs message set.
    sudo git clone https://github.com/NewEagleRaptor/raptor-dbw-ros.git ${dir_ros1}/src/raptor-dbw-ros --branch master
    cd ${dir_ros1}/src/raptor-dbw-ros
    sudo git reset --hard f50f91cd88ad27b2ce05bab1f8ff780931c41475
    cd ${dir_ros1}

    # Required for ford fusion drive by wire
    sudo git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ${dir_ros1}/src/dbw-mkz-ros --branch 1.2.4

    sudo git clone https://github.com/astuff/pacmod3.git ${dir_ros1}/src/pacmod3 --branch ros1_master
    cd ${dir_ros1}/src/pacmod3
    sudo git reset --hard 4e5e9cd5e821f4f19e31e10ba42f20449860b940
    cd ${dir_ros1}

    sudo git clone https://github.com/astuff/kvaser_interface.git ${dir_ros1}/src/kvaser_interface --branch ros1_master
    cd ${dir_ros1}/src/kvaser_interface
    sudo git reset --hard e2aa169e32577f2468993b89edf7a0f67d1e7f0e
    cd ${dir_ros1}
fi

if [ $build_ros2_pkgs -eq 1 ]; then
    # symlink the shared dependency
    mkdir -p  ${dir_ros2}/src/
    ln -sf ${dir}/shared_deps/CARMAMsgs ${dir_ros2}/src/
    ln -sf ${dir}/shared_deps/CARMAUtils ${dir_ros2}/src/

    sudo git clone https://github.com/NewEagleRaptor/raptor-dbw-ros2.git ${dir_ros2}/src/raptor-dbw-ros2 --branch foxy
    cd ${dir_ros2}/src/raptor-dbw-ros2
    sudo git reset --hard 4ad958dd07bb9c7128dc75bc7397bc8f5be30a3c
    cd ${dir_ros2}

    # TO DO: Add ros2 version for Ford fusion drive by wire
    # Required for ford fusion drive by wire
    # sudo git clone https://bitbucket.org/DataspeedInc/dbw_mkz_ros.git ${dir_ros2}/src/dbw-mkz-ros --branch 1.2.4

    #Pacmod3
    sudo git clone https://github.com/astuff/pacmod3.git ${dir_ros2}/src/pacmod3_ros2 --branch ros2_master
    cd ${dir_ros2}/src/pacmod3_ros2
    sudo git reset --hard 159ef36f26726cf8d7f58e67add8c8319a67ae85
    cd ${dir_ros2}

    # kvaser
    sudo git clone https://github.com/astuff/kvaser_interface.git ${dir_ros2}/src/kvaser_interface --branch ros2_master
    cd ${dir_ros2}/src/kvaser_interface
    sudo git reset --hard d7ea2fb82a1b61d0ce4c96d1422599f7ee6ed1b7
    cd ${dir_ros2}

    # Install automotive_autonomy_msgs
    sudo git clone https://github.com/astuff/automotive_autonomy_msgs.git ${dir_ros2}/src/automotive_autonomy_msgs --branch master
    cd ${dir_ros2}/src/automotive_autonomy_msgs
    sudo git reset --hard 191dce1827023bef6d69b31e8c2514cf82bf10c5
    cd ${dir_ros2}
fi
