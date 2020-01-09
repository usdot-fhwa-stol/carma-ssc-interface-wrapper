#!/bin/bash

#  Copyright (C) 2018-2020 LEIDOS.
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

set -ex

dir=~
if [[ -n ${1} ]]; then
      dir=${1}
fi

git clone https://github.com/usdot-fhwa-stol/CARMAMsgs.git ${dir}/src/CARMAMsgs --branch develop --depth 1
git clone https://github.com/usdot-fhwa-stol/CARMAUtils.git ${dir}/src/CARMAUtils --branch develop --depth 1
# Required to build the dbw_pacifica_msgs message set.
git clone https://github.com/NewEagleRaptor/pacifica-dbw-ros.git ${dir}/src/pacifica-dbw-ros --branch master --depth 1

#checkout working commit of third party repo
cd pacifica-dbw-ros
git checkout 6a3266dd72da2a378c27aedb2be65ae847c46b30
cd ..