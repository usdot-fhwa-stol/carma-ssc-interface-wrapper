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

FROM usdotfhwastoldev/autoware.ai:develop AS base_image

FROM base_image as source-code

# Install remaining package deps
RUN mkdir -p ~/workspace_ros1/src ~/workspace_ros2/src
COPY --chown=carma . /home/carma/workspace_ros1/src/ 
RUN sudo rm -rf ~/workspace_ros1 ~/workspace_ros2

# ROS1 build
RUN mkdir -p ~/workspace_ros1/src ~/workspace_ros2/src
COPY --chown=carma . /home/carma/workspace_ros1/src/
# RUN ~/workspace_ros1/src/docker/checkout.bash
# RUN ~/workspace_ros1/src/docker/install.sh

# ROS2 build
COPY --chown=carma . /home/carma/workspace_ros2/src/
RUN chmod u+x ~/workspace_ros2/src/docker/checkout_ros2.bash ~/workspace_ros2/src/docker/install_ros2.sh ~/workspace_ros2/src/docker/install_ssc_pm.sh
RUN ~/workspace_ros2/src/docker/checkout_ros2.bash
RUN ~/workspace_ros2/src/docker/install_ros2.sh

# Install astuff ros2 ssc_pm using tokens as arguments
ARG ACCESS_ID="NULL"
ARG SECRET_KEY="NULL"
RUN ~/workspace_ros2/src/docker/install_ssc_pm.sh ${ACCESS_ID} ${SECRET_KEY}

FROM base_image

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-ssc-interface-wrapper-driver"
LABEL org.label-schema.description="ssc interface wrapper driver for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-ssc-interface-wrapper/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=source-code --chown=carma /opt/carma /opt/carma
COPY --from=source-code --chown=carma /opt/ros/foxy /opt/ros/foxy

# Default launch ros1 verion
CMD [ "wait-for-it.sh", "localhost:11311", "--", "roslaunch", "ssc_interface_wrapper", "ssc_interface_wrapper.launch"]
