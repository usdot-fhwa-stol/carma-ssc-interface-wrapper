#  Copyright (C) 2018-2019 LEIDOS.
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

FROM usdotfhwastol/autoware.ai:3.1.0 as deps
#sudo curl -o linuxcan.tar.gz https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest
#https://www.kvaser.com/download/?utm_source=software&utm_ean=7330130980754&utm_status=latest
# Install LinuxCAN SDK
# RUN sudo apt install -y software-properties-common && \
#     sudo apt update && \
#     sudo apt-add-repository ppa:jwhitleyastuff/linuxcan-dkms && \
#     sudo apt update && \
#     sudo apt install -y linuxcan-dkms

# COPY --chown=carma ./kvaser /usr/src

# RUN bash -c 'cd /usr/src && \
#     sudo rm -rf linuxcan/ && \
#     sudo tar xvf linuxcan.tar.gz && \
#     cd linuxcan/ && \
#     make && \
#     sudo make install'
    
# # Install AStuff Kvaser Deps
# RUN sudo apt-get update && sudo apt-get install apt-transport-https && \
#     sudo apt-get install kvaser-canlib-dev && \
#     sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
#     sudo apt-get update && \
#     sudo apt-get install -y ros-$ROS_DISTRO-kvaser-interface

# Install remaining package deps
RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN rosdep install --from-paths /home/carma/src/ --ignore-src -r -y
RUN rm -R /home/carma/src/

FROM deps as setup

RUN sudo apt-get update && sudo apt-get install -y ros-kinetic-jsk-recognition-msgs

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/
RUN ~/src/docker/checkout.sh
RUN ~/src/docker/install.sh

FROM deps

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-ssc-interface-wrapper-driver"
LABEL org.label-schema.description="ssc interface wrapper driver for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/CARMASscInterfaceWrapper/"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --from=setup /home/carma/install /opt/carma/install

CMD [ "wait-for-it.sh", "localhost:11311", "--", "roslaunch", "ssc_interface_wrapper", "ssc_interface_wrapper.launch"]
