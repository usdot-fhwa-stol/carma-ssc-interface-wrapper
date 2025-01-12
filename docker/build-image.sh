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
declare -i false=0 true=1

USERNAME=usdotfhwastol

cd "$(dirname "$0")"
IMAGE=$(basename `git rev-parse --show-toplevel`)

echo ""
echo "##### $IMAGE Docker Image Build Script #####"
echo ""

token=""
build_ros1_pkgs="$false"
build_ros2_pkgs="$false"
dockerfile_dir=""

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
        -v|--version)
            COMPONENT_VERSION_STRING="$2"
            shift
            shift
            ;;
        --system-release)
            SYSTEM_RELEASE=true
            shift
            ;;
        -p|--push)
            PUSH=true
            shift
            ;;
        -d|--develop)
            USERNAME=usdotfhwastoldev
            COMPONENT_VERSION_STRING=develop
            shift
            ;;
        *) ##Arguments for ssc_pm_lexus
            token="$1"
            shift
            ;;
    esac
done

if [[ -z "$COMPONENT_VERSION_STRING" ]]; then
    COMPONENT_VERSION_STRING=$("./get-component-version.sh")
fi

echo "Building docker image for $IMAGE version: $COMPONENT_VERSION_STRING"
echo "Final image name: $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING"

# Get arguments for ssc_pm_lexus


if [[ $build_ros1_pkgs -eq 1 ]]; then
    cd ../noetic
    echo "======= ROS1 build selected ========"
elif [[ $build_ros2_pkgs -eq 1 ]]; then
    cd ../humble
    if [ -z $token ]; then
        echo "No argument provided for token for ROS2 build, this script needs to be run with <TOKEN>"
        exit 1
    fi
    echo "======= ROS2 build selected ========"
fi

if [[ $COMPONENT_VERSION_STRING = "develop" ]]; then
    sed "s|usdotfhwastoldev/|$USERNAME/|g; s|usdotfhwastolcandidate/|$USERNAME/|g; s|usdotfhwastol/|$USERNAME/|g; s|:[0-9]*\.[0-9]*\.[0-9]*|:$COMPONENT_VERSION_STRING|g; s|checkout.bash|checkout.bash -d|g" \
        Dockerfile | docker build --network=host -f - -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING \
        --build-arg VERSION="$COMPONENT_VERSION_STRING" \
        --build-arg VCS_REF=`git rev-parse --short HEAD` \
        --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` \
        --build-arg TOKEN=$token ../
else
    docker build --network=host -f Dockerfile -t $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING \
        --build-arg VERSION="$COMPONENT_VERSION_STRING" \
        --build-arg VCS_REF=`git rev-parse --short HEAD` \
        --build-arg BUILD_DATE=`date -u +”%Y-%m-%dT%H:%M:%SZ”` \
        --build-arg TOKEN=$token ../
fi

TAGS=()
TAGS+=("$USERNAME/$IMAGE:$COMPONENT_VERSION_STRING")

docker tag $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING $USERNAME/$IMAGE:latest
TAGS+=("$USERNAME/$IMAGE:latest")

echo "Tagged $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING as $USERNAME/$IMAGE:latest"

if [ "$SYSTEM_RELEASE" = true ]; then
    SYSTEM_VERSION_STRING=$("./get-system-version.sh")
    docker tag $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING $USERNAME/$IMAGE:$SYSTEM_VERSION_STRING
    echo "Tagged $USERNAME/$IMAGE:$COMPONENT_VERSION_STRING as $USERNAME/$IMAGE:$SYSTEM_VERSION_STRING"
    TAGS+=("$USERNAME/$IMAGE:$SYSTEM_VERSION_STRING")
fi

if [ "$PUSH" = true ]; then
    for tag in $TAGS; do
        docker push "${tag}"
    done
fi

echo ""
echo "##### $IMAGE Docker Image Build Done! #####"
