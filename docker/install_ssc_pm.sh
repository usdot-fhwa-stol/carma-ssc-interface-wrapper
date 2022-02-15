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

# This script installs the ssc_pm_lexus package using the access id and secret key as arguments
# These tokens are required to run the script
access_id="NULL"
secret_key="NULL"

# Check if valid arguments are passed
if [ -z "$1" ]
    then 
        echo "No argument provided for access_id, this script needs to be run with <ACCESS_ID> <SECRET_KEY>"
        exit
else
    access_id=$1
fi

if [ -z "$2" ]
    then 
        echo "No argument provided for secret_key, this script needs to be run with <ACCESS_ID> <SECRET_KEY>"
        exit
else
    secret_key=$2
fi

cd ~
sudo apt-get update
sudo apt-get -qq install apt-transport-s3

sudo sh -c 'echo "AccessKeyId = '$access_id'" > /etc/apt/s3auth.conf'
sudo sh -c 'echo "SecretAccessKey = '$secret_key'" >> /etc/apt/s3auth.conf'
sudo sh -c 'echo "Token = \"\"" >> /etc/apt/s3auth.conf'
sudo sh -c 'echo "Region = \"us-east-1\"" >> /etc/apt/s3auth.conf'

sudo sh -c 'echo "deb [trusted=yes] s3://autonomoustuff-ssc $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-ssc.list'
sudo apt-get update
sudo apt-get -y install ros-foxy-ssc-pm-lexus ros-foxy-ssc-joystick