# Overview

root workspace for ROS2 version of SCC interface wrapper

# Setting up the environment
## Clone/switch to the scc repo/ssc_interface_wrapper_ros2
```sh
$ git clone git@github.com:KBR-CARMA/carma-ssc-interface-wrapper.git
# or
cd ~/carma-ssc-interface-wrapper/ssc_interface_wrapper_ros2
```

## Switch to feature branch
```sh
$ git checkout <my-feature-branch> 
```
## Get latest 
```sh
$ git pull
```

## Go back to main repo
```sh
$ cd ~/carma-ssc-interface-wrapper
```

# Launching the CARMA container 
Pull the current c1tenth image quitter.tech/carma-platform:c1tenth-develop 
Launch under name "dev"
mounting the scc interface as a volume and bash into it

```sh
$ docker run -it --rm --name dev -v $PWD/ssc_interface_wrapper_ros2:/home/carma/ssc_interface_wrapper_ros2 quitter.tech/carma-platform:c1tenth-develop bash
```
## The prompt should now look something like this
```sh
carma@a1d099b96ab7:/$
```
## Start a new terminal session to avoid ROS source issues
```sh
docker exec -it dev bash
carma@b45d56a34bc:/$
```
## Create a workspace inside the container
```sh 
carma $ mkdir -p ~/tmp_ws/src  
carma $ cd ~/tmp_ws/src
```
## Create a symlink to the package
```sh 
carma $ ln -s /home/carma/ssc_interface_wrapper_ros2
carma $ cd ~/tmp_ws
```
## Source the container for ROS2 and check that it's pure ROS2
```sh
carma $ source /opt/carma/install_ros2/setup.bash
carma $ printenv | grep ROS
```
## install and update dependencies
```sh
sudo apt update
sudo apt-get install ros-foxy-pacmod3
rosdep install --from-paths src --ignore-src -r -y
```

## Build the packages
```sh
carma $ cd ~/tmp_ws
carma $ colcon build --event-handlers console_direct+ --packages-up-to ssc_interface_wrapper_ros2
```

## Source the package
```sh
carma $ source install/setup.bash
```
## Launch the package
```sh
carma $ ros2 launch ssc_interface_wrapper_ros2 ssc_interface_wrapper.launch.py
```
