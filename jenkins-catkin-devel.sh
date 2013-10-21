#!/bin/bash -x

trap 'exit 1' ERR

function setup {
    rm -fr catkin_ws
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    wstool init
    wstool merge https://rtm-ros-robotics.googlecode.com/svn/trunk/rtm-ros-robotics.rosinstall
    wstool update
    catkin_init_workspace
    cd ../../
}

source /opt/ros/groovy/setup.bash
setup
cd catkin_ws
catkin_make

source devel/setup.bash
source `rospack find openrtm_tools`/scripts/rtshell-setup.sh
rtmtest hironx_ros_bridge test-hironx.launch
rtmtest hironx_ros_bridge test-hironx-ros-bridge.launch

catkin_make install
