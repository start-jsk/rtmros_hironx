#!/bin/bash -x

trap 'exit 1' ERR

function setup {
    rm -fr catkin_ws
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    wstool init
    wstool merge https://rtm-ros-robotics.googlecode.com/svn/trunk/rtm-ros-robotics.rosinstall
    wstool remove rtm-ros-robotics/rtmros_tutorials
    wstool remove rtm-ros-robotics/rtmros_gazebo
    wstool update
    catkin_init_workspace
    cd ../../
}

DISTRO=${1:-groovy}
source /opt/ros/${DISTRO}/setup.bash
setup
cd catkin_ws
if [ "${DISTRO}" == "hydro" ]; then
    (cd src; wstool set robot_model https://github.com/ros/robot_model.git --git -y; wstool update robot_model)
fi
rosdep update
rosdep install --reinstall --from-paths src --ignore-src --rosdistro ${DISTRO} -y -r
catkin_make

source devel/setup.bash
source `rospack find openrtm_tools`/scripts/rtshell-setup.sh
trap 'rosrun rosunit clean_junit_xml.py' ERR
rtmtest hironx_ros_bridge test-hironx.launch
rtmtest hironx_ros_bridge test-hironx-ros-bridge.launch

catkin_make install
