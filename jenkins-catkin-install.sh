#!/bin/bash

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
    cd ../../
}
#

DISTRO=${1:-groovy}
source /opt/ros/${DISTRO}/setup.bash
setup
cd catkin_ws
if [ "${DISTRO}" == "hydro" ]; then
    (cd src; wstool set robot_model https://github.com/ros/robot_model.git --git -y; wstool update robot_model)
fi
rosdep update
rosdep install --reinstall --from-paths src --ignore-src --rosdistro ${DISTRO} -y -r

#
for dir in openrtm_common/openrtm_aist_core openrtm_common/rtshell_core openrtm_common/openhrp3 openrtm_common/hrpsys rtmros_common/rtmbuild rtmros_common/openrtm_tools rtmros_common/openrtm_ros_bridge rtmros_common/hrpsys_tools rtmros_common/hrpsys_ros_bridge rtmros_hironx/hironx_ros_bridge rtmros_hironx/hironx_moveit_config; do
    wsdir=`basename $dir`_ws
    echo "$dir -> $wsdir"
    rm -fr $wsdir
    mkdir -p $wsdir/src
    cp -r src/rtm-ros-robotics/$dir $wsdir/src/`basename $dir`
    (cd $wsdir/src; catkin_init_workspace)
    cd $wsdir
    catkin_make install
    rm -fr devel build
    source install/setup.bash
    cd ..
done

source `rospack find openrtm_tools`/scripts/rtshell-setup.sh
trap 'rosrun rosunit clean_junit_xml.py' ERR
rtmtest hironx_ros_bridge test-hironx.launch
rtmtest hironx_ros_bridge test-hironx-ros-bridge.launch
