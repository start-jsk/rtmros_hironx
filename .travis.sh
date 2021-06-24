#!/bin/bash

set -x

function error {
  if [ $BUILDER == rosbuild ]; then find ${HOME}/.ros/rosmake/ -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
  if [ ~/ros/ws_$REPOSITORY_NAME/build/test_results ]; then find ~/ros/ws_$REPOSITORY_NAME/build/test_results -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
  #if [ ${HOME}/.ros/test_results ]; then find ${HOME}/.ros/test_results -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
  #for file in ${HOME}/.ros/log/rostest-*; do echo "=== $file ==="; cat $file; done
  exit 1
}

#trap error ERR

echo "Environment Variables"
echo "CI_SOURCE_PATH=$CI_SOURCE_PATH"
echo "REPOSITORY_NAME=$REPOSITORY_NAME"
echo "DISTRO=$DISTRO"
env | grep ROS

cd ${CI_SOURCE_PATH}

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
apt-get install -y sudo software-properties-common git wget sed
####### https://github.com/ros-planning/moveit/pull/581
if [ "$ROS_DISTRO" == "kinetic" ]; then
    apt-get install -y python-pyassimp
    sed -i 's@load, load_mem, release, dll@load, release, dll@' /usr/lib/python2.7/dist-packages/pyassimp/core.py
    cat -n /usr/lib/python2.7/dist-packages/pyassimp/core.py
fi
#######
  # Define some config vars
mkdir -p ~/catkin_ws/src; ln -sf `pwd` ~/catkin_ws/src/rtmros_hironx
cd ~/; git clone http://github.com/fkanehiro/hrpsys-base --depth 1; cd hrpsys-base
wget https://raw.githubusercontent.com/fkanehiro/hrpsys-base/d7c339e7e8ed64bd4004ed6510ebb4a4179fd8a7/.travis.sh -O work_with_315_1_10_test.sh
sed -i 's@sudo apt-get install -qq -y ros-hydro-$pkg@sudo apt-get install -qq -y ros-hydro-$pkg || (sudo apt-get install -qq -y ros-hydro-hrpsys-ros-bridge; echo "OK")@' work_with_315_1_10_test.sh
sed -i 's@$HAVE_MONGO_DB@1@' work_with_315_1_10_test.sh # indigo does not requires mongodb hack
sed -i "s@hydro@$ROS_DISTRO@g" work_with_315_1_10_test.sh # update to $ROS_DISTRO
sed -i "s@precise@$DISTRO@g" work_with_315_1_10_test.sh # update to $ROS_DISTRO
cat work_with_315_1_10_test.sh # indigo does not requires mongodb hack
sed -i 's@wstool set rtmros_common http://github.com/start-jsk/rtmros_common --git -y@@' work_with_315_1_10_test.sh
sed -i 's@wstool set rtmros_hironx http://github.com/start-jsk/rtmros_hironx --git -y@@' work_with_315_1_10_test.sh
sed -i 's@wstool set rtmros_nextage http://github.com/tork-a/rtmros_nextage --git -y@@' work_with_315_1_10_test.sh
sed -i "s@sudo dpkg -r --force-depends ros-hydro-hrpsys@sudo dpkg -r --force-depends ros-$ROS_DISTRO-hrpsys; sudo mkdir -p /opt/ros/$ROS_DISTRO/include/hrpsys/idl@" work_with_315_1_10_test.sh
sed -i 's%cat hrpsys/catkin.cmake%sed -i "s@foreach(_bin_file \\${_bin_files})@list(REMOVE_DUPLICATES _bin_files)\\nforeach(_bin_file \\${_bin_files})@" hrpsys/catkin.cmake\n            cat hrpsys/catkin.cmake%' work_with_315_1_10_test.sh
sed -i 's@catkin_make_isolated -j1 -l1 --install --only-pkg-with-deps@catkin_make_isolated -j1 -l1 --install --only-pkg-with-deps hrpsys@' work_with_315_1_10_test.sh
sed -i 's@git clone http://github.com/fkanehiro/hrpsys-base --depth 1 -b 315.1.9 ../build_isolated/hrpsys/build/hrpsys-base-source@(git clone http://github.com/fkanehiro/hrpsys-base ../build_isolated/hrpsys/build/hrpsys-base-source; cd ../build_isolated/hrpsys/build/hrpsys-base-source; git checkout 315.1.9; sed -i s%-mt%% lib/util/CMakeLists.txt)@' work_with_315_1_10_test.sh
# melodic
if [ "$ROS_DISTRO" == "melodic" ]; then sed -i 's@libcv-dev libhighgui-dev@libopencv-dev libopencv-highgui-dev ros-melodic-openrtm-aist@' work_with_315_1_10_test.sh; fi
if [ "$ROS_DISTRO" == "melodic" ]; then sed -i 's!# we use latest hrpsys_ocnfig.py for this case, so do not install them!sed -i "s@OPENRTM_VERSION STREQUAL \\\"1.1.0\\\"@(NOT (OPENRTM_VERSION VERSION_LESS \\\"1.1.0\\\")) AND (OPENRTM_VERSION VERSION_LESS \\\"1.2.0\\\")@" ../build_isolated/hrpsys/build/hrpsys-base-source/cmake_modules/FindOpenRTM.cmake\n\t\tsed -i "s@\\\(boost_.*\\\)-mt@\\1@" ../build_isolated/hrpsys/build/hrpsys-base-source/lib/util/CMakeLists.txt\n\t\tsed -i "s@-DHRPSYS_PACKAGE_VERSION@-fpermissive -Wno-deprecated -DHRPSYS_PACKAGE_VERSION@" ../build_isolated/hrpsys/build/hrpsys-base-source/CMakeLists.txt!' work_with_315_1_10_test.sh; fi
TEST_TYPE=work_with_315_1_10  TEST_PACKAGE=hironx-ros-bridge bash ./work_with_315_1_10_test.sh; exit $?
