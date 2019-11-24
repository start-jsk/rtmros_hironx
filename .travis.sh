$!/bin/bash

set -x

function error {
  if [ $BUILDER == rosbuild ]; then find ${HOME}/.ros/rosmake/ -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
  if [ ~/ros/ws_$REPOSITORY_NAME/build/test_results ]; then find ~/ros/ws_$REPOSITORY_NAME/build/test_results -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
  #if [ ${HOME}/.ros/test_results ]; then find ${HOME}/.ros/test_results -type f -exec echo "=== {} ===" \; -exec cat {} \; ; fi
  #for file in ${HOME}/.ros/log/rostest-*; do echo "=== $file ==="; cat $file; done
  exit 1
}

trap error ERR

echo "Environment Variables"
echo "CI_SOURCE_PATH=$CI_SOURCE_PATH"
echo "REPOSITORY_NAME=$REPOSITORY_NAME"
echo "DISTRO=$DISTRO"
env | grep ROS

cd ${CI_SOURCE_PATH}

apt-get update
apt-get install -y sudo software-properties-common git wget sed
####### https://github.com/ros-planning/moveit/pull/581
if [ "$ROS_DISTRO" == "kinetic" ]; then
    apt-get install -y python-pyassimp
    sed -i 's@load, load_mem, release, dll@load, release, dll@' /usr/lib/python2.7/dist-packages/pyassimp/core.py
    cat -n /usr/lib/python2.7/dist-packages/pyassimp/core.py
fi
#######
  # Define some config vars
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then mkdir -p ~/catkin_ws/src; ln -sf `pwd` ~/catkin_ws/src/rtmros_hironx; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then cd ~/; git clone http://github.com/fkanehiro/hrpsys-base --depth 1; cd hrpsys-base  ; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then wget https://raw.githubusercontent.com/fkanehiro/hrpsys-base/d7c339e7e8ed64bd4004ed6510ebb4a4179fd8a7/.travis.sh -O work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@sudo apt-get install -qq -y ros-hydro-$pkg@sudo apt-get install -qq -y ros-hydro-$pkg || (sudo apt-get install -qq -y ros-hydro-hrpsys-ros-bridge; echo "OK")@' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@$HAVE_MONGO_DB@1@' work_with_315_1_10_test.sh; fi # indigo does not requires mongodb hack
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i "s@hydro@$ROS_DISTRO@g" work_with_315_1_10_test.sh; fi # update to $ROS_DISTRO
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i "s@precise@$DISTRO@g" work_with_315_1_10_test.sh; fi # update to $ROS_DISTRO
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then cat work_with_315_1_10_test.sh; fi # indigo does not requires mongodb hack
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@wstool set rtmros_common http://github.com/start-jsk/rtmros_common --git -y@@' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@wstool set rtmros_hironx http://github.com/start-jsk/rtmros_hironx --git -y@@' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@wstool set rtmros_nextage http://github.com/tork-a/rtmros_nextage --git -y@@' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i "s@sudo dpkg -r --force-depends ros-hydro-hrpsys@sudo dpkg -r --force-depends ros-$ROS_DISTRO-hrpsys; sudo mkdir -p /opt/ros/$ROS_DISTRO/include/hrpsys/idl@" work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's%cat hrpsys/catkin.cmake%sed -i "s@foreach(_bin_file \\${_bin_files})@list(REMOVE_DUPLICATES _bin_files)\\nforeach(_bin_file \\${_bin_files})@" hrpsys/catkin.cmake\n            cat hrpsys/catkin.cmake%' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@catkin_make_isolated -j1 -l1 --install --only-pkg-with-deps@catkin_make_isolated -j1 -l1 --install --only-pkg-with-deps hrpsys@' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then sed -i 's@git clone http://github.com/fkanehiro/hrpsys-base --depth 1 -b 315.1.9 ../build_isolated/hrpsys/build/hrpsys-base-source@(git clone http://github.com/fkanehiro/hrpsys-base ../build_isolated/hrpsys/build/hrpsys-base-source; cd ../build_isolated/hrpsys/build/hrpsys-base-source; git checkout 315.1.9; sed -i s%-mt%% lib/util/CMakeLists.txt)@' work_with_315_1_10_test.sh; fi
if [ "$TEST_TYPE" == work_with_315_1_10 ]; then TEST_TYPE=work_with_315_1_10  TEST_PACKAGE=hironx-ros-bridge bash ./work_with_315_1_10_test.sh; exit $?  ; fi

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"
sudo -E sh -c 'echo "deb $ROS_REPOSITORY_PATH $DISTRO main" > /etc/apt/sources.list.d/ros-latest.list'
cat /etc/apt/sources.list.d/ros-latest.list
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
sudo apt-get install -qq -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin
if [ $ROSWS == rosws ]; then sudo apt-get install -qq -y python-rosinstall     ; fi
if [ $BUILDER == rosbuild ]; then sudo apt-get install -qq -y ros-$ROS_DISTRO-rosmake ; fi
if [ "$EXTRA_DEB" ]; then sudo apt-get install -q -qq -y $EXTRA_DEB;  fi
  # Setup rosdep
sudo rosdep init
rosdep update --include-eol-distros; while [ $? != 0 ]; do sleep 1; rosdep update --include-eol-distros; done
  ##
### install: # Use this to install any prerequisites or dependencies necessary to run your build
  # Create workspace
mkdir -p ~/ros/ws_$REPOSITORY_NAME/src
cd ~/ros/ws_$REPOSITORY_NAME/src
env
if [ $USE_DEB == false -o $BUILDER == rosbuild ]; then $ROSWS init .   ; fi
if [ $USE_DEB == false ]; then $ROSWS merge file://$CI_SOURCE_PATH/.rosinstall      ; fi
  ##
if [ $USE_DEB == false -o $BUILDER == rosbuild ]; then if [ $ROSWS == rosws ]; then $ROSWS merge /opt/ros/$ROS_DISTRO/.rosinstall; fi  ; fi
if [ $USE_DEB == false ]; then python -c 'import sys, yaml; yaml.dump(yaml.load(sys.stdin), sys.stdout, width=1024, indent=4)'  < .rosinstall > .rosinstall.$$; mv .rosinstall.$$ .rosinstall; fi
if [ $USE_DEB == false ]; then sed -i "s@^\(.*github.com/.*/$REPOSITORY_NAME.*\)@#\1@" .rosinstall               ; fi # comment out current repo
if [ $USE_DEB == false ]; then $ROSWS update   ; fi
  # disable doc generation
if [ $USE_DEB == false ]; then sed -i "s@if(ENABLE_DOXYGEN)@if(0)@" rtm-ros-robotics/hrpsys/CMakeLists.txt   ; fi
ln -s $CI_SOURCE_PATH . # Link the repo we are testing to the new workspace
cd ../
  # Install dependencies for source repos
find -L src -name package.xml -exec dirname {} \; | xargs -n 1 -i find {} -name manifest.xml | xargs -n 1 -i mv {} {}.deprecated # rename manifest.xml for rosdep install
rosdep install -r -n --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
find -L src -name manifest.xml.deprecated | xargs -n 1 -i dirname {} | xargs -n 1 -i ln -sf `pwd`/{}/manifest.xml.deprecated `pwd`/{}/manifest.xml # rename manifest.xml for rosdep install
  #  find . \! -path "*/.*" -type f | xargs egrep -i "(hoge|fuga)" ; if [ $? == 0 ]; then exit 1; fi
# Use this to prepare your build for testing e.g. copy database configurations, environment variables, etc.
source /opt/ros/$ROS_DISTRO/setup.bash
if [ $BUILDER == rosbuild ]; then source src/setup.bash        ; fi
if [ $BUILDER == rosbuild ]; then rospack profile              ; fi
# All commands must exit with code 0 on success. Anything else is considered failure.
  # for catkin
if [ $BUILDER == catkin -a $USE_DEB == true  ]; then catkin_make ${ROS_PARALLEL_JOBS}            ; fi
  # - if [ $BUILDER == catkin -a $USE_DEB == false ]; then mkdir -p $HOME/ros/ws_$REPOSITORY_NAME/devel_isolated/hrpsys/; echo "$HOME/ros/ws_$REPOSITORY_NAME/devel_isolated/hrpsys/" > $HOME/ros/ws_$REPOSITORY_NAME/devel_isolated/hrpsys/.catkin; fi
if [ $BUILDER == catkin -a $USE_DEB == false ]; then awk '/cmake_minimum_required/,/## test/' src/rtm-ros-robotics/rtmros_common/hrpsys_ros_bridge/CMakeLists.txt | tee src/rtm-ros-robotics/rtmros_common/hrpsys_ros_bridge/CMakeLists.txt >/dev/null 2>&1; fi
if [ $BUILDER == catkin -a $USE_DEB == false ]; then catkin_make_isolated ${ROS_PARALLEL_JOBS}   ; fi
if [ $BUILDER == catkin ]; then export TARGET_PKG=`find build*/*hiro* -name Makefile -print | grep -v gtest | sed s@.*/\\\\\([^\/]*\\\\\)/Makefile@\\\1@g` ; echo ${TARGET_PKG}; fi
  # - if [ $BUILDER == catkin ]; then source devel/setup.sh; export EXIT_STATUS=0; for pkg in $TARGET_PKG; do (roscd $pkg; [ "`find . -iname '*.test'`" == "" ] && echo "[$pkg] No tests were found!!!"  || find . -iname "*.test" -print0 | xargs -0 -n1 rostest || export EXIT_STATUS=$?;) done; [ $EXIT_STATUS == 0 ] ; fi
export EXIT_STATUS=$?
if [ $BUILDER == catkin ]; then source devel*/setup.bash; env  ; fi
if [ $BUILDER == catkin ]; then python -c 'import sys; print(sys.path); import hrpsys_ros_bridge; print(hrpsys_ros_bridge)'; fi
if [ $BUILDER == catkin -a $USE_DEB == true  ]; then catkin_make test --pkg $TARGET_PKG ${ROS_PARALLEL_JOBS}  ; export EXIT_STATUS=$?; fi
  #- if [ $BUILDER == catkin -a $USE_DEB == false ]; then catkin_make_isolated --make-args ${ROS_PARALLEL_JOBS} test  ; export EXIT_STATUS=$?; fi
if [ $BUILDER == catkin -a $USE_DEB == false ]; then export EXIT_STATUS=0; for pkg in $TARGET_PKG; do make -C build_isolated/$pkg test || export EXIT_STATUS=$?; done; [ $EXIT_STATUS == 0 ] ; fi
if [ $EXIT_STATUS != 0 -a $BUILDER == catkin ]; then find build* -name LastTest.log -exec echo "==== {} ====" \; -exec cat {} \;  ; exit 1; fi
if [ $BUILDER == catkin -a $USE_DEB == true  ]; then source /opt/ros/$ROS_DISTRO/setup.bash       ; fi
if [ $BUILDER == catkin -a $USE_DEB == true  ]; then catkin_make ${ROS_PARALLEL_JOBS} install            ; fi
#if [ $BUILDER == catkin -a $USE_DEB == false ]; then rm -fr build devel; catkin_make_isolated ${ROS_PARALLEL_JOBS} --install      ; fi
if [ $BUILDER == catkin -a $USE_DEB == true ]; then rm -fr devel src build                 ; fi
if [ $BUILDER == catkin -a $USE_DEB == true ]; then source install*/setup.bash              ; fi
if [ $BUILDER == catkin -a $USE_DEB == true ]; then export EXIT_STATUS=0; for pkg in $TARGET_PKG; do [ "`find install/share/$pkg -iname '*.test'`" == "" ] && echo "[$pkg] No tests were found!!!"  || find install/share/$pkg -iname "*.test" -print0 | xargs -0 -n1 rostest || export EXIT_STATUS=$?; done; [ $EXIT_STATUS == 0 ] ; fi
  # for rosbuild
if [ $BUILDER == rosbuild ]; then rosmake -a --profile --pjobs=8       ; fi
if [ $BUILDER == rosbuild ]; then export TARGET_PKG=`find -L src | grep $REPOSITORY_NAME | grep /build/Makefile$ | sed s@.*/\\\\\([^\/]*\\\\\)/build/Makefile@\\\1@g` ; fi
if [ $BUILDER == rosbuild ]; then rosmake --test-only $TARGET_PKG --pjobs=8 ; fi

