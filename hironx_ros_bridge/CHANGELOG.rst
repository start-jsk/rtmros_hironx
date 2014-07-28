^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hironx_ros_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.19 (2014-07-28)
-------------------
* Enable RobotHardwareServiceROSBridge for when working with real robot. Fixes `#138 <https://github.com/start-jsk/rtmros_hironx/issues/138>`_ (servoOn/Off issue).
* (hironx_client) Add readDigitalOut.
* Robot installation
  * (robot-compile-openrtm.sh) Fix: Non-existent path. Add more instruction message.
  * (visionpc_install_setup.sh) Minor update (Add ros desktop-full, remove unnecessary Ubuntu init folders, ros env setting for nxouser)
* Contributors: Isaac IY Saito

1.0.18 (2014-07-21)
-------------------
* (hironx_client) Fixed some methods not returning what super class returns.
* Contributors: Isaac IY Saito

1.0.17 (2014-07-13)
-------------------
* 1st fully functional release (robot-compile-setup.sh, robot-system-check).
* Add install script for Vision PC Ubuntu.
* Add Nitta JR3 driver
* Adjust a few launch files to accommodate servo controller argument.
* Contributors: Kei Okada, Isaac IY Saito, Hajime Saito

1.0.16 (2014-07-08)
-------------------
* First release of install script suites (for QNX)
* (test-hironx.test, test-hironx-ros-bridge.test) Add omniNames script to start it on ros buildfarm (see https://github.com/start-jsk/rtmros_common/issues/416#issuecomment-46846623)
* (hironx_ros_bridge.launch) Pass corba port to collision detector launch
* hironx_ros_bridge_real.launch, enable ServoController for real robot
* Contributors: Kei Okada, Isaac IY Saito

1.0.15 (2014-06-22)
-------------------
* fix `#107 <https://github.com/start-jsk/rtmros_hironx/issues/107>`_
* Add acceptance test code for hrpsys-based api.
* (hironx_client.py) api document improved.
* (test_hironx.py) Add a testcase to check both arms simultaneous operation
* Launch collision detection viewer ("natto"-view) by default.
* (test-hironx-ros-bridge.test) Accept corba port input
* (robot/robot-compile-hrpsys.sh) update to use github
* (hironx_client.py) Improve arg name (#issues61#issuecomment-37535993)
* (test_hironx.py, test_hironx_ik.py, test_hironx_ros_bridge.py) relax test condition to pass travis
* rename test-hironx-ros-bridge.launch -> test-hironx-ros-bridge.test
* Add depends to gnuplot for test, currently our travis code does not see test_depends so add them to the {build,run}_depend
* (`#81 <https://github.com/start-jsk/rtmros_hironx/issues/81>`_) set test code for simulation environment
* add roslang/rosbash to depends for roslib.load_manifest()
* Contributors: Isaac IY Saito, Kei Okada

1.0.14 (2014-03-07)
-------------------
* Fix https://github.com/start-jsk/rtmros_hironx/issues/45 Add versioned build_dependency.
* Contributors: Isaac Isao Saito

1.0.13 (2014-03-06)
-------------------
* Add comment to clarify necessary build_depend.
* quick hack for missing python-tk on hrpsys/waitInput.py
* disable test-hironx-ros-bridge for now
* Comform to python file naming scheme so that test files run from travis
* use pkg-config --variable=idl_dir openhrp3.1 to specify openhrp3 directory
* support corbaport arguments
* Enable rostest
* Contributors: Isaac Isao Saito, Kei Okada

1.0.12 (2014-02-26)
-------------------
* Adding and improving unit test files.
* Adding travis conf files.
* Adding more checker programs for robot's internal os.
* Contributors: Isaac Isao Saito, Kei Okada

1.0.11 (2014-02-19)
-------------------
* Moved from googlecode.com to github.
* (hironx_client.py) Documenting a bunch. Removed humanoid specific methods.
* (hironx.py) fix to `#14 <https://github.com/start-jsk/rtmros_hironx/issues/14>`_
* add checker program for robot's internal os
* (test-hironx.py) quick fix to get it run with a real robot. This needs enhancement for versatility. Also removed test_goOffpose that interrupt the testing sequence by turning down servo.
* Fix the same issue with https://github.com/tork-a/rtmros_nextage/issues/25#issuecomment-32332068 by the same patch (https://github.com/tork-a/rtmros_nextage/commit/d4268d81ec14a514bb4b3b52614c81e708dd1ecc#diff-20257dd6ad60c0892cfb122c37a8f2ba)
* (hironx.py) Use generic name for the robot instance. This enables users on the script commandline (eg. ipython0 to run the same commands without asking them to specifically tell what robot they're using (eg. hiro, nxc). This is backward compatible so that users can still keep using `hiro`.
* Contributors: Isaac Isao Saito, Kei Okada
