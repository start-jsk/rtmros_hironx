^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hironx_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2017-09-07)
------------------
* set trajectory_execution/allowed_execution_duration_scaling to 2.0 (`#518 <https://github.com/start-jsk/rtmros_hironx/issues/518>`_)
  - Same as https://github.com/tork-a/rtmros_nextage/commit/b568101055fec975b6130cebf6150f85106e3bee
  - see https://answers.ros.org/question/196586/how-do-i-disable-execution_duration_monitoring/
  * Change to use moveit_simple_controller
* Contributors: Ryosuke Tajima

2.0.0 (2017-08-10)
------------------
* Use docker to run tests and add kinetic test (`#517 <https://github.com/start-jsk/rtmros_hironx/issues/517>`_)
  * remove pr2_controller_msgs and use control_msgs

* Contributors: Kei Okada

1.1.25 (2017-06-02)
-------------------

1.1.24 (2017-05-09)
-------------------

1.1.23 (2017-04-03)
-------------------

1.1.22 (2017-03-24)
-------------------

1.1.21 (2017-03-16)
-------------------

1.1.20 (2017-02-09)
-------------------

1.1.19 (2017-01-12)
-------------------

1.1.18 (2016-10-28)
-------------------
* [fix][moveit config] Fix Interactive Marker size.
* Contributors: Isaac I.Y. Saito

1.1.17 (2016-10-13)
-------------------

1.1.16 (2016-07-11)
-------------------

1.1.15 (2016-06-02)
-------------------
* Workaround MoveIt! RRT issue (see https://github.com/tork-a/rtmros_nextage/issues/170).
* Contributors: Isaac I.Y. Saito

1.1.14 (2016-05-19)
-------------------

1.1.13 (2016-05-16)
-------------------
* [sys] Add test for eef geometry.
* Contributors: Isaac I.Y. Saito

1.1.12 (2016-05-05)
-------------------
* [fix][test_hironx_moveit.py] fix "rospy.init_node() has already been called with different arguments" exception.
* [fix][test_hironx_moveit.py] old file names
* [improve][test-hironx-moveit.test] Relax test duration.
* Contributors: Kei Okada

1.1.11 (2016-02-18)
-------------------

1.1.10 (2016-02-11)
-------------------

1.1.9 (2016-02-11)
------------------

1.1.8 (2016-02-09)
------------------

1.1.7 (2016-02-05)
------------------

1.1.6 (2016-02-03)
------------------

1.1.5 (2016-01-26)
------------------
* [feat][ROS_CLient] Upperbody move group.
* [feat] Rename both arms group to adjust to that of NEXTAGE Open.
* [feat] Add torso and head move groups.
* [feat] Factory-init pose for MoveIt! reserved pose.
* Contributors: Isaac I.Y. Saito

1.1.4 (2016-01-25)
------------------
* [sys][moveit config] Enable unit test for ROS_Client-RobotCommander integration
  Missing dependency
* Contributors: Isaac I.Y. Saito

1.1.3 (2015-12-16)
------------------

1.1.2 (2015-11-11)
------------------

1.1.1 (2015-11-02)
------------------

1.0.37 (2015-09-11)
-------------------

1.0.36 (2015-08-24)
-------------------
* [feat] Add dual-arm moveit group
* [feat] Add init pose to moveit_config
* [test] Add unit test cases for dual-arm group
* Contributors: Isaac IY Saito

1.0.35 (2015-08-14)
-------------------

1.0.34 (2015-08-04)
-------------------

1.0.33 (2015-07-30)
-------------------

1.0.32 (2015-07-16)
-------------------

1.0.31 (2015-04-28)
-------------------

1.0.30 (2015-04-16)
-------------------

1.0.29 (2015-04-06)
-------------------

1.0.28 (2015-02-06)
-------------------

1.0.27 (2014-11-04)
-------------------

1.0.26 (2014-10-07)
-------------------

1.0.25 (2014-10-03)
-------------------

1.0.24 (2014-09-16)
-------------------

1.0.23 (2014-09-02)
-------------------
* (hironx moveit) Remove a file added by mistake.
* Contributors: Isaac IY Saito

1.0.22 (2014-08-26)
-------------------

1.0.21 (2014-08-11)
-------------------

1.0.20 (2014-07-31)
-------------------

1.0.19 (2014-07-28)
-------------------

1.0.18 (2014-07-21)
-------------------

1.0.17 (2014-07-13)
-------------------

1.0.16 (2014-07-08)
-------------------

1.0.15 (2014-06-22)
-------------------
* Enable "natto"-view on RViz.
* Disable query for start state in Moveit RViz plugin.
* Contributors: Isaac IY Saito, Kei Okada

1.0.14 (2014-03-07)
-------------------

1.0.13 (2014-03-06)
-------------------
* Applying an important change suggest by moveit developers (same as https://github.com/tork-a/rtmros_nextage/issues/46).
* (hironx_moveit_config) Add run_depend on moveit_planners to avoid the error happens on RViz Moveit plugin without.
* Contributors: Isaac Isao Saito

1.0.12 (2014-02-26)
-------------------
* (moveit_rviz.launch) Enable to respawn rviz
* Contributors: Isaac Isao Saito

1.0.11 (2014-02-19)
-------------------
* Initial commit of CHANGELOG.rst files.
* Contributors: Isaac Isao Saito
