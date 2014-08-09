Launch files
=============

The following two launch files are the main components for this robot. Users usually don't launch neither of them directly (instead, use the ones explained later).

 * `hironx_ros_bridge.launch` starts processes we call as "ROS bridge", that bridges between `ROS` and `OpenRTM`. This is necessary when you use ROS with both real robot and simulation.
 * `hironx_startup.launch` is used only for the simulation; This starts up `Omniorb`'s name server, which is not necessary with the real robot.

As mentioned, users don't directly launch neither of above; launch one of the followings instead:

 * `hironx_ros_bridge_real.launch` for the real robot.
 * `hironx_ros_bridge_simulation.launch` for the simulation.

Unit tests
============

There are multiple usecases for tests. So far with this package we aim two of them: stress tests and acceptance tests. They are separated into different `rostest` files (`.test`). Most of the files relevant to testing are available in `/test` folder. 

Stress tests
----------------

Test cases in these suites are relatively aggressive.
Similar to the `.launch`, `rostest` (`.test`) files are separated per `ROS` and `hrpsys` as follows:

 * `test-hironx-ros-bridge.test` tests `ROS`-based commands via `ROS_Client` client class. This internally launches `hironx_ros_bridge.launch`.
 * `test-hironx.test` tests `hrpsys`-based commands via `HIRONX` client class. This launches `hironx_startup.launch`.

NOTE: these stress tests are currently intended for the simulation only (see the reasoning[1_]. Run on the **REAL ROBOT WITH EXTRA CARE**.

Acceptance tests
------------------

(TBD)

.. _1: https://github.com/start-jsk/rtmros_hironx/issues/81#issuecomment-41697482
