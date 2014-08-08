^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2014-08-08)
------------------

0.6.0 (2014-08-08)
------------------
* remove kobuki_capabilities dependency from package.xml remove author emails
* moves app manager launcher to kobuki_capabilities (solves `#331 <https://github.com/yujinrobot/kobuki/issues/331>`_)
* kobuki_node: adds shutdown flag to nodelet (fixes `#324 <https://github.com/yujinrobot/kobuki/issues/324>`_)
* fixes typo
* updates icons for apps and app manager launcher
* adds minor changes due to capability server and app manager updates
* updates for new rapp lists
* publish_tf arg for the launcher.
* removes rviz launcher and dependency (fixes `#315 <https://github.com/yujinrobot/kobuki/issues/315>`_)
* adds app manager and capability server launcher for kobuki
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: Daniel Stonier, Jihoon Lee, Jorge Santos, Marcus Liebhardt

0.5.5 (2013-10-11)
------------------
* Add ftdi dependency.

0.5.4 (2013-09-06)
------------------

0.5.3 (2013-08-30)
------------------
* adds view robot launcher.

0.5.0 (2013-08-29)
------------------
* Added new interface about custom PID gain setting.
* Added extra url info on all packages.
* Updated old rnd email address.
* lock api for protecting data access with asynchronous getXXX calls.
* Fix URL to the previous changelog wiki.
* Changelogs at package level.
* Reset odometry also for heading.
* Set use_imu_heading as true (default).
* Do not use robot_pose_ekf. Instead, use imu for heading and encoders por x and y. However, parameter use_imu_heading makes trivial to switch back to the previous system.
* Added a debug topic that publish actual base command sent to robot.
* Added flexible logging features to using named logging system of ros/log4cxx.
* Reset odometry also works for heading (gyro).
* Do not use robot_pose_ekf; use imu for heading and encoders for position.

0.4.0 (2013-08-09)
------------------
* Change deprecated state_publisher to robot_state_publisher in launch files.
* Add launch file for full tf.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
