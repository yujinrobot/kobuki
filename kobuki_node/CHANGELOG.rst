^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
