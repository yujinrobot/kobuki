^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.6 (2018-03-14)
------------------
* Use more finite values for initial odometry covariance (i.e. not DBL_MAX) so as not to cause problems upstream

0.6.6 (2015-05-27)
------------------
* install image directory closes `#357 <https://github.com/yujinrobot/kobuki/issues/357>`_
* Contributors: Jihoon Lee

0.6.5 (2014-11-21)
------------------
* update diagnostics.yaml to show kobuki battery properly in kobuki dashboard. Fix `#350 <https://github.com/yujinrobot/kobuki/issues/350>`_
* Contributors: Jihoon Lee

0.6.4 (2014-08-26)
------------------
* Merge branch 'indigo' of https://github.com/yujinrobot/kobuki into indigo
* rename run_depend of kobuki_node from kobuki_apps to kobuki_rapps
* Contributors: Jihoon Lee

0.6.3 (2014-08-25)
------------------

0.6.2 (2014-08-11)
------------------
* add queue_size on publiehrs `#338 <https://github.com/yujinrobot/kobuki/issues/338>`_
* fixing the file link resolves `#339 <https://github.com/yujinrobot/kobuki/issues/339>`_
* Contributors: Jihoon Lee

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
