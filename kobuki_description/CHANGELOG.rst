^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.6 (2015-05-27)
------------------

0.6.5 (2014-11-21)
------------------

0.6.4 (2014-08-26)
------------------

0.6.3 (2014-08-25)
------------------

0.6.2 (2014-08-11)
------------------

0.6.1 (2014-08-08)
------------------

0.6.0 (2014-08-08)
------------------
* update body friction and revert torque limit
* update kobuki_gazebo.urdf.xacro to make gazebo simulation more stable.
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: John Hsu, Jorge Santos

0.5.5 (2013-10-11)
------------------

0.5.4 (2013-09-06)
------------------

0.5.3 (2013-08-30)
------------------
* disables vertical rays for the cliff sensors.
* slightly increases collision model for the base.

0.5.0 (2013-08-29)
------------------
* fixes collision object name for gazebo contact sensor.
* changes center cliff sensor name.
* changes simulated IMU.
* Added extra url info on all packages.
* Updated old rnd email address.
* Fix URL to the previous changelog wiki.
* Changelogs at package level.
* Do not use robot_pose_ekf. Instead, use imu for heading and encoders por x and y. However, parameter use_imu_heading makes trivial to switch back to the previous system.
* corrects inertia and center of mass using an approximation.

0.4.0 (2013-08-09)
------------------
* Many Gazebo 1.9+ related fixes.
* Update urdf and mesh files.
* We can recuperate the catkin version of this package because xacro has been (finally)  catkinized.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
