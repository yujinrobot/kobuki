^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_testsuite
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.7 (2014-08-18)
------------------

0.5.6 (2014-05-23)
------------------
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: Jorge Santos

0.5.5 (2013-10-11)
------------------
* New script: test_slow_drive.sh
  Moves following a 90cm radius circle while publishing accumulated and
  average error on angular velocity, using gyroscope data as reference.
  Useful for testing passive wheels configurations.
  See https://github.com/yujinrobot/kobuki/issues/202 for more details

0.5.4 (2013-09-06)
------------------

0.5.3 (2013-08-30)
------------------

0.5.0 (2013-08-29)
------------------
* adds extra url info to kobuki_testsuite
* adds readme for kobuki_testsuite
* removes turtlebot comments
* Fix URL to the previous changelog wiki
* Changelogs at package level

0.4.0 (2013-08-09)
------------------
* Add information about preliminary nodes for gyro_perf in the launcher file.
* Add mod function importing in gyro_perf.py which is missing.
* Add wrap_to_pi function into gyro_perf.py which is missing.
* Add convenient launcher file for single running of gyro_perf.py script.
* Add batch_test.sh for automated batch testing of gyro_perf script.
* Add gyro_perf.py script for performance measuring of gyro.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
