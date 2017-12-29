=========
Changelog
=========

0.7.5 (2017-12-29)
------------------
* bugfix missing nav_msgs dependency

0.7.1 (2016-08-13)
------------------
* bugfix for racing condition in goal assigning/enabling the docker

0.6.0 (2014-08-08)
------------------
* leaning comments
* refactoring
* publish debug message even if auto dock is not running
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: Jihoon Lee, Jorge Santos, jihoonl

0.5.5 (2013-10-11)
------------------
* Rename cmd_vel_mux as yocs_cmd_vel_mux.

0.5.4 (2013-09-06)
------------------

0.5.3 (2013-08-30)
------------------
* ros and non-ros stack split, driver, ftdi and auto-docking (partial) gone to kobuki_core.

0.5.0 (2013-08-29)
------------------
* Added extra url info on all packages.
* adds params for kobuki_auto_docking launchers.
* Changelogs at package level.
* 32 bit alignment error. Fast fix: just remove the
  eigen-inheritor attribute, as it's not really needed. It was already
  commented in a previous commit; here I just cleanup and provide a
  description of the fix.
  But the fact is that something is wrong on ecl. We keep track on.
* Fixed Eigenlib alignment error on 32 bit architectures.

0.4.0 (2013-08-09)
------------------
* Add minimum speed parameters: with heavy payloads, at very low speeds, the robot can get stuck easily.
* Remove motors enabling/disabling, as it can be confusing and it's not particularly useful.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
