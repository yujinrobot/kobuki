^^^^^^^^^
Changelog
^^^^^^^^^

Hydro, unstable
===============

Version: 0.4.0 [2013-07-10]
---------------------------
* [[kobuki_driver]] : Windows compatible.
* [[kobuki_bumper2pc]] : Remove the dependency on pcl-ros by manually composing sensor_msgs/PointCloud2 messages.
* [[kobuki_bumper2pc]] : Publish the pointcloud continuously as long as bumper/cliff events are present.
* [[kobuki_softnode]] : Migrated to an independent repo.
* [[kobuki_node]] : Changed deprecated state_publisher to robot_state_publisher in launch files.
* [[kobuki_description]] : Many minor changes to work on new gazebo.
* [[kobuki_auto_docking]] : Add minimum speed parameters: with heavy payloads, at very low speeds, the robot can get stuck easily.
* Update doxygen documentation for using wstool and for windows compilation.
* New scripts to help serve firmware and windows downloads from our file server.


Groovy, bugfixing
=================

Version: 0.3.9 [2013-06-04]
---------------------------
* [[kobuki_auto_docking]] : eigen unaligned array crash on 32-bit machines fixed.

Version: 0.3.8 [2013-05-22]
---------------------------
* [[kobuki_driver]] : bugfix a rounding error in the diff drive module causing shaky motions.
* [[kobuki_bumper2pc]] : eigen unaligned array crash on 32-bit machines fixed.

Version: 0.3.7 [2013-04-16]
---------------------------
* metapackage updates for new catkin-bloom.
* [[kobuki_testsuite]] : add missing pykdl dependency.
* [[kobuki_testsuite]] : clean out old rosbuild style imports.
* [[kobuki_driver]] : bugfix acceleration limits.
* [[kobuki_driver]] : mutex protection around diff drive module.

Version: 0.3.6 [2013-03-04]
---------------------------
* [[kobuki_driver]]: bugfix inverted turning at very low linear speeds.

Version: 0.3.5 [2013-02-20]
---------------------------
* [[kobuki_ftdi]] : udev rule conflict (w/ bluetooth) bugfix.

Version: 0.3.4 [2013-02-10]
---------------------------
* Catkinization and minor catkin fixes.

Version: 0.2.4 [2013-01-21]
---------------------------
* Fix undefined symbol compilation error on Quantal.
* Updated doxygen documentation for most packages.
* [[kobuki_driver]] : version_info program to print hardware/firmware/driver versions.
* [[kobuki_bumper2pc]] : Separate publishing of bumps and cliff detections as a pointcloud to new package.

Version: 0.2.3 [2013-01-16]
---------------------------
* Use ecl debs instead of the stacks
* Updated and extended documentation.
* [[kobuki_driver]] : added a simple loop demo program.
* [[kobuki_ftdi]] : added an 'unflasher' program (restores eeprom data to default).
* [[kobuki_driver]] : sigslots demo program.

Version: 0.2.2 [2013-01-02]
---------------------------
* Updated doxygen documentation.
* Fix up the prefix removal for android
* Update charging state messages to be identical to the create node.
* Added charging and motor state to diagnostics
* [[kobuki_driver]] : added raw 3d gyro data publishing
* Added view model launcher for kobuki

Version: 0.2.1 [2012-12-22]
---------------------------
* satisfy REP requirements for base_link
* nodelet api updated for groovy's pluginlib

Version: 0.2.0 [2012-12-21] 
---------------------------
* Diverging from fuerte compatibility.

Version: 0.1.9 [2012-12-20] 
---------------------------
* Catkin fix.

Version: 0.1.8 [2012-12-15] 
---------------------------
* First freeze.


Fuerte, deprecated
==================

Version: 0.1.9 [2012-12-21] 
---------------------------
* Last compatible fuerte release.
