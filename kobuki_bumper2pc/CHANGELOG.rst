^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_bumper2pc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.7 (2014-08-18)
------------------

0.5.6 (2014-05-23)
------------------
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: Jorge Santos

0.5.5 (2013-10-11)
------------------

0.5.4 (2013-09-06)
------------------

0.5.3 (2013-08-30)
------------------

0.5.0 (2013-08-29)
------------------
* kobuki : Added extra url info on all packages.
* Fix URL to the previous changelog wiki.
* Changelogs at package level.

0.4.0 (2013-08-09)
------------------
* Remove the dependency on pcl-ros. We compose sensor_msgs/PointCloud2 messages by hand, replacing pcl dependency by sensor_msgs one.
* Fixed Eigenlib alignment error on 32 bit architectures.
* Publish the pc continuously as long as bumper/cliff events are present.
* Publish stamped pointcloud.


Previous versions, bugfixing
============================

Available in ROS wiki: http://ros.org/wiki/kobuki/ChangeList
