^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_bumper2pc
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* added comments to explain about the faraway points
* added side_point_angle param to change the angle of the bumper pointcloud's side points
* Add missing run dependency on yocs_cmd_vel_mux
* Contributors: Jorge Santos, Kaijen Hsiao

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
