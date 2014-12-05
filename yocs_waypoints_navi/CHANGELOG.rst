^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yocs_waypoints_navi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2014-12-05)
------------------
* Use header frame instead of world_frame for goal
* Contributors: AlexReimann

0.6.2 (2014-11-30)
------------------
* Merge branch 'waypoints_update' of https://github.com/yujinrobot/yujin_ocs into waypoints_update
* yocs_waypoints_navi: minimal improvement
* [yocs_waypoints_navi] public interfaces need to be public for the
  concert.
* yocs_waypoints_navi: removes timeout when waiting for action server
* yocs_waypoints_navi: adds special build message dependency for yocs_msgs
* yocs_waypoints_navi: adds feedback when cancel goal fails
* yocs_waypoints_navi: adds minor fixes and improvements
* yocs_waypoints_navi: moves class declaration into a separate header file
* yocs_waypoints_navi: removes commented out, old features
* yocs_waypoints_navi: changes ROS API and interal logic
  * drop of config through yaml
  * drop of RViz inputs
  * adds way point and trajectory input
  * state logic updated to handle new inputs
* Contributors: Daniel Stonier, Marcus Liebhardt

0.6.0 (2014-07-08)
------------------
* updating package informations. remove email for authors. updating maintainer
* Contributors: Jihoon Lee

0.5.3 (2014-03-24)
------------------
* Added support for YAML-CPP 0.5+.
  The new yaml-cpp API removes the "node >> outputvar;" operator, and
  it has a new way of loading documents. There's no version hint in the
  library's headers, so I'm getting the version number from pkg-config.
  This part of the patch is a port of the ones created by @ktossell for
  map_server and other packages.
  The new yaml-cpp also does not have FindValue.
* Contributors: Scott K Logan

0.5.2 (2013-11-05)
------------------
* Initial version.
