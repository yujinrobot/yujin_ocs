^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yocs_waypoints_navi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
