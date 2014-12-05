^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package yocs_cmd_vel_mux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.3 (2014-12-05)
------------------

0.6.2 (2014-11-30)
------------------
* yocs_cmd_vel_mux: fixes node handle for output pub
  to keep backwards compatibility
* adds a little launcher restructing for muxer and smoother
* Contributors: Marcus Liebhardt

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

0.5.1 (2013-10-14)
------------------
* Unify naming politics for binaries and plugins.

0.5.0 (2013-10-11)
------------------
* Renamed as yocs_cmd_vel_mux.

0.4.1 (2013-10-08)
------------------

0.4.0 (2013-08-29)
------------------
* Add bugtracker and repo info URLs.
* Changelogs at package level.
* License link fixed.

0.3.0 (2013-07-02)
------------------

0.2.3 (2013-04-15)
------------------

0.2.2 (2013-02-10)
------------------

0.2.1 (2013-02-08)
------------------

0.2.0 (2013-02-07)
------------------
* Catkinized.

0.1.3 (2013-01-08)
------------------
* More generous description.

0.1.2 (2013-01-02)
------------------
* Dynamically reconfigurable.
* Upgraded to new groovy plugin formats.
* Add reconfigure launcher and parameter file.
* Add a dynamic reconfigure script to accept a yaml filename.

0.1.1 (2012-12-21)
------------------

0.1.0 (2012-12-05)
------------------
* Initial version.
