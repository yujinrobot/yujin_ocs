^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ar_track_alvar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.3 (2016-02-02)
------------------
* [feat] New bool-Topic to enable/disable the marker detection (`#70 <https://github.com/sniekum/ar_track_alvar/issues/70>`_)
* [feat] added public way to set intrinsicCalibration for Camera class
* [fix] not publishing marker that are facing in the same direction as the camera.
* [sys] removed duplicate code for image subscription
* Contributors: Nikolas Engelhard, Scott Niekum

0.5.2 (2015-11-27)
------------------
* [fix] Move tf include from header to cpp files, fixes `#66 <https://github.com/sniekum/ar_track_alvar/issues/66>`_
  The header currently prevents us from re-using the library as a given library (because it pulls in tf2 which causes trouble). The include has been moved to the individual nodes which actually use a TransformBroadcaster.
* [fix] proper virtual destruction `#63 <https://github.com/sniekum/ar_track_alvar/issues/63>`_.
* improve license information in package.xml (`#58 <https://github.com/sniekum/ar_track_alvar/issues/58>`_)
* Added time stamp to header (`#57 <https://github.com/sniekum/ar_track_alvar/issues/57>`_)
  Previously, each pose had a timestamp, but the whole message did not. By including the timestamp for the whole message, it is now possible to use the results of the ar_pose_marker topic with other messages using message_filters::Synchronizer.
* Contributors: Alex Henning, Bener Suay, Lukas Bulwahn, Scott Niekum, Tim Niemueller, Isaac I. Y. Saito

0.5.1 (2015-04-14)
------------------
* Remove meta pkg; ar_track_alvar is 'unary stack' so no need for the meta pkg.
* Contributors: Scott Niekum, Isaac IY Saito

0.5.0 (2014-06-25)
------------------
* move README to root directory
* Merge remote-tracking branch 'origin/hydro-devel' into indigo-devel
* ar_track_alvar package uses ar_track_alvar_msgs
* restructuring packages. Separate out the message package.
* Contributors: Jihoon Lee

0.4.1 (2013-11-28)
------------------

0.3.3 (2013-02-22)
------------------

0.3.2 (2013-02-18)
------------------

0.3.1 (2013-02-14)
------------------

0.3.0 (2013-01-17)
------------------

0.2.0 (2012-08-08)
------------------
