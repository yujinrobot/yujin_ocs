^^^^^^^^^
Changelog
^^^^^^^^^

Hydro, unstable
===============

Version: 0.3,0 [2013-07-02]
---------------------------
* New pose controller for differential-drive bases.
* Fix on velocity smoother to deal with low-rate simulated time.
* Allow using end velocity commands as robot feedback on velocity smoother.


Groovy, bugfixing
=================

Version: 0.2.3 [2013-04-14]
---------------------------
* Metapackage updates for catkin.

Version: 0.2.2 [2013-02-15]
---------------------------
* Add missing geometry_msgs dependency.

Version: 0.2.1 [2013-02-08]
---------------------------
* Catkinization and minor catkin fixes.

Version: 0.1.3 [2013-01-08]
---------------------------
* Dynamic reconfigure for velocity smoother parameters
* Fixed velocity smoothing; deceleration smoothing wasn't working

Version: 0.1.2 [2013-01-02]
---------------------------
* dynamic reconfigure variable for [[cmd_vel_mux]].
* upgraded to new groovy plugin formats.

Version: 0.1.1 [2012-12-22]
---------------------------
* additional smoother constraints - max. bound and heading

Version: 0.1.0 [2012-12-15] 
---------------------------
* First dubious release (aren't they all?).
