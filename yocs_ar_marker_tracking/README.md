## Overview

This package simply collects ar_track_alvar marker output and stores tracking observations and
tracking information in an easy to use container. Some features:

* Throws away observations that haven't been tracked for a while.
* Observation data includes confidence (frame movement consideration), persistence, stability.

## To 3d Sense or not to 3d Sense?

You can hook up the ar_track_alvar node to the kinect if you wish. This should give you better
stability of results, but keep in mind that it will fail when you move within the sensor's
minimum range (~50cm).

We've found the rgb mode (no 3d sensing) is actually quite good - especially when used with
the paired tracking algorithm (see alternative package), so that is what the roslaunchers use.

## Usage

### Launching

```
# openni and ar_track_alvar
> roslaunch yocs_ar_marker_tracking machinery.launch
> roslaunch yocs_ar_marker_tracking tracking.launch
```

### Debugging

Fire up `rqt_logger_level` and set the node's logger level to debug. You'll get full
ar marker information and generated statistics as well.

## Testing

There is a test script that can be used for playing around/testing the ar marker output. It was
designed with the paired tracking in mind, but it is useful for checking out the difference
between kinect and no kinect.

This is used with a left marker id 3, right marker id 0, marker size 10.0 (can change these
in the launchers).

```
> roslaunch yocs_ar_marker_tracking testies-3dsensor.launch
# OR hooking the 3d sensor up to the ar_track_alvar node...
> roslaunch yocs_ar_marker_tracking testies-3dsensor-depth.launch
# AND
> rosrun yocs_ar_marker_tracking testies.py
```

### Understanding the Script

https://docs.google.com/a/yujinrobot.com/drawings/d/1zEwZM1QMsz9F_NJ6jWP-5vxME7HIXXL_2q45sDt81_w/edit?usp=sharing

The calculated target x, z and heading is relative to the camera rgb pose (z forward, x to the right).
In the script I've set the target location 40cm in front of the wall and inbetween the markers.
