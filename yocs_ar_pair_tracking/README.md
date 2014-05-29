## Overview

Parameterised by two AR Markers this package tracks the markers and computes a relative pose
from the robot to the midpoint (of the baseline) between the two markers.

## Why two markers? 

You can do this with a single ar marker - it will supply you with all the information you need -
a full six degrees of position and orientation information. However the orientation data
can often be quite dodgy. Detection rates and the position data on the other hand is quite reliable,
so here we use position data from the two ar markers to provide more robustness than we could
get with ar marker data from one.

## Usage

### Markers

We use left marker id 3 and right marker id 0 by default, but these are configurable as parameters
for the node. 

### Publishers

* `~/spotted_markers` : string identifying which markers are spotted - 'none', 'left', 'right', 'both'.
* `~/initial_pose` : pose with covariance stamped for use to set an initial pose for amcl (assuming 
`/map` is at the base of the midpoint of the two ar markers.
* `~relative_target_pose` : 2d pose between sensor and target location, use for an approach controller.

## Todo

Remove the transform logic inside the node and just publish the relative target pose. This removes
any need for this node to have external information (in this case a camera_rgb_optical_frame to
base_footprint transform). 