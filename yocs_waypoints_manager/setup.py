#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['yocs_waypoints_manager'],
    package_dir={'': 'src'},
    requires=['yocs_msgs', 
              'rospy', 
              'nav_msgs',
             ]
)

setup(**d)
