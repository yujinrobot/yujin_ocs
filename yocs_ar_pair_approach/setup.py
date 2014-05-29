#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['yocs_ar_pair_approach'],
    package_dir={'': 'src'},
#    scripts=['scripts/concert_roles_and_apps',
#             ],
)

setup(**d)
