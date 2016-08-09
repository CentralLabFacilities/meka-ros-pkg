#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rqt_m3dashboard', 'rqt_m3dashboard.widgets', 'rqt_m3dashboard.dialogs'],
    package_dir={'': 'src'}
)

setup(**d)
