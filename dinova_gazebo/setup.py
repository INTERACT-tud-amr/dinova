#!/usr/bin/env python

from os.path import dirname, abspath, basename
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['dinova_gazebo']
)

setup(**setup_args)