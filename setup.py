#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['rqt_qml'],
        package_dir={'': 'src'},
        scripts=['scripts/rqt_qml']
)

setup(**d)
