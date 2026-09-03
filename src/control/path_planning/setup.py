#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""catkin이 path_planning Python 모듈을 ROS 실행 환경에 설치하도록 설정하는 파일."""

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


setup_args = generate_distutils_setup(
    packages=["path_planning"],
    package_dir={"": "src"},
)

setup(**setup_args)
