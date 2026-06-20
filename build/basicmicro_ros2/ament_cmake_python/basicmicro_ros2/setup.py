from setuptools import find_packages
from setuptools import setup

setup(
    name='basicmicro_ros2',
    version='1.0.0',
    packages=find_packages(
        include=('basicmicro_ros2', 'basicmicro_ros2.*')),
)
