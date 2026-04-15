from setuptools import find_packages
from setuptools import setup

setup(
    name='agrobot_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('agrobot_interfaces', 'agrobot_interfaces.*')),
)
