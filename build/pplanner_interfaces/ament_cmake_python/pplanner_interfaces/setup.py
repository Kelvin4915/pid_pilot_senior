from setuptools import find_packages
from setuptools import setup

setup(
    name='pplanner_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('pplanner_interfaces', 'pplanner_interfaces.*')),
)
