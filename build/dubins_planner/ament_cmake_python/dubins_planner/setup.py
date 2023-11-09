from setuptools import find_packages
from setuptools import setup

setup(
    name='dubins_planner',
    version='0.0.0',
    packages=find_packages(
        include=('dubins_planner', 'dubins_planner.*')),
)
