from setuptools import find_packages
from setuptools import setup

setup(
    name='sum_srv_interface',
    version='0.0.0',
    packages=find_packages(
        include=('sum_srv_interface', 'sum_srv_interface.*')),
)
