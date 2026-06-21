from setuptools import find_packages
from setuptools import setup

setup(
    name='status_interface',
    version='0.0.0',
    packages=find_packages(
        include=('status_interface', 'status_interface.*')),
)
