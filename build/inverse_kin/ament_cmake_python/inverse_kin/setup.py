from setuptools import find_packages
from setuptools import setup

setup(
    name='inverse_kin',
    version='0.0.0',
    packages=find_packages(
        include=('inverse_kin', 'inverse_kin.*')),
)
