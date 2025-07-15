from setuptools import find_packages, setup

package_name = 'inverse_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emily',
    maintainer_email='edial@wpi.edu',
    description='inverse kinematics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = inverse_kinematics.service_member_function:main',
            'client = inverse_kinematics.client_member_function:main',
        ],
    },
)
