from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob("launch/*.launch.py")), 
        (os.path.join('share', package_name, 'config'), glob("config/*.yaml")), 


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schninthorn',
    maintainer_email='schninthorn@todo.todo', 
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                    "first_pudlisher = mypkg.first_node:main", 
                    "twist_pudlisher = mypkg.twist_pub:main",
                    "first_subcription = mypkg.first_sub:main", 
                    "turtle_avoidane = mypkg.turtle_control:main",
                    "first_param = mypkg.first_param:main", 
                    "turtle_service_server = mypkg.turtle_service_sever:main",
                    "turtle_service_client = mypkg.turtle_service_client:main",
                    "pid_tuner = mypkg.pid_tuner:main",
                    "robot_core = mypkg.robot_core:main",
        ],
         
    },
)
