from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_robot_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    	('share/ament_index/resource_index/packages',
        	['resource/' + package_name]),
    	('share/' + package_name, ['package.xml']),
    	(os.path.join('share', package_name, 'launch'),
    	    glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mirinda',
    maintainer_email='mirinda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts':[
		'click_control_node = my_robot_interface.click_control_node:main',
	],
    },
)
