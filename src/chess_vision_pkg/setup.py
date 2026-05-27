from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chess_vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Bundle the static assets into the final build
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        (os.path.join('share', package_name, 'calibration_data'), glob('calibration_data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='ROS 2 Chess Vision Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_homography = chess_vision_pkg.calibrate_homography:main',
            'vision_node = chess_vision_pkg.vision_node:main'
        ],
    },
)