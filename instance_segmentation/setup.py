from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'instance_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pth')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'torch', 'opencv-python', 'detectron2', 'cv_bridge', 'rclpy'],
    zip_safe=True,
    maintainer='appuser',
    maintainer_email='appuser@todo.todo',
    description='ROS 2 package for Detectron2 instance segmentation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'instance_segmentation_node = instance_segmentation.instance_segmentation_node:main',
        ],
    },
)
