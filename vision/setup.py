from setuptools import setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ash',
    maintainer_email='ash.masame@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laneDetection = vision.opencv_laneDetection:main', 
            'img_publisher = vision.test_img_publisher:main',
            'mask_rgb = vision.laneMasker:main',
            'mask_depth = vision.depthMasker:main',
            
            ],
    },
)
