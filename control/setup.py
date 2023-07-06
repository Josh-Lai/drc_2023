from setuptools import setup

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmad',
    maintainer_email='a.abuaysha@uq.net.au',
    description='Outputs speed and steering commands based on lane detection inputs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laneDetection = vision.opencv_laneDetection:main',
            'img_publisher = vision.test_img_publisher:main',
        ],
    },
)
