from glob import glob

from setuptools import setup


package_name = 'webrtc_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'fastapi'
    ],
    zip_safe=True,
    maintainer='Andreas Bresser',
    maintainer_email='self@andreasbresser.de',
    description='ROS 2 WebRTC Bridge',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webrtc_bridge = webrtc_bridge.webrtc_node:main'
        ],
    },
)
