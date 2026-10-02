from glob import glob
from setuptools import find_packages, setup


package_name = 'deergo_mqtt_bridge'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),

        (
            'share/' + package_name + '/launch',
            glob('launch/*.launch.py')
        ),

        (
            'share/' + package_name + '/rviz',
            glob('rviz/*.rviz')
        ),

        (
            'share/' + package_name + '/config',
            glob('config/*.yaml')
            + glob('config/*.yml')
            + glob('config/*.lua')
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='helson',
    maintainer_email='hanson5977299@gmail.com',

    description=(
        'DeerGo MQTT bridge, Cartographer 2D SLAM '
        'and Nav2 navigation package'
    ),

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'mqtt_bridge = deergo_mqtt_bridge.mqtt_bridge:main',
            'scan_bridge = deergo_mqtt_bridge.scan_bridge:main',
            'map_manager = deergo_mqtt_bridge.map_manager:main',

            # Enable after cartographer_odom_bridge.py is added:
            # 'cartographer_odom_bridge = '
            # 'deergo_mqtt_bridge.cartographer_odom_bridge:main',
        ],
    },
)