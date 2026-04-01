from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'driver_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammadahsan',
    maintainer_email='ahsanakhlaque@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'Tugbots_Positions_Extractor_Node = driver_nodes.Tugbots_Positions_Extractor_Node:main',
        'Tugbots_Bearings_Extractor_Node = driver_nodes.Tugbots_Bearings_Extractor_Node:main',
        'Tugbot_1_Controller_Node = driver_nodes.Tugbot_1_Controller_Node:main',
        'Tugbot_1_Second_Floor_Patrol = driver_nodes.Tugbot_1_Second_Floor_Patrol:main',
        'Tugbot_2_Controller_Node = driver_nodes.Tugbot_2_Controller_Node:main',
        'Tugbot_3_Controller_Node = driver_nodes.Tugbot_3_Controller_Node:main',
        'LiDAR_Readings_Extractor_Node = driver_nodes.LiDAR_Readings_Extractor_Node:main',
        'CAN_Data_Decoder_Node = driver_nodes.CAN_Data_Decoder_Node:main',
        'test = driver_nodes.test:main',
        ],
    },
)

