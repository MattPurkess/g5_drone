from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'drone_control'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models', 'apriltag_landing_pad'), 
         glob('../../models/apriltag_landing_pad/model.*')),
        (os.path.join('share', package_name, 'models', 'apriltag_landing_pad', 'materials', 'textures'), 
         glob('../../models/apriltag_landing_pad/materials/textures/*')),
        (os.path.join('share', package_name, 'models', 'x500_downward_cam'), 
         glob('../../models/x500_downward_cam/model.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'worlds', 'meshes'),
         glob('worlds/meshes/*')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
        (os.path.join('share', package_name, 'models', 'x500_depth_survey'),
         glob('models/x500_depth_survey/*.sdf') +
         glob('models/x500_depth_survey/*.config')),
        (os.path.join('share', package_name, 'models', 'x500_depth_survey', 'meshes'),
         glob('models/x500_depth_survey/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='MattPurkess@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'takeoff_land = drone_control.takeoff_land:main',
            'mapping_survey = drone_control.mapping_survey:main',
            'circle_flight = drone_control.circle_flight:main',
            'waypoint_nav = drone_control.waypoint_nav:main',
            'apriltag_search_pixels = drone_control.apriltag_search_pixels:main',
            'precision_landing_pixels = drone_control.precision_landing_pixels:main',
            'apriltag_search_pose = drone_control.apriltag_search_pose:main',
            'precision_landing_pose = drone_control.precision_landing_pose:main',
            'follow = drone_control.follow:main',
            'lidar_to_obstacle = drone_control.lidar_to_obstacle:main',
            'avoidance_test = drone_control.avoidance_test:main',
            'analyse_landing = drone_control.analyse_landing:main',
        ],
    },
)
