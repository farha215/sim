from setuptools import setup
import os
from glob import glob

package_name = 'hydrogen'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),

        # Robot model files
        ('share/' + package_name + '/model', glob('model/*')),

        #plugins
        ('share/' + package_name + '/plugins', glob('plugins/*')),

        # Meshes
        ('share/' + package_name + '/meshes', glob('meshes/*')),

        # World files
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),

        # Subfolders inside worlds/
        ('share/' + package_name + '/worlds/startgate', package_files('worlds/startgate')),
        ('share/' + package_name + '/worlds/pathmarker', package_files('worlds/pathmarker')),
        ('share/' + package_name + '/worlds/red_pole', package_files('worlds/red_pole')),
        ('share/' + package_name + '/worlds/white_pole', package_files('worlds/white_pole')),
        ('share/' + package_name + '/worlds/bruvs', package_files('worlds/bruvs')),
        ('share/' + package_name + '/worlds/pool', package_files('worlds/pool')),
        ('share/' + package_name + '/worlds/buoy', package_files('worlds/buoy')),
        ('share/' + package_name + '/worlds/octagon', package_files('worlds/octagon')),
        ('share/' + package_name + '/worlds/trash', package_files('worlds/trash')),
        ('share/' + package_name + '/worlds/preq_task', package_files('worlds/preq_task')),
        # Parameters
        ('share/' + package_name + '/parameters', ['parameters/bridge_params.yaml']),
        # YOLO model
        ('share/' + package_name, ['hydrogen/prequal.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Animesh Mishra',
    maintainer_email='animeshmishra211@gmail.com',
    description='Hydrogen prototype simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'thruster_teleop = hydrogen.thruster_teleop:main',
            'controller_node = hydrogen.controller_node:main',
            'distance_node = hydrogen.distance_node:main',
            'test_ROI_Publisher = hydrogen.test_ROI_publisher:main',
            'thruster_teleop_GP = hydrogen.thruster_teleop_GP:main',
            'dataset_collector = hydrogen.dataset_collector:main',
            'imu_node = hydrogen.imu_publisher:main',
            'dshot_node = hydrogen.thruster_Dshot_publisher:main',
            'yolo_node = hydrogen.yolo_node:main',
            'data_distance_node = hydrogen.data_distance_node:main',
            'vision_fusion_node = hydrogen.vision_fusion_node:main',
        ],
    },
)

