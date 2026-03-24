from setuptools import find_packages, setup

package_name = 'control_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='assemblex',
    maintainer_email='aditya.jemshetty@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'allocation_matrix = control_system.allocation_matrix:main',
            'global_planner = control_system.global_planner_node:main',
            '6dof_pid = control_system.6dof_pid:main',
            'los_controller = control_system.los_control:main',
        ],
    },
)
