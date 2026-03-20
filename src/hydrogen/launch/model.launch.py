import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    package_name = 'hydrogen'
    robot_name = 'Hydrogen'

    pkg_share = get_package_share_directory(package_name)
    
    worlds_path = os.path.join(pkg_share, 'worlds')
    model_path = os.path.join(pkg_share, 'model')

    install_dir = os.path.dirname(pkg_share)

    # ---------------- Launch arguments ----------------
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    # ---------------- Gazebo resource paths ----------------
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([worlds_path, install_dir])
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([worlds_path, install_dir])
    )

    # ---------------- Robot description ----------------
    xacro_file = os.path.join(model_path, 'robot.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ---------------- World ----------------
    world_file = os.path.join( worlds_path, 'buoyant_pool.sdf')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ---------------- Spawn robot ----------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ],
        output='screen'
    )

    # ---------------- Robot State Publisher ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ---------------- ROS <-> Gazebo Bridge ----------------
    bridge_params = os.path.join(
        pkg_share,
        'parameters',
        'bridge_params.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '--param', f'config_file:={bridge_params}'
        ],
        parameters=[{'use_sim_time': True}]
    )
    
    controller_node = Node(
    	package='hydrogen',
    	executable='controller_node',
    	name='controller_node',
    	output='screen',
    	parameters=[{'use_sim_time':True}]
    )    

    # ---------------- Launch Description ----------------
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('x', default_value='3.5', description='Spawn X position'))
    ld.add_action(DeclareLaunchArgument('y', default_value='-21.5', description='Spawn Y position'))
    ld.add_action(DeclareLaunchArgument('z', default_value='2.5', description='Spawn Z position'))
    ld.add_action(DeclareLaunchArgument('R', default_value='0.0', description='Spawn Roll'))
    ld.add_action(DeclareLaunchArgument('P', default_value='0.0', description='Spawn Pitch'))
    ld.add_action(DeclareLaunchArgument('Y', default_value='0.0', description='Spawn Yaw'))

    ld.add_action(set_ign_resource_path)
    ld.add_action(set_gz_resource_path)
    ld.add_action(gazebo_launch)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    

    return ld

