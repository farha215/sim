from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
     
    controller_node = Node(
    	package='hydrogen',
    	executable='controller_node',
    	name='controller_node',
    	output='screen',
    )
     
    imu_node = Node(
        package='hydrogen',
        executable='imu_node',
        name='imu_node',
        output='screen',
    )

    dhshot_publisher = Node(
        package='hydrogen',
        executable='dshot_node',
        name='dshot_node',
        output='screen',
    )

    gamepad_publisher= Node(
        package='hydrogen',
        executable='thruster_teleop_GP',
        name='thruster_teleop_GP',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(controller_node)
    ld.add_action(imu_node)
    ld.add_action(dhshot_publisher)
    ld.add_action(gamepad_publisher)

    return ld
