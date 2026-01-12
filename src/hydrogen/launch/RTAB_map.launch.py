import os

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

def launch_setup(context: LaunchContext, *args, **kwargs):
    
    # 1. Get configuration from arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    frame_id = LaunchConfiguration('frame_id').perform(context)
    
    # 2. Define parameters
    parameters=[{
        'frame_id': frame_id,
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'approx_sync': False, # Set to True if sim topics have different timestamps
        'use_sim_time': use_sim_time,
        'wait_imu_to_init': False # Disabled as most sims don't sync IMU with standard plugins
    }]

    # 3. Define Remappings
    # These map the rtabmap node inputs to your simulation topics
    remappings=[
        ('rgb/image', rgb_topic),
        ('rgb/camera_info', camera_info_topic),
        ('depth/image', depth_topic)
    ]

    return [
        # Node 1: Sync rgb/depth/camera_info together
        # This creates an 'rgbd_image' topic that RTAB-Map needs
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            remappings=remappings
        ),

        # Node 2: Visual Odometry
        # Computes odometry from the camera images (since we aren't using ZED's internal odom)
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'warn'] # Reduce noise
        ),

        # Node 3: RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'] # Delete previous database on start
        ),

        # Node 4: Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        
        # Simulation time is crucial for Gazebo/Ignition
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Topic Arguments (Change default_value to match your sim if you want)
        DeclareLaunchArgument(
            'rgb_topic', default_value='/camera/RGB_image_raw/front',
            description='Topic for the raw RGB image'),
            
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth_image_raw/front',
            description='Topic for the depth image'),

        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera_info_front',
            description='Topic for the camera info'),

        DeclareLaunchArgument(
            'frame_id', default_value='zed_camera_front_link',
            description='The TF frame of the camera (Optical frame, Z-forward)'),

        OpaqueFunction(function=launch_setup)
    ])