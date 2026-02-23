from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/lidar_points',
                              description='Input point cloud topic from Hesai driver'),
        DeclareLaunchArgument('output_topic', default_value='/velodyne_points',
                              description='Output point cloud topic in Velodyne format'),
        DeclareLaunchArgument('output_frame_id', default_value='velodyne',
                              description='Frame ID for the output point cloud'),
        DeclareLaunchArgument('override_timestamp', default_value='true',
                              description='Override header timestamp with current ROS time (fixes PTP unsync)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulated time from /clock'),

        Node(
            package='hesai_to_velodyne',
            executable='hesai_to_velodyne_node',
            name='hesai_to_velodyne',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'override_timestamp': LaunchConfiguration('override_timestamp'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
