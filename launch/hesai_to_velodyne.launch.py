from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_type', default_value='XYZIRT',
                              description='Input point cloud type: XYZI or XYZIRT'),
        DeclareLaunchArgument('output_type', default_value='XYZIRT',
                              description='Output point cloud type: XYZI, XYZIR, or XYZIRT'),
        DeclareLaunchArgument('input_topic', default_value='/lidar_points',
                              description='Input point cloud topic from Hesai driver'),
        DeclareLaunchArgument('output_topic', default_value='/velodyne_points',
                              description='Output point cloud topic in Velodyne format'),
        DeclareLaunchArgument('output_frame_id', default_value='velodyne',
                              description='Frame ID for the output point cloud'),

        Node(
            package='hesai_to_velodyne',
            executable='hesai_to_velodyne_node',
            name='hesai_to_velodyne',
            output='screen',
            parameters=[{
                'input_type': LaunchConfiguration('input_type'),
                'output_type': LaunchConfiguration('output_type'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
            }],
        ),
    ])
