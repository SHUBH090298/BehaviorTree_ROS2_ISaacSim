from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tree = LaunchConfiguration('tree')
    return LaunchDescription([
        DeclareLaunchArgument(
            'tree',
            default_value=PathJoinSubstitution(
                [FindPackageShare('bt_thesis'), 'trees', 'orchestration.xml'])),
        Node(
            package='bt_thesis',
            executable='bt_executor',
            name='bt_executor',
            output='screen',
            parameters=[{
                'tree_file': tree,
                'use_sim_time': True,
                'groot2_port': 1667,
                'tick_rate': 20.0,
            }],
        ),
    ])
