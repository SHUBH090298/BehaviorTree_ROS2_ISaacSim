from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tree = PathJoinSubstitution(
        [FindPackageShare('bt_thesis'), 'trees', 'orchestration.xml'])
    return LaunchDescription([
        Node(package='bt_thesis_agents', executable='perception_agent',
             name='perception_agent', output='screen'),
        Node(package='bt_thesis_agents', executable='skill_agent',
             name='skill_agent', output='screen'),
        Node(package='bt_thesis', executable='bt_executor', name='bt_executor',
             output='screen',
             parameters=[{'tree_file': tree, 'use_sim_time': True,
                          'groot2_port': 1667, 'tick_rate': 20.0}]),
    ])