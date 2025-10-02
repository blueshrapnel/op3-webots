from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    this_package = get_package_share_directory('robocup_behavior')

    return LaunchDescription([
        Node(
            package='robocup_behavior',
            executable='bt',
            name='robocup_behavior_bt',
            output='screen',
            parameters=[
                os.path.join(this_package, 'config', 'bt.yaml'),
                {'use_sim_time': True},
                {'behavior': 'test_walk'},
            ]
        )
    ])