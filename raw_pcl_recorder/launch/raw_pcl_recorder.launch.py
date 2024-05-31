import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000'
        ),
        launch.actions.DeclareLaunchArgument(
            name='topic_name',
            default_value='/sensing/lidar/concatenated/pointcloud'
        ),
        launch.actions.DeclareLaunchArgument(
            name='data_path',
            default_value='/tmp/data'
        ),
        launch_ros.actions.Node(
            package='raw_pcl_recorder',
            executable='raw_pcl_recorder_node',
            name='raw_pcl_recorder_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True
                },
                {
                    'topic_name': launch.substitutions.LaunchConfiguration('topic_name')
                },
                {
                    'data_path': launch.substitutions.LaunchConfiguration('data_path')
                },
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
