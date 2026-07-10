"""Launch the ROS side of the APF stack: obstacle_sensor_node,
apf_planner_node, and optionally RViz.

PX4 SITL / Gazebo / XRCE agent / ros_gz_bridge are infrastructure processes
started by run.sh (tmux panes) - launching them here would bury their logs.

Arguments:
  goal_x, goal_y, goal_z  goal in Gazebo world ENU (default 12, 0, 2)
  source                  obstacle source: fixed | depth (default fixed)
  rviz                    true|false (default false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('apf_oa')
    params = os.path.join(share, 'config', 'apf_params.yaml')
    rviz_cfg = os.path.join(share, 'rviz', 'apf.rviz')

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_z = LaunchConfiguration('goal_z')
    source = LaunchConfiguration('source')

    return LaunchDescription([
        DeclareLaunchArgument('goal_x', default_value='12.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_z', default_value='2.0'),
        DeclareLaunchArgument('source', default_value='fixed'),
        DeclareLaunchArgument('rviz', default_value='false'),

        Node(
            package='apf_oa',
            executable='obstacle_sensor_node',
            name='obstacle_sensor_node',
            output='screen',
            parameters=[params, {'source': source}],
        ),
        Node(
            package='apf_oa',
            executable='apf_planner_node',
            name='apf_planner_node',
            output='screen',
            parameters=[params, {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_z': goal_z,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='log',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
