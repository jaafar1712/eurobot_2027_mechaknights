"""
Spawns the robot in its team's throne room and sets up the `table` frame.

Included by arena.launch.py and full_system.launch.py, which pass team_color.

    table ──(table_localizer)──> map ──(SLAM)──> odom ──(wheels)──> base_footprint

`table` is the rule-book frame (origin at the yellow front-left corner). SLAM
starts its `map` frame where the robot starts, so table -> map starts at the
start pose; table_localizer then corrects it whenever the camera sees one of the
playmat's fixed markers (marker_localization:=false keeps it fixed).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from eurobot_navigation import field

WORLD_NAME = 'eurobot_2027_arena'


def spawn(context):
    team = field.check_team(LaunchConfiguration('team_color').perform(context))
    x, y, yaw = field.start_pose(team)
    model_sdf = os.path.join(get_package_share_directory('mechaknights_2027'),
                             'models', 'simple_robot', 'model.sdf')
    pose = ['--x', str(x), '--y', str(y), '--z', '0', '--yaw', str(yaw)]

    return [
        LogInfo(msg=f'Team {team}: robot starts at x={x:.2f} y={y:.2f} yaw={yaw:.2f}'),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-world', WORLD_NAME, '-file', model_sdf, '-name', 'simple_robot',
                       '-x', str(x), '-y', str(y), '-z', '0.015', '-Y', str(yaw)],
            output='screen',
        ),
        # table -> map: corrected with the playmat's fixed ArUco markers, or fixed at the start pose
        Node(
            package='eurobot_navigation',
            executable='table_localizer',
            name='table_localizer',
            parameters=[{'use_sim_time': True, 'start_x': x, 'start_y': y, 'start_yaw': yaw}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('marker_localization')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='table_to_map',
            arguments=pose + ['--frame-id', 'table', '--child-frame-id', 'map'],
            parameters=[{'use_sim_time': True}],
            condition=UnlessCondition(LaunchConfiguration('marker_localization')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('team_color', default_value='yellow',
                              choices=list(field.TEAMS),
                              description='Team colour: sets the start area and the strategy side'),
        DeclareLaunchArgument('marker_localization', default_value='true',
                              description='Correct the robot position with the fixed table markers (IDs 20-23)'),
        DeclareLaunchArgument('publish_map_to_odom', default_value='false',
                              description='Publish an identity map -> odom when SLAM is not running'),
        OpaqueFunction(function=spawn),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('publish_map_to_odom')),
        ),
        Node(
            package='eurobot_navigation',
            executable='field_markers',
            name='field_markers',
            output='screen',
        ),
    ])
