from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('mechaknights_2027')
    eurobot_nav_dir = get_package_share_directory('eurobot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    models_path = os.path.join(pkg_share, 'models')
    urdf_path = os.path.join(pkg_share, 'models', 'simple_robot', 'simple_robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'arena_world.sdf')
    rviz_config = os.path.join(pkg_share, 'config', 'arena.rviz')

    # --- Environment ---
    set_resource_path    = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', models_path)
    set_libgl_sw         = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    set_gallium_driver   = SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe')
    set_mesa_override    = SetEnvironmentVariable('MESA_LOADER_DRIVER_OVERRIDE', 'llvmpipe')

    # --- Simulation (t=0) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': open(urdf_path).read()},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen'
    )

    use_teleop = DeclareLaunchArgument(
        'teleop', default_value='true', choices=['true', 'false'],
        description='Open the keyboard teleop window (xterm)'
    )

    marker_localization = DeclareLaunchArgument(
        'marker_localization', default_value='true', choices=['true', 'false'],
        description='Correct the robot position with the fixed table markers (IDs 20-23)'
    )

    team_color = DeclareLaunchArgument(
        'team_color', default_value='yellow', choices=['yellow', 'blue'],
        description='Team colour: sets the start area and the strategy side'
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'robot_spawn.launch.py')),
        launch_arguments={'team_color': LaunchConfiguration('team_color'),
                          'marker_localization': LaunchConfiguration('marker_localization')}.items()
    )

    navigation_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_nav_dir, 'launch', 'bridges.launch.py')
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- Navigation (t=5s) ---
    scan_frame_fixer = Node(
        package='eurobot_navigation',
        executable='scan_frame_fixer',
        name='scan_frame_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # SLAM (t=7s)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eurobot_nav_dir, 'launch', 'online_async_launch.py')
        )
    )

    # Nav2 (t=10s)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(eurobot_nav_dir, 'config', 'nav2_params.yaml')
        }.items()
    )

    # Teleop (t=13s)
    teleop = Node(
        package='eurobot_navigation',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen',
        condition=IfCondition(LaunchConfiguration('teleop'))
    )

    # Stone perception: ArUco ID 13 -> stone positions on the table (t=5s, after camera bridge is up)
    stone_perception = Node(
        package='eurobot_perception',
        executable='stone_perception',
        name='stone_perception',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Camera preview with the detected stones drawn on it (t=6s, after perception node is up)
    image_view = Node(
        package='image_view',
        executable='image_view',
        name='stone_debug_view',
        remappings=[('image', '/stones/debug')],
        output='screen'
    )

    return LaunchDescription([
        # Team colour (ros2 launch ... team_color:=blue)
        team_color,
        marker_localization,
        use_teleop,

        # Environment
        set_resource_path,
        set_libgl_sw,
        set_gallium_driver,
        set_mesa_override,

        # t=0: Simulation
        robot_state_publisher,
        gazebo,
        robot_spawn,
        navigation_bridges,
        rviz,

        # t=5s: scan_frame_fixer
        TimerAction(period=5.0, actions=[scan_frame_fixer]),

        # t=7s: SLAM
        TimerAction(period=7.0, actions=[slam]),

        # t=10s: Nav2
        TimerAction(period=10.0, actions=[nav2]),

        # t=13s: Teleop
        TimerAction(period=13.0, actions=[teleop]),

        # t=5s: Stone perception
        TimerAction(period=5.0, actions=[stone_perception]),

        # t=6s: Camera preview with detections (/stones/debug)
        TimerAction(period=6.0, actions=[image_view]),
    ])
