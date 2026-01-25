"""
Full Navigation Launch File

Launches complete Nav2 stack with custom ToF terrain layer.

Usage:
    ros2 launch tmr_costmap navigation.launch.py
    ros2 launch tmr_costmap navigation.launch.py use_slam:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('tmr_costmap')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_pkg_dir = get_package_share_directory('tmr_slam')
    
    # Config files
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'nav2_costmap.rviz')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Use SLAM for mapping (vs static map)'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map file (if not using SLAM)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params,
        description='Path to Nav2 parameters file'
    )
    
    # =========================================================================
    # Nav2 Lifecycle Manager for Navigation
    # =========================================================================
    
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'smoother_server',
            ]
        }]
    )
    
    # =========================================================================
    # Controller Server
    # =========================================================================
    
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odometry/filtered'),
        ]
    )
    
    # =========================================================================
    # Planner Server
    # =========================================================================
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # =========================================================================
    # Behavior Server
    # =========================================================================
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    # =========================================================================
    # BT Navigator
    # =========================================================================
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('odom', '/odometry/filtered'),
        ]
    )
    
    # =========================================================================
    # Smoother Server
    # =========================================================================
    
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file')]
    )
    
    # =========================================================================
    # Costmap Nodes (Local and Global)
    # =========================================================================
    
    # These are included in the controller_server and planner_server
    # but we ensure the custom plugin is loaded
    
    # =========================================================================
    # RViz
    # =========================================================================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # =========================================================================
    # Costmap Diagnostics
    # =========================================================================
    
    costmap_diagnostics = Node(
        package='tmr_costmap',
        executable='costmap_diagnostics.py',
        name='costmap_diagnostics',
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        use_slam_arg,
        map_file_arg,
        rviz_arg,
        params_file_arg,
        
        # Set sim time for all nodes
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        
        # Navigation nodes
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        
        # Lifecycle manager
        nav2_lifecycle_manager,
        
        # Diagnostics
        costmap_diagnostics,
        
        # Visualization
        rviz_node,
    ])
