"""
RTAB-MAP SLAM Launch File

Main launch file for RTAB-MAP SLAM on the Terrain Mapping Rover. 

Usage:
    ros2 launch tmr_slam slam.launch.py
    ros2 launch tmr_slam slam.launch.py config:=low_texture
    ros2 launch tmr_slam slam.launch. py rviz:=true
    ros2 launch tmr_slam slam.launch.py localization:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch. conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('tmr_slam')
    
    # Config file paths
    config_default = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    config_low_texture = os.path.join(pkg_dir, 'config', 'rtabmap_low_texture.yaml')
    config_indoor = os.path.join(pkg_dir, 'config', 'rtabmap_indoor.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'rviz_slam. rviz')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='default',
        description='Config profile:  default, low_texture, indoor'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Localization only mode (no mapping)'
    )
    
    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value='',
        description='Path to RTAB-MAP database (empty = memory only)'
    )
    
    delete_db_arg = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='true',
        description='Delete database on start'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # =========================================================================
    # Topic Remappings
    # =========================================================================
    
    remappings = [
        # RGB camera
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        
        # Depth from ToF
        ('depth/image', '/tof/depth/image_raw'),
        
        # Odometry from EKF
        ('odom', '/odometry/filtered'),
        
        # Output map topic
        ('map', '/map'),
        ('grid_map', '/map'),
    ]
    
    # =========================================================================
    # RTAB-MAP Parameters
    # =========================================================================
    
    # Select config file based on argument
    # This is a simplified approach - in practice you'd use conditionals
    rtabmap_parameters = [
        config_default,
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'database_path': LaunchConfiguration('database_path'),
            'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
        }
    ]
    
    # =========================================================================
    # RTAB-MAP Node
    # =========================================================================
    
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=remappings,
        arguments=[
            '--delete_db_on_start', LaunchConfiguration('delete_db_on_start'),
        ],
    )
    
    # =========================================================================
    # RTAB-MAP Visualization Node
    # =========================================================================
    
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=rtabmap_parameters,
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # =========================================================================
    # RGB-D Odometry Node (optional - we use EKF)
    # =========================================================================
    
    # Not used since we have wheel odometry + IMU via EKF
    # Uncomment if you want visual odometry as backup
    
    # rgbd_odom = Node(
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     name='rgbd_odometry',
    #     output='screen',
    #     parameters=rtabmap_parameters,
    #     remappings=remappings,
    # )
    
    # =========================================================================
    # Point Cloud Assembler (optional)
    # =========================================================================
    
    point_cloud_assembler = Node(
        package='rtabmap_util',
        executable='point_cloud_assembler',
        name='point_cloud_assembler',
        output='screen',
        parameters=[{
            'max_clouds': 10,
            'fixed_frame_id': 'map',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('cloud', '/tof/points'),
            ('odom', '/odometry/filtered'),
        ]
    )
    
    # =========================================================================
    # Map Optimizer Node (for periodic optimization)
    # =========================================================================
    
    map_optimizer = Node(
        package='rtabmap_util',
        executable='map_optimizer',
        name='map_optimizer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
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
    # SLAM Diagnostics
    # =========================================================================
    
    slam_diagnostics = Node(
        package='tmr_slam',
        executable='slam_diagnostics. py',
        name='slam_diagnostics',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # =========================================================================
    # Startup Message
    # =========================================================================
    
    startup_msg = LogInfo(
        msg="\n" + "="*60 + "\n" +
            "  TMR Phase 4 - RTAB-MAP SLAM\n" +
            "="*60 + "\n" +
            "  Inputs:\n" +
            "    - RGB:  /camera/image_raw\n" +
            "    - Depth: /tof/depth/image_raw\n" +
            "    - Odometry:  /odometry/filtered (from EKF)\n" +
            "  Outputs:\n" +
            "    - TF: map -> odom\n" +
            "    - /map (2D occupancy grid)\n" +
            "    - /rtabmap/cloud_map (3D point cloud)\n" +
            "="*60
    )
    
    return LaunchDescription([
        # Arguments
        config_arg,
        rviz_arg,
        localization_arg,
        database_path_arg,
        delete_db_arg,
        use_sim_time_arg,
        
        # Info
        startup_msg,
        
        # Nodes
        rtabmap_node,
        rtabmap_viz,
        point_cloud_assembler,
        slam_diagnostics,
        rviz_node,
    ])
