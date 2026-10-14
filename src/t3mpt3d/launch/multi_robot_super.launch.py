
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 1. IMPORT NODE HERE
from launch_ros.actions import Node

TB1_NAME = "tb3_1"
TB2_NAME = "tb3_2"

def generate_launch_description():
    # Get the paths to your packages
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    t3mpt3d_pkg = get_package_share_directory('t3mpt3d')

    # Path to the Gazebo multi-launch file
    gazebo_multi_launch = os.path.join(gazebo_pkg, 'launch', 'multi_robot_world.launch.py')

    # Path to the SLAM/Nav launch file
    slam_nav_launch = os.path.join(t3mpt3d_pkg, 'launch', 'plank_retrieval.launch.py')

    # --- INCLUDES ---
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_multi_launch)
    )

    nav_tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_nav_launch),
        launch_arguments={
            'namespace': TB1_NAME,
            'use_sim_time': 'true',   # <--- ADD THIS
            'other_target_topic': f'/{TB2_NAME}/my_target_topic',
            'yields': 'false',
            'stop_on_found': 'true',
            'min_frontiers_cells': '20',
            'plank_index': '0',
            'segment_size': '0.02',
            'plank_rad': '0.05',
        }.items()
    )

    # Launch SLAM and Navigation for Robot 2
    nav_tb3_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_nav_launch),
        launch_arguments={
            'namespace': TB2_NAME,
            'use_sim_time': 'true',   # <--- ADD THIS
            'other_target_topic': f'/{TB1_NAME}/my_target_topic',
            'yields': 'true',
            'stop_on_found': 'true',
            'min_frontiers_cells': '20',
            'plank_index': '1',
            'segment_size': '0.02',
            'plank_rad': '0.05',

        }.items()
    )

    # ==========================================
    # 2. DEFINE YOUR EXTRA NODE HERE
    # ==========================================

    mission_node = Node(
        package='t3mpt3d',                  # Replace with your package name
        executable='mission_control_node',      # Replace with your executable name
        name='mission_control',
        output='screen',
        parameters=[{'use_sim_time': True,
                        "plank_pos1": [2.5, -0.5],
                        "plank_pos2": [2.5,0.5],
                     }] # Good practice if relying on Gazebo time
    )

    map_merge_node = Node(
        package='t3mpt3d',                  # Replace with your package name
        executable='map_merge_node',      # Replace with your executable name
        name='map_merge',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'tb1_map_topic':f'{TB1_NAME}/map',
                     'tb2_map_topic':f'{TB2_NAME}/map',
                     'tb1_odom_world':f'{TB1_NAME}/odom_world',
                     'tb2_odom_world':f'{TB2_NAME}/odom_world',}]
    )

    frontier_detector = Node(
          package='t3mpt3d',
          executable='frontier_detector_node',
          parameters=[{'use_sim_time': True}],
        )

    odom_to_world = Node(
        package='t3mpt3d',
        executable='odom_to_world_node',
        name='odom_to_world_node',
        parameters=[
            {
                'use_sim_time': True,
                'robots': [TB1_NAME, TB2_NAME],
                'target_frame': f'{TB1_NAME}/map',
            }
            ],
        )

    plank_controller_node = Node(
        package='t3mpt3d',
        executable='plank',
        name='plank_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'tb1_cmd_vel': f'{TB1_NAME}/cmd_vel',
                'tb2_cmd_vel': f'{TB2_NAME}/cmd_vel',
                'tb1_odom_world': f'{TB1_NAME}/odom_world',
                'tb2_odom_world': f'{TB2_NAME}/odom_world',
            }
        ],
    )
    plank_planner_node = Node(
            package='t3mpt3d',
            executable='plank_planner',
            name='plank_planner_node',
            output='screen',
            parameters=[
                {'use_sim_time': True,}
            ],
        )
    # Bridge for tb3_1 (Spawned at x=-2.0, y=-0.5)
    tf_world_to_tb3_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_tb3_1',
        # Removed --x, --y, etc. Just pure numbers in order: X, Y, Z, Yaw, Pitch, Roll, Parent, Child
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', f'{TB1_NAME}/map'], #ALWAYS (0,0)
        parameters=[{'use_sim_time': True}]
    )

    # Bridge for tb3_2 (Spawned at x=-2.0, y=0.5)
    tf_world_to_tb3_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_tb3_2',
        arguments=['0.0', '1.0', '0.0', '0.0', '0.0', '0.0', 'world', f'{TB2_NAME}/map'],
        parameters=[{'use_sim_time': True}]
    )
    path_follow = Node(
             package='t3mpt3d',
             executable='path_mpc',
             name='path_mpc',
             output='screen',
             parameters=[{'use_sim_time': True,
                        'tb1_cmd_vel': f'{TB1_NAME}/cmd_vel',
                        'tb2_cmd_vel': f'{TB2_NAME}/cmd_vel',
                        'tb1_odom': f'{TB1_NAME}/odom_world',
                        'tb2_odom': f'{TB2_NAME}/odom_world',
                        'tb1_path': f'{TB1_NAME}/path',
                        'tb2_path': f'{TB2_NAME}/path',
                          }] # Good practice if relying on Gazebo time
    )

    # 3. ADD THE NODE TO THE RETURN LIST
    return LaunchDescription([
        simulation,
        nav_tb3_1,
        nav_tb3_2,
        mission_node,
        map_merge_node,
        tf_world_to_tb3_1, # <--- Added
        tf_world_to_tb3_2,  # <--- Added
        frontier_detector,
        odom_to_world,
        path_follow,
        plank_controller_node,   # <--- Added here
        plank_planner_node,
    ])