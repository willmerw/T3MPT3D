import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 1. IMPORT NODE HERE
from launch_ros.actions import Node

def generate_launch_description():
    # Get the paths to your packages
    gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    t3mpt3d_pkg = get_package_share_directory('t3mpt3d')

    # Path to the Gazebo multi-launch file
    gazebo_multi_launch = os.path.join(gazebo_pkg, 'launch', '/r7021e_v2/turtlebot3_simulations/turtlebot3_gazebo/launch/multi_robot_world.launch.py')

    # Path to the SLAM/Nav launch file
    slam_nav_launch = os.path.join(t3mpt3d_pkg, 'launch', '/r7021e_v2/project/T3MPT3D/src/t3mpt3d/launch/plank_retrieval.launch.py')

    # --- INCLUDES ---
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_multi_launch)
    )

    nav_tb3_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_nav_launch),
        launch_arguments={
            'namespace': 'tb3_1',
            'use_sim_time': 'true'   # <--- ADD THIS
        }.items()
    )

    # Launch SLAM and Navigation for Robot 2
    nav_tb3_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_nav_launch),
        launch_arguments={
            'namespace': 'tb3_2',
            'use_sim_time': 'true'   # <--- ADD THIS
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
        parameters=[{'use_sim_time': True}] # Good practice if relying on Gazebo time
    )
    pnk_node = Node(
            package='t3mpt3d',                  # Replace with your package name
            executable='plank_node',      # Replace with your executable name
            name='plank_node',
            output='screen',
            parameters=[{'use_sim_time': True}] # Good practice if relying on Gazebo time
        )

    # 3. ADD THE NODE TO THE RETURN LIST
    return LaunchDescription([
        simulation,
        nav_tb3_1,
        nav_tb3_2,
        mission_node,
        #pnk_node   # <--- Added here
    ])