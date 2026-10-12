import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Get the directory of the custom package
    share_dir = get_package_share_directory('t3mpt3d')

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(share_dir, 'launch', 'rviz', 'exploration.rviz')

    # Launch Configurations
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    other_target_topic = LaunchConfiguration('other_target_topic')

    # 1. Declare Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace for the robot')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.')

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(share_dir, 'config', 'slam_config.yaml'),
        description='Full path to the ROS 2 parameters file to use for the slam_toolbox node')

    declare_other_target_cmd = DeclareLaunchArgument(
        'other_target_topic', default_value='other_target_topic',
        description='Topic with the other robot\'s target (absolute, e.g. /tb3_1/my_target_topic)')

    # 2. Custom Nodes and RViz
    custom_nodes = GroupAction([
        PushRosNamespace(namespace),

        # Node(
        #     package='t3mpt3d',
        #     executable='frontier_detector_node',
        #     name='frontier_detector',
        #     parameters=[{'use_sim_time': use_sim_time}],
        # ),
         Node(
             package='t3mpt3d',
             executable='navigation_node',
             name='navigation_node',
             parameters=[{'use_sim_time': use_sim_time,
                        'base_frame': [namespace, '/base_footprint'],
                        'other_target_topic': other_target_topic}],
             remappings=[('map', '/map'), ('frontiers', '/frontiers')],
             output='screen'
         ),
         Node(
             package='t3mpt3d',
             executable='path_follower_node',
             name='path_follower_node',
             parameters=[{'use_sim_time': use_sim_time,
                          'base_frame': [namespace, '/base_footprint']}],
         ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     arguments=['-d', rviz_config_file],
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     remappings=[
        #         ('/tf', '/tf'),
        #         ('/tf_static', '/tf_static')
        #     ],
        # ),
    ])

    # 3. SLAM Toolbox Lifecycle Node Setup
    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time,

            # These fix the TF frames (Internal math)
            'odom_frame': [namespace, '/odom'],
            'base_frame': [namespace, '/base_footprint'],
            'map_frame': [namespace, '/map'],
            'scan_topic': ['/', namespace, '/scan'],
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        # --- UPDATE THIS SECTION ---
        remappings=[
            ('/map', ['/', namespace, '/map']),
            ('/map_metadata', ['/', namespace, '/map_metadata']),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ],
        # ---------------------------
    )
    # 4. Lifecycle Events for SLAM
    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    # 5. Build and return the unified LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_other_target_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)

    ld.add_action(custom_nodes)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld