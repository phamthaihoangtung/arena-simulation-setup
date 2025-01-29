import os
from launch import LaunchDescription
from launch.actions import (GroupAction, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import PushRosNamespace, SetRemap, Node
from launch_ros.substitutions import FindPackageShare

from arena_bringup.substitutions import LaunchArgument, YAMLFileSubstitution, YAMLReplaceSubstitution, YAMLMergeSubstitution, YAMLRetrieveSubstitution

def generate_launch_description():
    ss_root = FindPackageShare('arena_simulation_setup')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    robot = LaunchArgument('robot')
    namespace = LaunchArgument('namespace')
    frame = LaunchArgument('frame')
    use_sim_time = LaunchArgument('use_sim_time')
    global_planner = LaunchArgument('global_planner')
    local_planner = LaunchArgument('local_planner')

    substitutions = YAMLMergeSubstitution(
        YAMLFileSubstitution(
            PathJoinSubstitution([
                ss_root,
                'configs',
                'nav2',
                'model_params.yaml'
            ])
        ),
        YAMLFileSubstitution(
            PathJoinSubstitution([
                ss_root,
                'configs',
                'nav2',
                'temp_params.yaml'
            ]),
            default={},
        ),
        YAMLFileSubstitution(
            PathJoinSubstitution([
                ss_root,
                'entities',
                'robots',
                robot.substitution,
                'model_params.yaml'
            ])
        ),
        YAMLFileSubstitution.from_dict(
            {
                'frame': frame.substitution,
            },
            substitute=True
        ),
    )

    substituted_parameters = YAMLReplaceSubstitution(
        obj=YAMLFileSubstitution(
            PathJoinSubstitution([
                ss_root,
                'configs',
                'nav2',
                'nav2.yaml'
            ])
        ),
        substitutions=YAMLFileSubstitution(substitutions)
    )

    robot_base_frame = YAMLRetrieveSubstitution(
        YAMLFileSubstitution(substitutions),
        os.path.join('robot_base_frame'),
    )

    robot_odom_frame = YAMLRetrieveSubstitution(
        YAMLFileSubstitution(substitutions),
        os.path.join('robot_odom_frame'),
    )

    remappings = [
        ('map_server', '/map_server'),
    ]

    bringup_cmd_group = GroupAction([
        *[SetRemap(src=r[0], dst=r[1]) for r in remappings],
        PushRosNamespace(namespace=namespace.substitution),

        # TF publishers
        GroupAction([
            SetRemap(src='/tf_static', dst='/tf'),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odomframe_publisher",
                namespace=namespace.substitution,
                arguments=["0", "0", "0", "0", "0", "0", "map", [frame.substitution, robot_odom_frame]],
                parameters=[use_sim_time.parameter],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="odomframe_to_baseframe_publisher",
                namespace=namespace.substitution,
                arguments=["0", "0", "0", "0", "0", "0", [frame.substitution, robot_odom_frame], [frame.substitution, robot_base_frame]],
                parameters=[use_sim_time.parameter],
            ),
        ]),

        # Localization Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        pkg_nav2_bringup,
                        'launch',
                        'localization_launch.py'
                    ]
                )
            ),
            launch_arguments={
                'namespace': namespace.substitution,
                'use_namespace': 'True',
                'use_sim_time': use_sim_time.substitution,
                'autostart': 'true',
                'params_file': substituted_parameters,
                'use_composition': 'False',
            }.items()
        ),

        # AMCL Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=namespace.substitution,
            parameters=[
                substituted_parameters,
                {'use_sim_time': use_sim_time.substitution}
            ],
            remappings=[
                ('scan', 'scan'),
                ('map', '/map'),
                ('map_metadata', '/map_metadata'),
                ('initialpose', 'initialpose')
            ]
        ),

        # Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        pkg_nav2_bringup,
                        'launch',
                        'navigation_launch.py'
                    ]
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time.substitution,
                'autostart': 'True',
                'params_file': substituted_parameters,
                'use_composition': 'False',
            }.items()
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription([
        robot,
        namespace,
        frame,
        use_sim_time,
        global_planner,
        local_planner,
        bringup_cmd_group,
    ])

    return ld