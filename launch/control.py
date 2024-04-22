# Copyright 2021 iRobot Corporation. All Rights Reserved.
# @author Rodrigo Jose Causarano Nunez (rcausaran@irobot.com)
#
# Launch Create(R) 3 with diffdrive controller in Gazebo and optionally also in RViz.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]


def generate_launch_description():
    this_package = get_package_share_directory('dis_tutorial7')

    namespace = LaunchConfiguration('namespace')

    control_params_file = PathJoinSubstitution(
        [this_package, 'config', 'all_controls_jtc.yaml'])

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,  # Namespace is not pushed when used in EventHandler
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', 'controller_manager'],
        output='screen',
    )

    load_diffdrive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diffdrive_controller'],
        output='screen'
    )

    arm_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,  # Namespace is not pushed when used in EventHandler
        parameters=[control_params_file],
        arguments=['arm_controller', '-c', 'controller_manager'],
        output='screen',
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen',
    )

    # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    arm_controller_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diffdrive_controller_node,
            on_exit=[arm_controller_node]
        )
    )


    # Static transform from <namespace>/odom to odom
    # See https://github.com/ros-controls/ros2_controllers/pull/533
    tf_namespaced_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_odom_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'odom', [namespace, '/odom']],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    # Static transform from <namespace>/base_link to base_link
    tf_namespaced_base_link_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_namespaced_base_link_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   [namespace, '/base_link'], 'base_link'],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('namespace', '')
    )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)
    ld.add_action(arm_controller_callback)
    ld.add_action(tf_namespaced_odom_publisher)
    ld.add_action(tf_namespaced_base_link_publisher)

    return ld
