from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import yaml


def _load_yaml_file(package_name: str, relative_path: str):
    pkg_share = FindPackageShare(package_name)
    file_path_subst = PathJoinSubstitution([pkg_share, relative_path])
    # Note: The Substitution needs to be resolved at runtime; we use a small trick by
    # evaluating the path inside a lambda passed to a parameter value later.
    return file_path_subst


def generate_launch_description():
    # Launch arguments
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link')
    tcp_frame_arg = DeclareLaunchArgument('tcp_frame', default_value='tool0')
    samples_arg = DeclareLaunchArgument('samples', default_value='10')

    dt_arg = DeclareLaunchArgument('dt', default_value='0.5')
    planning_group_arg = DeclareLaunchArgument('planning_group', default_value='ur_manipulator')
    controller_ns_arg = DeclareLaunchArgument('controller_ns', default_value='/joint_trajectory_controller')

    # MoveIt / robot model inputs
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur3', description='UR robot type (e.g., ur3, ur3e)')
    kin_yaml_arg = DeclareLaunchArgument(
        'kinematics_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'), 'config', 'kinematics.yaml'
        ]),
        description='Path to MoveIt kinematics.yaml'
    )

    # URDF via xacro (typical UR package layout). Users can override if different.
    urdf_xacro_arg = DeclareLaunchArgument(
        'urdf_xacro',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro'
        ]),
        description='Path to UR xacro file'
    )

    # Build robot_description via xacro command
    robot_description = {
        'robot_description': Command([
            'xacro ', LaunchConfiguration('urdf_xacro'),
            ' ur_type:=', LaunchConfiguration('ur_type')
        ])
    }

    # Load kinematics.yaml into a dict under robot_description_kinematics
    # We cannot resolve the path at import time, so we use a small runtime loader
    def load_kinematics_yaml(context):
        kin_path = LaunchConfiguration('kinematics_yaml').perform(context)
        with open(kin_path, 'r') as f:
            kin = yaml.safe_load(f)
        return kin

    # Nodes
    capture_node = Node(
        package='ur3_trajectory_senders',
        executable='capture_interpolate_node',
        name='capture_interpolate_node',
        output='screen',
        parameters=[
            {'base_frame': LaunchConfiguration('base_frame')},
            {'tcp_frame': LaunchConfiguration('tcp_frame')},
            {'samples': LaunchConfiguration('samples')},
        ]
    )

    # Wrap IK node with an OpaqueFunction-like parameter evaluation via on_execute
    # But simpler: we use a lambda in parameters; launch will evaluate on runtime
    class KinematicsParam(dict):
        # a tiny wrapper so launch doesn't try to serialize until runtime
        def __init__(self, key):
            super().__init__()
            self._key = key
        def perform(self, context):
            return {self._key: load_kinematics_yaml(context)}

    ik_node = Node(
        package='ur3_trajectory_senders',
        executable='ik_queue_executor_node',
        name='ik_queue_executor_node',
        output='screen',
        parameters=[
            robot_description,
            KinematicsParam('robot_description_kinematics'),
            {'dt': LaunchConfiguration('dt')},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'controller_ns': LaunchConfiguration('controller_ns')},
        ]
    )

    return LaunchDescription([
        base_frame_arg,
        tcp_frame_arg,
        samples_arg,
        dt_arg,
        planning_group_arg,
        controller_ns_arg,
        ur_type_arg,
        kin_yaml_arg,
        urdf_xacro_arg,
        capture_node,
        ik_node,
    ]) 