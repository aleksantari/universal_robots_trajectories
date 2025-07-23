from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Allow override of kinematics.yaml; defaults to the one in ur_moveit_config
    declare_kin_yaml = DeclareLaunchArgument(
        'kinematics_yaml',
        default_value=PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'),
            'config',
            'kinematics.yaml'
        ]),
        description='Path to MoveIt kinematics.yaml'
    )

    ik_node = Node(
        package='ur3_trajectory_senders',
        executable='ik_solver_node',
        name='ik_solver_node',
        output='screen',
        parameters=[LaunchConfiguration('kinematics_yaml')]
    )

    return LaunchDescription([
        declare_kin_yaml,
        ik_node
    ])
