import launch
import os
import sys

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

def get_robot_description():
    joint_limit_params = PathJoinSubstitution([FindPackageShare('mycobot_moveit'), 'config', 'mycobot_joint_limits.yaml'])
    kinematics_params = PathJoinSubstitution([FindPackageShare('mycobot_moveit'), 'config', 'mycobot_kinematics.yaml'])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution([FindPackageShare('mycobot_moveit'), 'config', 'mycobot.urdf.xacro']),
            " ",
            "joint_limits_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params
        ]
    )
    robot_description = {
        'robot_description': robot_description_content
    }
    return robot_description

def get_robot_description_semantic():
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            " ",
            PathJoinSubstitution([FindPackageShare('mycobot_moveit'), 'config', 'mycobot.srdf.xacro'])
        ]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }
    return robot_description_semantic

def generate_launch_description():
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()
    pick_and_place_node = Node(
        package='mycobot_moveit',
        executable='pick_and_place',
        name='pick_and_place',
        output='screen',
        parameters=[robot_description, robot_description_semantic]
    )

    return launch.LaunchDescription([
        pick_and_place_node
    ])