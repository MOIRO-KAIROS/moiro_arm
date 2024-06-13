from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moiro_arm", package_name="moiro_arm_moveit_config").to_moveit_configs()
    use_sim_time = True
    # MoveGroupInterface demo executable
    move_group_demo = Node(
        package="moiro_arm_move_group",
        executable="moiro_arm_move_group_topic_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],

    )

    return LaunchDescription([move_group_demo])