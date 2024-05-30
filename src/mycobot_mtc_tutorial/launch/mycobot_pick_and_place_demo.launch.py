from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mycobot",package_name="mycobot_moveit_config").to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="mycobot_mtc_tutorial",
        executable="mycobot_mtc_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])