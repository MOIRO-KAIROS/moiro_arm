import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    res = []

    port_launch_arg = DeclareLaunchArgument(
        name="port",
        default_value="/dev/ttyUSB0"
    )
    res.append(port_launch_arg)

    baud_launch_arg = DeclareLaunchArgument(
        name="baud",
        default_value="115200"
    )
    res.append(baud_launch_arg)

    model_launch_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/mycobot.xacro"
        )
    )
    res.append(model_launch_arg)

    rvizconfig_launch_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=os.path.join(
            get_package_share_directory("mycobot_320"),
            "config/mycobot_pro_320.rviz"
        )
    )
    res.append(rvizconfig_launch_arg)

    num_launch_arg = DeclareLaunchArgument(
        name="num",
        default_value="0",
    )
    res.append(num_launch_arg)

    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                LaunchConfiguration('model')
            ]
        ),
        value_type=str
    )

    robot_state_publisher_node = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        arguments=[LaunchConfiguration("model")]
    )
    res.append(robot_state_publisher_node)
    
    cam_topic_cmd = DeclareLaunchArgument(
        "input_image_topic",
        default_value="/camera/camera/color/image_raw",
        description="Name of the input image topic",
        )
    res.append(cam_topic_cmd)

    realsense_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("realsense2_camera"), '/launch/rs_launch.py']),
        launch_arguments=[
            ('depth_module.profile', '640x480x60'),
            ('rgb_camera.profile', '640x480x60'),
            ('pointcloud.enable', 'true')]
        )
    res.append(realsense_launch_cmd)

    cube_pixel_node = Node(
        name="cube_detect",
        package="cube_detect",
        executable="cube_pixel",
        parameters=[{'input_image_topic' : LaunchConfiguration("input_image_topic")}]
    )
    res.append(cube_pixel_node)

    cube_world_node = Node(
        name="cube_detect",
        package="cube_detect",
        executable="cube_world",
        parameters=[{'input_image_topic' : LaunchConfiguration("input_image_topic")}]
    )
    res.append(cube_world_node)

    rviz_node = Node(
        name="rviz2",
        package="rviz2",
        executable="rviz2",
        # output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    res.append(rviz_node)
    
    # real_listener_node = Node(
    #     name="listen_real_of_topic",
    #     package="mycobot_320",
    #     executable="listen_real_of_topic"
    # )
    # res.append(real_listener_node)

    return LaunchDescription(res)
