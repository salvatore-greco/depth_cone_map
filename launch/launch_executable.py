import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    debug = DeclareLaunchArgument("debug", default_value="False")
    config = os.path.join(
        get_package_share_directory("depth_cone_map"), "config", "parameters.yaml"
    )
    node = Node(
        package="depth_cone_map",
        executable="node_executable",
        name="depthConeMapNode",
        output="both",
        remappings=[
            ("/bounding_boxes", "/cone_detection/output"),
            ("/depth_images", "/zed/zed_node/depth/depth_registered"),
            ("/camera_pose", "/orb_slam3/camera_pose"),
            ("/cones", "/slam/cone_map"),
            ("/camera_info", "/zed/zed_node/depth/camera_info"),
            ("/cone_map", "/slam/cone_map"),
            ("/camera_left_image", "/zed/zed_node/left/color/rect/image"),
        ],
        parameters=[config, {"debug": LaunchConfiguration("debug")}],
        prefix="gdbserver localhost:3000",
    )
    return LaunchDescription([debug, node])
