from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    debug = DeclareLaunchArgument(
        "debug",
        default_value='False'
    )
    config = os.path.join(
        get_package_share_directory('depth_cone_map'),
        'config',
        'parameters.yaml'
    )
    container = ComposableNodeContainer(
        name = 'depth_cone_map_node_container',
        namespace = '',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_cone_map',
                plugin='DepthConeMapNode',
                name='DepthConeMapNode',
                remappings=[
                    ("/bounding_boxes", "/cone_detection/output"),
                    ("/depth_images", "/zed/zed_node/depth/depth_registered"),
                    ("/camera_pose", "/orb_slam3/camera_pose"),
                    ("/cones", "/slam/cone_map"),
                    ("/camera_info", "/zed/zed_node/depth/camera_info"),
                    ("/cone_map", "/slam/cone_map")
                ],
                parameters=[config, {"debug": LaunchConfiguration("debug")}]
            )
        ],
        output = 'both'
    )
    return LaunchDescription([debug, container])