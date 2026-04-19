from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
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
                parameters= [{
                    "world_frame": "map",
                    "camera_frame": "zed_camera_link",
                    "percentile": float(0.2)
                }]
            )
        ],
        output = 'both'
    )
    return LaunchDescription([container])