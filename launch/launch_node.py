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
                name='DepthConeMapNode'
            )
        ],
        output = 'both'
    )
    return LaunchDescription([container])