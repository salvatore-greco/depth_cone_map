#include "depth_cone_map/DepthConeMapNode.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(DepthConeMapNode);
DepthConeMapNode::DepthConeMapNode(): Node("depth_cone_map") {
    ros_handler = std::make_unique<RosHandler>(this->shared_from_this(), *this);
}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr &bounding_boxes,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_image,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr &camera_pose) {
}
