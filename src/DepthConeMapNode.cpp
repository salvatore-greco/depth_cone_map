#include "depth_cone_map/DepthConeMapNode.hpp"


DepthConeMapNode::DepthConeMapNode(const rclcpp::NodeOptions &options) : Node("depth_cone_map", options) {
    parameterDeclaration();
    ros_handler = std::make_unique<RosHandler>(this, *this);
    image_processor = std::make_unique<ImageProcessor>(this->get_logger());
    image_transformer = std::make_unique<ImageTransformer>(
        this->get_clock(), 
        this->get_parameter("world_frame").as_string(), 
        this->get_parameter("camera_frame").as_string(), 
        this->get_logger()
    );
}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr &bounding_boxes,
                                const sensor_msgs::msg::Image::ConstSharedPtr &depth_image,
                                const driverless_msgs::msg::PoseStamped::ConstSharedPtr &camera_pose) {
    RCLCPP_INFO(this->get_logger(), "Received messages");
    const MessageContainer messages = MessageContainer(bounding_boxes, depth_image, camera_pose);
    const auto bounding_boxes_list = image_processor->getBBInJSON(messages);
    const auto cones = image_processor->coneFinder(messages, bounding_boxes_list);
    auto marker_array_cones = image_transformer->cameraToWorld(cones);
    ros_handler->publish_cones(std::move(marker_array_cones));
}

void DepthConeMapNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info){
    image_processor->saveKMatrixAsCvMat(camera_info->k);
    ros_handler->camera_info_unsubscribe();
}
void DepthConeMapNode::parameterDeclaration() {
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("camera_frame", "zed_left_camera_frame_optical");
}
RCLCPP_COMPONENTS_REGISTER_NODE(DepthConeMapNode);
