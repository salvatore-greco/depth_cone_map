#include "depth_cone_map/DepthConeMapNode.hpp"

#include "depth_cone_map/MessageContainer.hpp"

// TODO: togliere pose che tanto non serve e usare message container come membro delle classi
DepthConeMapNode::DepthConeMapNode(const rclcpp::NodeOptions& options) : Node("depth_cone_map", options) {
    parameterDeclaration();
    parameterInitialization();
    message_container = std::make_shared<MessageContainer>();
    ros_handler = std::make_unique<RosHandler>(this, *this);
    image_processor = std::make_unique<ImageProcessor>(this->get_logger(), this->percentile, message_container);
    image_transformer = std::make_unique<ImageTransformer>(this->get_clock(), this->map_frame_name,
                                                           this->camera_frame_name, this->get_logger());
    keyframe_handler = std::make_shared<KeyframeHandler>(std::make_unique<TemporalKeyframeStrategy>(std::chrono::seconds(1))); //è shared ptr in caso volessi mettere superpoint-glue in un thread
    //TODO: fare switch per istanziare la strategy da parametro launch (+ settare parametro tempo)
    // TODO: implementa l'altra strategia
}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const sensor_msgs::msg::Image::ConstSharedPtr& image_left, const driverless_msgs::msg::PoseStamped::ConstSharedPtr& pose) {

    RCLCPP_INFO(this->get_logger(), "Received messages");
    this->message_container->saveMessages(bounding_boxes, depth_image, image_left);
    RCLCPP_INFO(this->get_logger(), "after saving messages");
    const auto bounding_boxes_list = image_processor->getBBInJSON();
    const auto cones = image_processor->getConeInCameraFrame(bounding_boxes_list);
    auto marker_array_cones = image_transformer->cameraToWorld(cones);
    if (debug) {
        printDebug(bounding_boxes_list, cones, marker_array_cones.markers);
    }
    ros_handler->publish_cones(std::move(marker_array_cones));
}

void DepthConeMapNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    image_processor->saveKMatrixAsCvMat(camera_info->k);
    ros_handler->camera_info_unsubscribe();
}

void DepthConeMapNode::parameterDeclaration() {
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("camera_frame", "zed_left_camera_optical_frame");
    this->declare_parameter("percentile", 0.2);
    this->declare_parameter("debug", false);
}

void DepthConeMapNode::parameterInitialization() {
    map_frame_name = this->get_parameter("world_frame").as_string();
    camera_frame_name = this->get_parameter("camera_frame").as_string();
    percentile = this->get_parameter("percentile").as_double();
    debug = this->get_parameter("debug").as_bool();
}

void DepthConeMapNode::printDebug(const std::list<std::pair<cv::Point, cv::Point>>& bounding_boxes_list,
                                  const std::vector<cv::Point3f>& cones,
                                  const std::vector<visualization_msgs::msg::Marker>& marker_array_cones) {
    std::ostringstream ss;
    ss << "JSON parsed: [\n";
    for (const auto& point : bounding_boxes_list) {
        ss << point.first << ", " << point.second << "\n";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    ss.str("");
    ss.clear();

    ss << "Cones in camera frame: [\n";
    for (const auto& cone_camera : cones) {
        ss << cone_camera << "\n";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    ss.str("");
    ss.clear();

    ss << "Cones in world frame, size " << marker_array_cones.size() << ": [\n";
    for (const auto& cone_world : marker_array_cones) {
        ss << "[" << cone_world.pose.position.x << "," << cone_world.pose.position.y << ", "
           << cone_world.pose.position.z << "],";
    }
    ss.seekp(-1, std::ios_base::end);  // mi cancella l'ultima virgola
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

RCLCPP_COMPONENTS_REGISTER_NODE(DepthConeMapNode);

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthConeMapNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
