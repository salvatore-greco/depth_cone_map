#include "depth_cone_map/DepthConeMapNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>

#include "depth_cone_map/KeyframeHandler.hpp"
#include "depth_cone_map/MessageContainer.hpp"
#include "depth_cone_map/SuperGlueFeatureMatcher.hpp"
#include "depth_cone_map/SuperPointFeatureExtractor.hpp"
#include "depth_cone_map/TemporalKeyframeStrategy.hpp"

DepthConeMapNode::DepthConeMapNode(const rclcpp::NodeOptions& options) : Node("depth_cone_map", options) {
    parameterDeclaration();
    parameterInitialization();
    message_container = std::make_shared<MessageContainer>();
    ros_handler = std::make_unique<RosHandler>(this, *this);
    image_processor = std::make_unique<ImageProcessor>(this->get_logger(), this->percentile, message_container);
    image_transformer = std::make_unique<ImageTransformer>(this->get_clock(), this->map_frame_name,
                                                           this->camera_frame_name, this->get_logger());
    std::cout<<superpointglue_config<<", "<<superpointglue_weight<<std::endl;
    superpoint = std::make_unique<SuperPointFeatureExtractor>(superpointglue_config, superpointglue_weight, this->get_logger(), message_container);
    superglue = std::make_unique<SuperGlueFeatureMatcher>(superpointglue_config, superpointglue_weight, this->get_logger(), message_container);
    keyframe_handler = std::make_shared<KeyframeHandler>(std::make_unique<TemporalKeyframeStrategy>(std::chrono::seconds(1)));
    //TODO: fare switch per istanziare la strategy da parametro launch (+ settare parametro tempo)
    // TODO: implementa l'altra strategia
}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes,
                                const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
                                const sensor_msgs::msg::Image::ConstSharedPtr& image_left) {
    // RCLCPP_INFO(this->get_logger(), "Received messages");
    this->message_container->saveMessages(bounding_boxes, depth_image, image_left);

    const auto bounding_boxes_list = image_processor->getBBInJSON();
    const auto cones = image_processor->getConeInCameraFrame(bounding_boxes_list);
    auto marker_array_cones = image_transformer->cameraToWorld(cones);


    // questa implementazione di superpoint sembra preferire le immagini in bianco e nero con encoding mono8.
    // con mono16 trova pochissime feature
    cv::Mat img = cv_bridge::toCvShare(image_left, "mono8")->image;
    auto val = superglue->getWidthAndHeight();
    // std::cout<<(img.rows /val.second)<<" "<<img.cols / val.first<<std::endl;
    superpoint->setScaleFactorX(static_cast<float>(img.cols) / val.first);
    superpoint->setScaleFactorY(static_cast<float>(img.rows) / val.second);
    cv::resize(img, img, cv::Size(val.first, val.second)); //ridimensionamento per rispettare ciò che si aspetta superpoint; settato in config.yaml
    auto feature = superpoint->getFeatureInBB(img, bounding_boxes_list);
    // RCLCPP_INFO(this->get_logger(), "extracted features");

    if(!do_not_match){
        //FIXME: assicurati venga fatto RVO oppure usa la semantica move; (così viene copiata ogni volta)
        Eigen::Matrix<double, 259, Eigen::Dynamic> kf_feature = keyframe_handler->getCurrentFrameFeature();
        auto matches = superglue->matchFeature(kf_feature, feature);
    }

    if (debug) {
        printDebug(bounding_boxes_list, cones, marker_array_cones.markers);
    }
    if(keyframe_handler->saveKeyframe(image_left, std::move(feature))){
        do_not_match = false;
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
    this->declare_parameter("superpointglue_config", "/");
    this->declare_parameter("superpointglue_weights", "/");
}

void DepthConeMapNode::parameterInitialization() {
    map_frame_name = this->get_parameter("world_frame").as_string();
    camera_frame_name = this->get_parameter("camera_frame").as_string();
    percentile = this->get_parameter("percentile").as_double();
    debug = this->get_parameter("debug").as_bool();
    superpointglue_config = this->get_parameter("superpointglue_config").as_string();
    superpointglue_weight = this->get_parameter("superpointglue_weights").as_string();
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
