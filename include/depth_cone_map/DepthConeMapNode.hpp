#ifndef DEPTHCONEMAPNODE_DEFINE
#define DEPTHCONEMAPNODE_DEFINE

#include "RosHandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "driverless_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "MessageContainer.hpp"
#include "tf2_ros/buffer.h"
#include <memory>
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>
#include "ImageProcessor.hpp"
#include "ImageTransformer.hpp"
#include "MessageContainer.hpp"
#include <ios>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>

class DepthConeMapNode : public rclcpp::Node {
public:
    explicit DepthConeMapNode(const rclcpp::NodeOptions& options);

    void callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const sensor_msgs::msg::Image::ConstSharedPtr& image_left);

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);
private:
    std::unique_ptr<RosHandler> ros_handler;

    std::unique_ptr<ImageProcessor> image_processor;

    std::unique_ptr<ImageTransformer> image_transformer;

    std::shared_ptr<MessageContainer> message_container;

    //ROS parameter from launch file
    std::string map_frame_name;

    std::string camera_frame_name;

    double percentile;

    bool debug;

    void parameterDeclaration();

    void parameterInitialization();

    void printDebug(const std::list<std::pair<cv::Point, cv::Point>>& bounding_boxes_list, const std::vector<cv::Point3f>& cones, const std::vector<visualization_msgs::msg::Marker>& marker_array_cones);
};

#endif
