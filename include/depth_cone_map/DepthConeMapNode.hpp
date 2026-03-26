#ifndef DEPTHCONEMAPNODE_DEFINE
#define DEPTHCONEMAPNODE_DEFINE 

#include "RosHandler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class DepthConeMapNode : public rclcpp::Node {
public:
    DepthConeMapNode();
    void callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const geometry_msgs::msg::PoseStamped::ConstSharedPtr& camera_pose);
private:
    std::unique_ptr<RosHandler> ros_handler;
};

#endif