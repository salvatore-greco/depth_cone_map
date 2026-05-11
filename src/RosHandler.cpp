#include "depth_cone_map/RosHandler.hpp"
#include "depth_cone_map/DepthConeMapNode.hpp"
#include <functional>
#include <sensor_msgs/msg/detail/image__struct.hpp>

RosHandler::RosHandler(rclcpp::Node* node_ptr, DepthConeMapNode& depth_cone_map_node) : qos(10), node_ptr(node_ptr) {
    int queue_size = 10; //FIXME: trova un valore appropriato

    bb_sub.subscribe(node_ptr, "/bounding_boxes");
    depth_sub.subscribe(node_ptr, "/depth_images");
    image_left_sub.subscribe(node_ptr, "/camera_left_image");
    pose_sub.subscribe(node_ptr, "/camera_pose");


    //Questa sintassi un po' maledetta istanzia lo shared_ptr con l'approximate time synchronizer.
    //I template type sono il Synchronizer, la Policy(ApproximateTime) e i 4 messaggi da sincronizzare
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<driverless_msgs::msg::BoundingBoxes,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, driverless_msgs::msg::PoseStamped>>>(message_filters::sync_policies::ApproximateTime<driverless_msgs::msg::BoundingBoxes,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, driverless_msgs::msg::PoseStamped>(queue_size), bb_sub, depth_sub, image_left_sub, pose_sub
    );


    sync->registerCallback(std::bind(&DepthConeMapNode::callback, &depth_cone_map_node, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    cone_pub = node_ptr->create_publisher<driverless_msgs::msg::MarkerArrayStamped>("/cone_map", qos);

    camera_info_sub = node_ptr->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", qos, std::bind(&DepthConeMapNode::cameraInfoCallback, &depth_cone_map_node, std::placeholders::_1));
}

void RosHandler::publish_cones(driverless_msgs::msg::MarkerArrayStamped msg) const{
    msg.header.stamp = node_ptr->now();
    cone_pub->publish(msg);
}
