#include "depth_cone_map/RosHandler.hpp"
#include "depth_cone_map/DepthConeMapNode.hpp"

RosHandler::RosHandler(rclcpp::Node* node_ptr, DepthConeMapNode& depth_cone_map_node) : qos(10) {
    int queue_size = 10; //FIXME: trova un valore appropriato

    //TODO: guarda il qos degli altri nodi e matchalo sennò non funziona una mazza
    bb_sub.subscribe(node_ptr, "/cone_detection/output");
    depth_sub.subscribe(node_ptr, "/zed2i/zed_node/depth/depth_registered");
    pose_sub.subscribe(node_ptr, "/orb_slam3/camera_pose");

    //Questa sintassi un po' maledetta istanzia lo shared_ptr con l'approximate time synchronizer.
    //I template type sono il Synchronizer, la Policy(ApproximateTime) e i 3 messaggi da sincronizzare
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<driverless_msgs::msg::BoundingBoxes, 
        sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>>>(message_filters::sync_policies::ApproximateTime<driverless_msgs::msg::BoundingBoxes, 
        sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped>(queue_size), bb_sub, depth_sub, pose_sub
    );
    
    //TODO: controlla se i 3 messaggi hanno ritardi sistematici e in caso chiama setAgePenalty();
    
    sync->registerCallback(std::bind(&DepthConeMapNode::callback, &depth_cone_map_node, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    cone_pub = node_ptr->create_publisher<driverless_msgs::msg::MarkerArrayStamped>("/slam/cone_map", qos);
}

void RosHandler::publish_cones(driverless_msgs::msg::MarkerArrayStamped msg) const{
    cone_pub->publish(msg);
}
