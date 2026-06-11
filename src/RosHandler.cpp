#include "depth_cone_map/RosHandler.hpp"
#include <functional>
// #include "depth_cone_map/DepthConeMapNode.hpp"
class DepthConeMapNode;
RosHandler::RosHandler(rclcpp::Node* node_ptr, const std::string& map_frame_name, callback callback, camera_info_callback camera_info_callback) : qos(10), node_ptr(node_ptr), map_frame_name(map_frame_name)  {
    int queue_size = 10; //FIXME: trova un valore appropriato

    bb_sub.subscribe(node_ptr, "/bounding_boxes");
    depth_sub.subscribe(node_ptr, "/depth_images");
    pose_sub.subscribe(node_ptr, "/camera_pose");


    //Questa sintassi un po' maledetta istanzia lo shared_ptr con l'approximate time synchronizer.
    //I template type sono il Synchronizer, la Policy(ApproximateTime) e i 4 messaggi da sincronizzare
    sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<driverless_msgs::msg::BoundingBoxes,
        sensor_msgs::msg::Image, driverless_msgs::msg::PoseStamped>>>(message_filters::sync_policies::ApproximateTime<driverless_msgs::msg::BoundingBoxes,
        sensor_msgs::msg::Image, driverless_msgs::msg::PoseStamped>(queue_size), bb_sub, depth_sub, pose_sub
    );


    sync->registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    cone_pub = node_ptr->create_publisher<driverless_msgs::msg::MarkerArrayStamped>("/cone_map", qos);

    // camera_info_sub = node_ptr->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", qos, std::bind(&DepthConeMapNode::cameraInfoCallback, &depth_cone_map_node, std::placeholders::_1));
    camera_info_sub = node_ptr->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info", qos, camera_info_callback);
}

void RosHandler::publish_cones(std::vector<Cone>& cones) const{
    std::vector<visualization_msgs::msg::Marker> markers;
    for (const auto& cone : cones){
        visualization_msgs::msg::Marker marker;
        //copiando come viene fatto da clustering_node
        // il colore del cono viene identificato da traj tramite l'id del marker.
        switch(cone.color){
            case ConeColor::BLUE:
                marker.id = cone.id;
                break;
            case ConeColor::YELLOW:
                marker.id = cone.id + 1000;
                break;
            case ConeColor::ORANGE:
                marker.id = cone.id + 2000;
                break;
            case ConeColor::LARGE_ORANGE:
                marker.id = cone.id + 3000;
                break;
            case ConeColor::UNKNOWN:
                break;
        }
        marker.type = marker.CYLINDER;
        marker.header.frame_id = this->map_frame_name;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.7;
        marker.pose.position.x = cone.position_world_frame.x;
        marker.pose.position.y = cone.position_world_frame.y;
        marker.pose.position.z = cone.position_world_frame.z;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.z = 0.0;
        // Questo attributo colore è solo per la visualizzazione in rviz.
        marker.color.r = 0.0;
        marker.color.b = 0.0;
        marker.color.g = 0.0;
        marker.color.a = 0.0;
        markers.push_back(std::move(marker)); //non posso usare emplace back perchè il messaggio non ha un costruttore con argomenti.
    }
    driverless_msgs::msg::MarkerArrayStamped marker_array_stamped;
    marker_array_stamped.markers = markers;
    marker_array_stamped.header.stamp = node_ptr->now();
    cone_pub->publish(marker_array_stamped);
}
