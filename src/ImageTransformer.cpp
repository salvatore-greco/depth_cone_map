#include "depth_cone_map/ImageTransformer.hpp"

driverless_msgs::msg::MarkerArrayStamped ImageTransformer::cameraToWorld(const std::vector<cv::Point3f> &cones) {

    std::vector<visualization_msgs::msg::Marker> markers;
    geometry_msgs::msg::TransformStamped transformation;
    try {
        transformation = tf2_buffer->lookupTransform(
            map_frame_name, camera_frame_name, tf2::TimePointZero);
    }
    catch (const tf2::LookupException& e) {
        RCLCPP_ERROR(logger, "Error when looking up transform %s", e.what());
    }
    for (const auto &cone: cones) {
        //marker non ha le funzioni per la trasformazione
        geometry_msgs::msg::Point point_to_transform = cvPoint3fToGeometryMsgsPoint(cone);
        geometry_msgs::msg::Point point_trasformed;
        //tf2_buffer->transform(point_to_transform, point_trasformed, "map");
        //se questa non funziona da tf2_geometry_msgs:
        tf2::doTransform<geometry_msgs::msg::Point>(point_to_transform, point_trasformed, transformation);
        //std::cout<<"["<<point_trasformed.x<<","<<point_trasformed.y<<","<<point_trasformed.z<<"]"<<std::endl;
        visualization_msgs::msg::Marker marker;
        //copiando come viene fatto da clustering_node
        marker.type = marker.CYLINDER;
        marker.header.frame_id = map_frame_name;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.7;
        marker.pose.position.x = point_trasformed.x;
        marker.pose.position.y = point_trasformed.y;
        marker.pose.position.z = point_trasformed.z;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.z = 0.0;
        markers.push_back(std::move(marker));
    }
    driverless_msgs::msg::MarkerArrayStamped marker_array_stamped;
    marker_array_stamped.markers = markers;
    return marker_array_stamped;
}

geometry_msgs::msg::Point ImageTransformer::cvPoint3fToGeometryMsgsPoint(cv::Point3f point) {
    //geometry_msgs/Point non ha il costruttore, assurdo lo so
    geometry_msgs::msg::Point geometry_msgs_point;
    geometry_msgs_point.x = point.x;
    geometry_msgs_point.y = point.y;
    geometry_msgs_point.z = point.z;
    return geometry_msgs_point;
}
