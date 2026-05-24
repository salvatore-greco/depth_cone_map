#include "depth_cone_map/ImageTransformer.hpp"
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include "depth_cone_map/Cone.hpp"
#include <omp.h>

std::vector<Cone> ImageTransformer::cameraToWorld(const std::vector<Cone> &cones) {

    geometry_msgs::msg::TransformStamped transformation;
    auto time = message_container->getPose()->header.stamp;
    try {
        transformation = tf2_buffer->lookupTransform(
            map_frame_name, camera_frame_name, time);
    }
    catch (const tf2::LookupException& e) {
        RCLCPP_ERROR(logger, "Error when looking up transform %s", e.what());
    }
    catch(const tf2::ExtrapolationException& e){
        RCLCPP_ERROR(logger, "Cannot transform back in time... returning an empty list.");
        RCLCPP_ERROR(logger, "exception what: %s", e.what());
        return std::vector<Cone>();
    }
    std::vector<Cone> cones_world_frame(cones.size());

    #pragma omp parallel for num_threads(4)
    // for (const auto &cone: cones) {
    for(size_t i = 0; i<cones.size(); i++){
        //marker non ha le funzioni per la trasformazione
        geometry_msgs::msg::Point point_to_transform = cvPoint3fToGeometryMsgsPoint(cones[i].position_world_frame);
        geometry_msgs::msg::Point point_trasformed;
        //tf2_buffer->transform(point_to_transform, point_trasformed, "map");
        //se questa non funziona da tf2_geometry_msgs:
        tf2::doTransform<geometry_msgs::msg::Point>(point_to_transform, point_trasformed, transformation);
        //std::cout<<"["<<point_trasformed.x<<","<<point_trasformed.y<<","<<point_trasformed.z<<"]"<<std::endl;

        cones_world_frame[i] = Cone(
            cv::Point3f(point_trasformed.x, point_trasformed.y, point_trasformed.z),
            cones[i].color,
            -1
        );
        // cones_world_frame.emplace_back(cv::Point3f(point_trasformed.x, point_trasformed.y, point_trasformed.z), cones[i].color, -1);
    }
    return cones_world_frame;
}

geometry_msgs::msg::Point ImageTransformer::cvPoint3fToGeometryMsgsPoint(cv::Point3f point) {
    //geometry_msgs/Point non ha il costruttore, assurdo lo so
    geometry_msgs::msg::Point geometry_msgs_point;
    geometry_msgs_point.x = point.x;
    geometry_msgs_point.y = point.y;
    geometry_msgs_point.z = point.z;
    return geometry_msgs_point;
}
