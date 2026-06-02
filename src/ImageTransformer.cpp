#include "depth_cone_map/ImageTransformer.hpp"
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include "depth_cone_map/Cone.hpp"
// #include <omp.h>
#include <execution>

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

    std::transform(std::execution::par_unseq, cones.begin(), cones.end(), cones_world_frame.begin(), [this, transformation](const Cone& cone){
        geometry_msgs::msg::Point point_to_transform = cvPoint3fToGeometryMsgsPoint(cone.position_world_frame);
        geometry_msgs::msg::Point point_trasformed;
        tf2::doTransform<geometry_msgs::msg::Point>(point_to_transform, point_trasformed, transformation);
        return Cone(
            cv::Point3f(point_trasformed.x, point_trasformed.y, point_trasformed.z),
            cone.color,
            -1
        );
    });

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
