#ifndef BUILD_IMAGEPROCESSOR_H
#define BUILD_IMAGEPROCESSOR_H

#include "MessageContainer.hpp"
#include <cv_bridge/cv_bridge.h>
#include "simdjson.h"
#include <list>
#include <tf2_ros/buffer.h>
ad#include <tf2_ros/transform_listener.h>


#include "driverless_msgs/msg/marker_array_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImageProcessor {

public:
    explicit ImageProcessor(const rclcpp::Clock::SharedPtr& clock): tf2_buffer(std::make_unique<tf2_ros::Buffer>(clock)), tf2_listener(std::make_unique<tf2_ros::TransformListener>(*tf2_buffer)), ros_clock(clock), k_matrix(3,3,CV_64FC1) {};

    std::list<std::pair<cv::Point, cv::Point>> getBBInJSON(const MessageContainer &messages);

    std::vector<cv::Point3f> coneFinder(const MessageContainer &messages,
                                        const std::list<std::pair<cv::Point, cv::Point> > &bb_points);

    driverless_msgs::msg::MarkerArrayStamped cameraToWorld(const std::vector<cv::Point3f> &cones);

    inline void saveKMatrixAsCvMat(const std::array<double,9>& k_array){
        std::memcpy(k_matrix.data, k_array.data(), k_array.size()*sizeof(double));
    }

    cv::Mat backProjection(cv::Vec3d pixel, float depth);


private:
    const float PERCENTILE = 0.2;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::shared_ptr<rclcpp::Clock> ros_clock;

    geometry_msgs::msg::Point cvPoint3fToGeometryMsgsPoint(cv::Point3f point);

    cv::Mat k_matrix;
};

#endif //BUILD_IMAGEPROCESSOR_H
