#ifndef BUILD_IMAGEPROCESSOR_H
#define BUILD_IMAGEPROCESSOR_H

#include "MessageContainer.hpp"
#include <cv_bridge/cv_bridge.h>
#include "simdjson.h"
#include <list>
#include <tf2_ros/buffer.h>

#include "driverless_msgs/msg/marker_array_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"

class ImageProcessor {

public:
    explicit ImageProcessor(const rclcpp::Clock::SharedPtr& clock): tf2_buffer(clock) {};

    std::list<std::pair<cv::Point, cv::Point>> getBBInJSON(const MessageContainer &messages);

    std::vector<cv::Point3f> coneFinder(const MessageContainer &messages,
                                        const std::list<std::pair<cv::Point, cv::Point> > &bb_points);

    driverless_msgs::msg::MarkerArrayStamped cameraToWorld(const std::vector<cv::Point3f> &cones);

private:
    const float PERCENTILE = 0.2;
    tf2_ros::Buffer tf2_buffer;
    static geometry_msgs::msg::Point cvPoint3fToGeometryMsgsPoint(cv::Point3f point);
};

#endif //BUILD_IMAGEPROCESSOR_H
