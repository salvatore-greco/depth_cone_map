#ifndef IMAGETRANSFORMER_HPP
#define IMAGETRANSFORMER_HPP

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "driverless_msgs/msg/marker_array_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/core/types.hpp>


class ImageTransformer{
public:
    explicit ImageTransformer(const rclcpp::Clock::SharedPtr& clock, const std::string& map_frame_name, const std::string& camera_frame_name, const rclcpp::Logger& logger): 
        clock(clock),
        map_frame_name(map_frame_name),
        camera_frame_name(camera_frame_name),
        tf2_buffer(std::make_unique<tf2_ros::Buffer>(clock)),
        tf2_listener(std::make_unique<tf2_ros::TransformListener>(*tf2_buffer)),
        logger(logger)
        {};

        driverless_msgs::msg::MarkerArrayStamped cameraToWorld(const std::vector<cv::Point3f> &cones);
        


private:
    geometry_msgs::msg::Point cvPoint3fToGeometryMsgsPoint(cv::Point3f point);


    rclcpp::Clock::SharedPtr clock;
    std::string map_frame_name;
    std::string camera_frame_name;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    rclcpp::Logger logger;
};

#endif // IMAGETRANSFORMER_HPP
