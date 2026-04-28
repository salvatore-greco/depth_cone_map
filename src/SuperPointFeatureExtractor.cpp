#include "depth_cone_map/SuperPointFeatureExtractor.hpp"
#include <rclcpp/logging.hpp>

SuperPointFeatureExtractor::SuperPointFeatureExtractor(const std::string& config_path, const std::string& weight_path, rclcpp::Logger logger): 
        superpoint_config(config_path, weight_path), 
        superpoint(std::make_unique<SuperPoint>(superpoint_config.superpoint_config)),
        logger(logger)
        {
            RCLCPP_INFO(logger, "Building engine for superpoint, may take a while if first time");
            superpoint->build();
        };

Eigen::Matrix<double, 259, Eigen::Dynamic> SuperPointFeatureExtractor::getFeatureInBB(cv::Mat image, const std::list<std::pair<cv::Point, cv::Point>>& bb) {
    auto feature = extractFeature(image);
    //do something
    return feature;
}

Eigen::Matrix<double, 259, Eigen::Dynamic> SuperPointFeatureExtractor::extractFeature(cv::Mat image) {
    Eigen::Matrix<double, 259, Eigen::Dynamic> features;
    superpoint->infer(image, features);
    return features;
}


