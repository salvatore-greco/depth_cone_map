#include "depth_cone_map/SuperGlueFeatureMatcher.hpp"


SuperGlueFeatureMatcher::SuperGlueFeatureMatcher(const std::string& config_path, const std::string& weight_path, const rclcpp::Logger& logger)
  : superglue_configs(config_path, weight_path),
    superglue(std::make_unique<SuperGlue>(superglue_configs.superglue_config)),
    logger(logger)
{
    RCLCPP_INFO(logger, "Building engine for superglue, may take a while if first time");
    superglue->build();
    RCLCPP_INFO(logger, "Superglue engine build finished");
}

std::vector<cv::DMatch> SuperGlueFeatureMatcher::matchFeature(Eigen::Matrix<double, 259, Eigen::Dynamic>& keyframe_features, Eigen::Matrix<double, 259, Eigen::Dynamic>& current_frame_features) {
    std::vector<cv::DMatch> matches;
    superglue->matching_points(keyframe_features, current_frame_features, matches);
    return matches;
}
