#ifndef SUPERGLUEFEATUREMATCHER_HPP
#define SUPERGLUEFEATUREMATCHER_HPP

#include "AbstractFeatureMatcher.hpp"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <opencv2/core/types.hpp>
#include <vector>
#include "super_glue.h"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

class SuperGlueFeatureMatcher : public AbstractFeatureMatcher<Eigen::Matrix<double, 259, Eigen::Dynamic>, cv::DMatch>{
    public:
    SuperGlueFeatureMatcher(const std::string& config_path, const std::string& weight_path, const rclcpp::Logger& logger);

    std::vector<cv::DMatch> matchFeature(Eigen::Matrix<double, 259, Eigen::Dynamic>& keyframe_features, Eigen::Matrix<double, 259, Eigen::Dynamic>& current_frame_features) override;
    
    private:
    Configs superglue_configs;

    std::unique_ptr<SuperGlue> superglue;

    rclcpp::Logger logger;
};
#endif // SUPERGLUEFEATUREMATCHER_HPP
