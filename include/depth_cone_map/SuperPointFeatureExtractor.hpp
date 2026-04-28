#ifndef SUPERPOINTFEATUREEXTRACTOR_HPP
#define SUPERPOINTFEATUREEXTRACTOR_HPP

#include "AbstractFeatureExtractor.hpp"
#include <super_point.h>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <rclcpp/logger.hpp>

class SuperPointFeatureExtractor : public AbstractFeatureExtractor<Eigen::Matrix<double, 259, Eigen::Dynamic>>{
    public:
    SuperPointFeatureExtractor(const std::string& config_path, const std::string& weight_path, rclcpp::Logger logger);
    
    Eigen::Matrix<double, 259, Eigen::Dynamic> getFeatureInBB(cv::Mat image, const std::list<std::pair<cv::Point, cv::Point>>& bb) override;

    protected:
    Eigen::Matrix<double, 259, Eigen::Dynamic> extractFeature(cv::Mat image) override;
    
    private:
    Configs superpoint_config;
    std::unique_ptr<SuperPoint> superpoint;

    rclcpp::Logger logger;
};

#endif // SUPERPOINTFEATUREEXTRACTOR_HPP
