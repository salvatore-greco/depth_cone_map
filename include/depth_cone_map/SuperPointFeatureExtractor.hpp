#ifndef SUPERPOINTFEATUREEXTRACTOR_HPP
#define SUPERPOINTFEATUREEXTRACTOR_HPP

#include "AbstractFeatureExtractor.hpp"
#include <super_point.h>
#include <memory>
#include <string>
#include <Eigen/Core>
#include <rclcpp/logger.hpp>
#include <utility>
#include "Exceptions.hpp"

class MessageContainer;

class SuperPointFeatureExtractor : public AbstractFeatureExtractor<Eigen::Matrix<double, 259, Eigen::Dynamic>>{
    public:
    SuperPointFeatureExtractor(const std::string& config_path, const std::string& weight_path, const rclcpp::Logger& logger, std::shared_ptr<MessageContainer> message_container);

    Eigen::Matrix<double, 259, Eigen::Dynamic> getFeatureInBB(cv::Mat image, const std::list<std::pair<cv::Point, cv::Point>>& bb) override;

    void setScaleFactorX(float scale_factor_x){
        this->scale_factor_x = scale_factor_x;
    }

    void setScaleFactorY(float scale_factor_y){
        this->scale_factor_y = scale_factor_y;
    }

    protected:
    Eigen::Matrix<double, 259, Eigen::Dynamic> extractFeature(cv::Mat image) override;

    private:
    Configs superpoint_config;

    std::unique_ptr<SuperPoint> superpoint;

    rclcpp::Logger logger;

    std::shared_ptr<MessageContainer> message_container;

    bool isInsideBoundingBox(const double x_feature, const double y_feature, const std::pair<cv::Point, cv::Point>& bb) const;

    float scale_factor_x = 1;
    float scale_factor_y = 1;

};

#endif // SUPERPOINTFEATUREEXTRACTOR_HPP
