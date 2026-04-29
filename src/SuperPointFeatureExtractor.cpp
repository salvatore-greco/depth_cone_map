#include "depth_cone_map/SuperPointFeatureExtractor.hpp"
#include "depth_cone_map/Exceptions.hpp"
#include <Eigen/src/Core/util/Constants.h>
#include <rclcpp/logging.hpp>

SuperPointFeatureExtractor::SuperPointFeatureExtractor(const std::string& config_path, const std::string& weight_path, const rclcpp::Logger& logger): 
        superpoint_config(config_path, weight_path), 
        superpoint(std::make_unique<SuperPoint>(superpoint_config.superpoint_config)),
        logger(logger)
        {
            RCLCPP_INFO(logger, "Building engine for superpoint, may take a while if first time");
            superpoint->build();
        };

Eigen::Matrix<double, 259, Eigen::Dynamic> SuperPointFeatureExtractor::getFeatureInBB(cv::Mat image, const std::list<std::pair<cv::Point, cv::Point>>& bb) {
    Eigen::Matrix<double, 259, Eigen::Dynamic> features;
    try{
        features = extractFeature(image); //FIXME: spero venga fatta RVO. Runna un bel profiler.
    }
    catch(const NoFeatureFoundException& e){
        RCLCPP_ERROR(logger, "Extract feature failed: %s", e.what());
    }
    std::vector<int> indexes;
    //TODO: controlla se è troppo lento
    for(const auto& bounding_box : bb){
        for(int i=0; i<features.cols(); i++){
            if(isInside(features(1,i), features(2,i), bounding_box))
                indexes.push_back(i);
        }
    }
    Eigen::Matrix<double, 259, Eigen::Dynamic> feature_in_bb(259, indexes.size());
    for (size_t i = 0; i<indexes.size(); i++)
        feature_in_bb.col(i) = features.col(indexes[i]);
    RCLCPP_INFO(logger, "Found %zu feature inside bounding boxes", feature_in_bb.size());
    return feature_in_bb;
}

Eigen::Matrix<double, 259, Eigen::Dynamic> SuperPointFeatureExtractor::extractFeature(cv::Mat image) {
    Eigen::Matrix<double, 259, Eigen::Dynamic> features;
    if(!superpoint->infer(image, features))
        throw NoFeatureFoundException("SuperPoint infer failed: no feature returned");
    return features;
}

bool SuperPointFeatureExtractor::isInside(const double x, const double y, const std::pair<cv::Point, cv::Point>& bb) const{
    return (x>bb.first.x) && (x<bb.second.x) && (y>bb.first.y) && (y<bb.second.y);
}


