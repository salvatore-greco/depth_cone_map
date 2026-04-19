#ifndef BUILD_IMAGEPROCESSOR_H
#define BUILD_IMAGEPROCESSOR_H

#include "MessageContainer.hpp"
#include <cv_bridge/cv_bridge.h>
#include "simdjson.h"
#include <list>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>

#include <rclcpp/logging.hpp>
#include <utility>
#include <cmath>

class ImageProcessor {

public:
    ImageProcessor(const rclcpp::Logger& logger): 
        k_matrix(3,3,CV_64FC1),
        logger(logger)
    {};

    std::list<std::pair<cv::Point, cv::Point>> getBBInJSON(const MessageContainer &messages);

    std::vector<cv::Point3f> coneFinder(const MessageContainer &messages,
                                        const std::list<std::pair<cv::Point, cv::Point> > &bb_points);

    inline void saveKMatrixAsCvMat(const std::array<double,9>& k_array){
        std::memcpy(k_matrix.data, k_array.data(), k_array.size()*sizeof(double));
    }

    cv::Mat backProjection(cv::Vec3d pixel, float depth);


private:
    const float PERCENTILE = 0.2;

    cv::Mat k_matrix;

    rclcpp::Logger logger;

    bool isDepthValueInvalid(const float& value) const;
};

#endif //BUILD_IMAGEPROCESSOR_H
