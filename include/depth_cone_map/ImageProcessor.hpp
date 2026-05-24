#ifndef BUILD_IMAGEPROCESSOR_H
#define BUILD_IMAGEPROCESSOR_H

#include "MessageContainer.hpp"
#include <cv_bridge/cv_bridge.h>
#include "simdjson.h"
#include <list>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>

#include <rclcpp/logging.hpp>
#include <string_view>
#include <utility>
#include <cmath>
#include "Cone.hpp"

class ImageProcessor {

public:
    ImageProcessor(const rclcpp::Logger& logger, const float& percentile, std::shared_ptr<MessageContainer> message_container):
        PERCENTILE(percentile),
        k_matrix(3,3,CV_64FC1),
        logger(logger),
        message_container(message_container)
    {};

    std::list<std::pair<cv::Rect, ConeColor>> getBBInJSON();

    std::vector<Cone> getConeInCameraFrame(const std::list<std::pair<cv::Rect, ConeColor>> &bb_points);

    inline void saveKMatrixAsCvMat(const std::array<double,9>& k_array){
        std::memcpy(k_matrix.data, k_array.data(), k_array.size()*sizeof(double));
    }

    cv::Mat backProjection(cv::Vec3d pixel, float depth);

    //usare con semantica move
    ConeColor colorStringToEnum(std::string_view color_string){
        if(color_string == "blue_cone")
            return ConeColor::BLUE;
        else if(color_string == "yellow_cone")
            return ConeColor::YELLOW;
        else if(color_string == "orange_cone")
            return ConeColor::ORANGE;
        else if(color_string == "large_orange_cone")
            return ConeColor::LARGE_ORANGE;
        else return ConeColor::UNKNOWN;
    }


private:
    const float PERCENTILE;

    cv::Mat k_matrix;

    rclcpp::Logger logger;

    std::shared_ptr<MessageContainer> message_container;

    bool isDepthValueInvalid(const float& value) const;

    cv::Point extractBoundingBox(simdjson::ondemand::array& bounding_box);
};

#endif //BUILD_IMAGEPROCESSOR_H
