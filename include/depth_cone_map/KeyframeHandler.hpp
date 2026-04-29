#ifndef KEYFRAMEHANDLER_HPP
#define KEYFRAMEHANDLER_HPP

#include "KeyframeStrategy.hpp"
#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/core/mat.hpp>

class KeyframeHandler{

    public: 
    explicit KeyframeHandler(std::unique_ptr<AbstractKeyframeStrategy> keyframe_strategy) : keyframe_strategy(std::move(keyframe_strategy)){};

    void saveKeyframe(const sensor_msgs::msg::Image::ConstSharedPtr image){
        if (keyframe_strategy->isKeyframeInvalid()) {
            current_keyframe = cv_bridge::toCvShare(image, image->encoding)->image;
        }
    }

    void setCurrentFrameFeature(Eigen::Matrix<double, 259, Eigen::Dynamic> feature){
        current_keyframe_feature = feature;
    }

    private:
    std::unique_ptr<AbstractKeyframeStrategy> keyframe_strategy;
    Eigen::Matrix<double, 259, Eigen::Dynamic> current_keyframe_feature;
    cv::Mat current_keyframe;
};

#endif // KEYFRAMEHANDLER_HPP
