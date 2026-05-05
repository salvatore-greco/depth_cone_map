#ifndef KEYFRAMEHANDLER_HPP
#define KEYFRAMEHANDLER_HPP

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include "KeyframeStrategy.hpp"

class KeyframeHandler{

    public:
    explicit KeyframeHandler(std::unique_ptr<AbstractKeyframeStrategy> keyframe_strategy) : keyframe_strategy(std::move(keyframe_strategy)){};

    // diverse strategie possono avere diverse cose da salvare per vedere se il keyframe è valido. È difficile generalizzarle in un unica interfaccia
    // temporal ha bisogno del timestamp
    // match con score basso ha bisogno dei match
    bool saveKeyframe(const sensor_msgs::msg::Image::ConstSharedPtr image, Eigen::Matrix<double, 259, Eigen::Dynamic> current_keyframe_feature){
        if (keyframe_strategy->isKeyframeInvalid()) {
            current_keyframe = cv_bridge::toCvShare(image, image->encoding)->image;
            this->current_keyframe_feature = current_keyframe_feature;
            return true;
        }
        return false;
    }

    Eigen::Matrix<double, 259, Eigen::Dynamic> getCurrentFrameFeature(){
        return current_keyframe_feature;
    }

    cv::Mat getCurrentKeyframe(){
        return current_keyframe;
    }

    private:
    std::unique_ptr<AbstractKeyframeStrategy> keyframe_strategy;
    Eigen::Matrix<double, 259, Eigen::Dynamic> current_keyframe_feature;
    cv::Mat current_keyframe;
};

#endif // KEYFRAMEHANDLER_HPP
