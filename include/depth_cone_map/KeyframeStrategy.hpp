#ifndef KEYFRAMESTRATEGY_HPP
#define KEYFRAMESTRATEGY_HPP

#include "sensor_msgs/msg/image.hpp"

class AbstractKeyframeStrategy{
public:
    virtual void saveKeyframe(sensor_msgs::msg::Image image) = 0;
};

#endif // KEYFRAMESTRATEGY_HPP
