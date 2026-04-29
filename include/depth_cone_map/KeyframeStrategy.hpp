#ifndef KEYFRAMESTRATEGY_HPP
#define KEYFRAMESTRATEGY_HPP

#include <opencv2/core/mat.hpp>

class AbstractKeyframeStrategy{
public:
    virtual bool isKeyframeInvalid() = 0;
};

#endif // KEYFRAMESTRATEGY_HPP
