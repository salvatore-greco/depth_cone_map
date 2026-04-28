#ifndef FEATUREEXTRACTOR_HPP
#define FEATUREEXTRACTOR_HPP

#include <opencv2/core/mat.hpp>
#include <list>

template <typename T>
class AbstractFeatureExtractor{
    public:
    virtual T getFeatureInBB(cv::Mat image, const std::list<std::pair<cv::Point, cv::Point>>& bb) = 0;
    protected:
    virtual T extractFeature(cv::Mat image) = 0;
};

#endif // FEATUREEXTRACTOR_HPP
