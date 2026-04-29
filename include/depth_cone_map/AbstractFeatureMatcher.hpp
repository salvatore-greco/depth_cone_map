#ifndef ABSTRACTFEATUREMATCHER_HPP
#define ABSTRACTFEATUREMATCHER_HPP

#include <vector>

template <typename FeatureType, typename FeatureMatch>
class AbstractFeatureMatcher{
public:
    virtual std::vector<FeatureMatch> matchFeature(FeatureType& keyframe_features, FeatureType& current_frame_features) = 0;
};

#endif // ABSTRACTFEATUREMATCHER_HPP
