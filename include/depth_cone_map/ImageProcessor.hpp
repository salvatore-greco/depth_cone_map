#ifndef BUILD_IMAGEPROCESSOR_H
#define BUILD_IMAGEPROCESSOR_H

#include "MessageContainer.hpp"
#include <cv_bridge/cv_bridge.h>
#include "simdjson.h"
#include <list>

class ImageProcessor {

public:
    ImageProcessor() = default;

    std::list<std::pair<cv::Point, cv::Point>> getPixelInBB(const MessageContainer &messages);

    std::vector<cv::Point3f> coneFinder(const MessageContainer &messages,
                                        const std::list<std::pair<cv::Point, cv::Point> > &bb_points);

private:
    std::list<std::pair<cv::Point, cv::Point>> parseBoundingBoxesJson(simdjson::padded_string json_bb);
    const float PERCENTILE = 0.2;
};

#endif //BUILD_IMAGEPROCESSOR_H
