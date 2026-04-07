#ifndef BUILD_IMAGEPROCESSOR_H
#define BUILD_IMAGEPROCESSOR_H

#include "MessageContainer.hpp"
#include <cv_bridge/cv_bridge.h>
#include "simdjson.h"
#include <list>

class ImageProcessor {

public:
    cv::Mat getPixelInBB(const MessageContainer &messages);
    void coneFinder();

private:
    std::list<std::pair<cv::Point, cv::Point>> parseBoundingBoxesJson(simdjson::padded_string json_bb);
};

#endif //BUILD_IMAGEPROCESSOR_H
