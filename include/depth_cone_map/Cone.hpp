#ifndef CONE_STRUCT
#define CONE_STRUCT

#include <opencv2/core/types.hpp>
struct Cone{
    cv::Point3f position_world_frame;
    std::string color;
    int id;
};

#endif
