#ifndef CONE_STRUCT
#define CONE_STRUCT

#include <opencv2/core/types.hpp>
struct Cone{
    Cone(cv::Point3f pos, std::string& color, int id) : position_world_frame(pos), color(color), id(id){};
    cv::Point3f position_world_frame;
    std::string color;
    int id;
};

#endif
