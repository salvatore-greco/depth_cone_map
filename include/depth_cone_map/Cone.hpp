#ifndef CONE_STRUCT
#define CONE_STRUCT

#include <opencv2/core/types.hpp>
#include <vector>

enum class ConeColor{
    UNKNOWN = -1,
    BLUE,
    YELLOW,
    ORANGE,
    LARGE_ORANGE
};

struct Cone{
    Cone(cv::Point3f pos, ConeColor color, int id) : position_world_frame(pos), color(color), id(id){};
    cv::Point3f position_world_frame;
    ConeColor color;
    int id;
};

#endif
