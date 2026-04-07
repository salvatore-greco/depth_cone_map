#ifndef BUILD_MESSAGECONTAINER_H
#define BUILD_MESSAGECONTAINER_H

#include <utility>

#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MessageContainer {
public:
    MessageContainer(driverless_msgs::msg::BoundingBoxes::ConstSharedPtr bb,
                     sensor_msgs::msg::Image::ConstSharedPtr depth,
                     geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
        : bb(std::move(bb)),
          depth(std::move(depth)),
          pose(std::move(pose))
          {
    }

    std::shared_ptr<const sensor_msgs::msg::Image> getImage() const {
        return depth;
    }

    std::shared_ptr<const driverless_msgs::msg::BoundingBoxes> getBB() const {
        return bb;
    }

private
:
    std::shared_ptr<const driverless_msgs::msg::BoundingBoxes> bb;
    std::shared_ptr<const sensor_msgs::msg::Image> depth;
    //sensor_msgs::msg::Image::ConstSharedPtr depth;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
};
#endif //BUILD_MESSAGECONTAINER_H
