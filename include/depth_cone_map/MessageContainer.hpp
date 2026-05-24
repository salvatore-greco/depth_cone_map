#ifndef BUILD_MESSAGECONTAINER_H
#define BUILD_MESSAGECONTAINER_H

#include <algorithm>
#include <memory>
#include <mutex>
#include <utility>

#include "depth_cone_map/KeyframeHandler.hpp"
#include "depth_cone_map/KeyframeStrategy.hpp"
#include "driverless_msgs/msg/pose_stamped.hpp"
#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>

//TODO: leva image left!!!
class MessageContainer {
public:
    MessageContainer(driverless_msgs::msg::BoundingBoxes::ConstSharedPtr bb,
                     sensor_msgs::msg::Image::ConstSharedPtr depth)
        : bb(std::move(bb)),
          depth(std::move(depth))
          {};

    MessageContainer() = default;

    MessageContainer(MessageContainer& m) = delete; //non voglio che venga copiato, troppo oneroso

    std::shared_ptr<const sensor_msgs::msg::Image> getDepthImage() {
        const std::lock_guard<std::mutex> lock(this->mutex);
        return depth;
    }

    std::shared_ptr<const driverless_msgs::msg::BoundingBoxes> getBB() {
        const std::lock_guard<std::mutex> lock(this->mutex);
        return bb;
    }

    std::shared_ptr<const driverless_msgs::msg::PoseStamped> getPose() {
        const std::lock_guard<std::mutex> lock(this->mutex);
        return pose;
    }

    void saveMessages(
        driverless_msgs::msg::BoundingBoxes::ConstSharedPtr bb,
        sensor_msgs::msg::Image::ConstSharedPtr depth,
        driverless_msgs::msg::PoseStamped::ConstSharedPtr pose
    ){
        const std::lock_guard<std::mutex> lock(this->mutex);
        this->bb = bb;
        this->depth = depth;
        this->pose = pose;
    }

private:
    std::shared_ptr<const driverless_msgs::msg::BoundingBoxes> bb;
    std::shared_ptr<const sensor_msgs::msg::Image> depth;
    std::shared_ptr<const driverless_msgs::msg::PoseStamped> pose;
    //sensor_msgs::msg::Image::ConstSharedPtr depth;

    std::unique_ptr<KeyframeHandler> keyframe_handler;

    std::mutex mutex;


};
#endif //BUILD_MESSAGECONTAINER_H
