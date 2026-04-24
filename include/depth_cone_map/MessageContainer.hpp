#ifndef BUILD_MESSAGECONTAINER_H
#define BUILD_MESSAGECONTAINER_H

#include <algorithm>
#include <memory>
#include <mutex>
#include <utility>

#include "depth_cone_map/KeyframeStrategy.hpp"
#include "driverless_msgs/msg/pose_stamped.hpp"
#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class MessageContainer {
public:
    MessageContainer(driverless_msgs::msg::BoundingBoxes::ConstSharedPtr bb,
                     sensor_msgs::msg::Image::ConstSharedPtr depth,
                     driverless_msgs::msg::PoseStamped::ConstSharedPtr pose)
        : bb(std::move(bb)),
          depth(std::move(depth)),
          pose(std::move(pose))
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

    void saveMessages(driverless_msgs::msg::BoundingBoxes::ConstSharedPtr bb, sensor_msgs::msg::Image::ConstSharedPtr depth, sensor_msgs::msg::Image::ConstSharedPtr current_image){
        {
            const std::lock_guard<std::mutex> lock(this->mutex);
            this->bb = std::move(bb);
            this->depth = std::move(depth);
            this->previous_image = this->current_image;
            this->current_image = std::move(current_image);
        }
        keyframe_strategy->saveKeyframe(*(this->current_image));
    }

    //si può rendere thread safe e farlo parte di una classe
    // metodi per salvare tutti i messaggi, getter dei messaggi tutto thread safe. (basta un mutex o un lock)
private:
    std::shared_ptr<const driverless_msgs::msg::BoundingBoxes> bb;
    std::shared_ptr<const sensor_msgs::msg::Image> depth;
    //sensor_msgs::msg::Image::ConstSharedPtr depth;
    driverless_msgs::msg::PoseStamped::ConstSharedPtr pose;

    std::shared_ptr<const sensor_msgs::msg::Image> current_image;
    std::shared_ptr<const sensor_msgs::msg::Image> previous_image;
    std::shared_ptr<const sensor_msgs::msg::Image> keyframe;

    std::unique_ptr<AbstractKeyframeStrategy> keyframe_strategy;

    std::mutex mutex;
    

};
#endif //BUILD_MESSAGECONTAINER_H
