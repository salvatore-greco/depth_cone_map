#ifndef SUBSCRIBER_DEFINE
#define SUBSCRIBER_DEFINE

#include "rclcpp/rclcpp.hpp"
#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "driverless_msgs/msg/marker_array_stamped.hpp"
#include "driverless_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "driverless_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/qos.hpp>
#include <functional>
#include <sensor_msgs/msg/detail/image__struct.hpp>



class DepthConeMapNode;

class RosHandler{
    public:
        RosHandler(rclcpp::Node* node_ptr, DepthConeMapNode& depth_cone_map_node);
        void publish_cones(driverless_msgs::msg::MarkerArrayStamped msg) const;
        inline void camera_info_unsubscribe() {
            camera_info_sub.reset();
        }
    private:
        message_filters::Subscriber<driverless_msgs::msg::BoundingBoxes> bb_sub;

        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub;

        message_filters::Subscriber<sensor_msgs::msg::Image> image_left_sub;

        message_filters::Subscriber<driverless_msgs::msg::PoseStamped> pose_sub;

        rclcpp::Publisher<driverless_msgs::msg::MarkerArrayStamped>::SharedPtr cone_pub;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;

        rclcpp::QoS qos; //TODO: fai matchare col qos degli altri 3 messaggi che devi sincronizzare

        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
            driverless_msgs::msg::BoundingBoxes,
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image, driverless_msgs::msg::PoseStamped>>> sync;

        rclcpp::Node* node_ptr;

};


#endif
