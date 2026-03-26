#ifndef SUBSCRIBER_DEFINE
#define SUBSCRIBER_DEFINE

#include "rclcpp/rclcpp.hpp"
#include "driverless_msgs/msg/bounding_boxes.hpp"
#include "driverless_msgs/msg/marker_array_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include <memory>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/qos.hpp>
#include <functional>


class DepthConeMapNode;

class RosHandler{
    public:
        RosHandler(rclcpp::Node::SharedPtr node_ptr, DepthConeMapNode& depth_cone_map_node);

    private:
        message_filters::Subscriber<driverless_msgs::msg::BoundingBoxes> bb_sub;

        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub;

        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub;

        rclcpp::Publisher<driverless_msgs::msg::MarkerArrayStamped>::SharedPtr cone_pub;

        rclcpp::QoS qos; //TODO: fai matchare col qos degli altri 3 messaggi che devi sincronizzare
        
        std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
            driverless_msgs::msg::BoundingBoxes,
            sensor_msgs::msg::Image,
            geometry_msgs::msg::PoseStamped>>> sync;

        rclcpp::Node::SharedPtr node_ptr;
};


#endif