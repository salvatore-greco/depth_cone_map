#include "depth_cone_map/DepthConeMapNode.hpp"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>

DepthConeMapNode::DepthConeMapNode(const rclcpp::NodeOptions& options) :
    Node("depth_cone_map", options),
    data_associator(std::make_unique<DataAssociator>()),
    gtsam_wrapper(std::make_unique<GtsamWrapper>()),
    cone_idx(0)
    {
        parameterDeclaration();
        parameterInitialization();
        message_container = std::make_shared<MessageContainer>();
        ros_handler = std::make_unique<RosHandler>(
            this,
            this->map_frame_name,
            std::bind(&DepthConeMapNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
            std::bind(&DepthConeMapNode::cameraInfoCallback, this, std::placeholders::_1)
        );
        image_processor = std::make_unique<ImageProcessor>(this->get_logger(), this->percentile, message_container);
        image_transformer = std::make_unique<ImageTransformer>(this->get_clock(), this->map_frame_name,
                                                            this->camera_frame_name, this->get_logger(), message_container);
        keyframe_handler = std::make_shared<KeyframeHandler>(std::make_unique<TemporalKeyframeStrategy>(std::chrono::seconds(1))); //è shared ptr in caso volessi mettere superpoint-glue in un thread


        cones.reserve(500);
        std::cout<<cone_dist_threshold<<std::endl;

}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const driverless_msgs::msg::PoseStamped::ConstSharedPtr& pose) {

    gtsam::Pose3 gtsam_pose(
        gtsam::Rot3::Quaternion(
            pose->pose.orientation.w,
            pose->pose.orientation.x,
            pose->pose.orientation.y,
            pose->pose.orientation.z
        ),
        gtsam::Point3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z)
    );
    gtsam_wrapper->savePose(std::move(gtsam_pose));

    // RCLCPP_INFO(this->get_logger(), "Received messages");
    this->message_container->saveMessages(bounding_boxes, depth_image, pose);
    const auto bounding_boxes_list = image_processor->getBBInJSON();
    const auto cones = image_processor->getConeInCameraFrame(bounding_boxes_list);
    auto cones_world_frame = image_transformer->cameraToWorld(cones);

    if(cones_world_frame.empty()){
        RCLCPP_ERROR(this->get_logger(), "No cone found, skipping");
        return;
    }

    if (this->cones.empty()){
        Cone first_cone = cones_world_frame[0];
        first_cone.id = cone_idx++;
        this->cones.push_back(first_cone);
        gtsam_wrapper->saveValue(&first_cone, cone_idx-1);
    }

    data_associator->updateIndex(this->cones);

    for (const auto& cone : cones_world_frame){
        if(cone.color == ConeColor::ORANGE) continue; //FIXME: ricordati di levarlo!
        Eigen::Map<const Eigen::Vector3f> actual_cone(&cone.position_world_frame.x);
        Eigen::Map<const Eigen::Vector3d> eigen_pose(&(pose->pose.position.x));
        if((actual_cone-eigen_pose.cast<float>()).norm() > car_cone_dist_threshold) continue; // il cono è troppo lontano, non ha senso metterlo

        int matched_id;
        const auto I = data_associator->searchNearestCone(cone);

        Eigen::Map<const Eigen::Vector3f> nearest_neighbor(&this->cones[I].position_world_frame.x);


        // non ho gia visto il cono in esame e non è troppo lontano dalla macchina
        if ((I == -1) || (actual_cone-nearest_neighbor).norm()>this->cone_dist_threshold){
            this->cones.emplace_back(cone.position_world_frame, cone.color, cone_idx++);
            //va aggiunto a isam come values
            gtsam_wrapper->saveValue(&cone, cone_idx-1);
            matched_id = cone_idx-1;
        }
        else{
            matched_id = I;
            // matched_id = this->cones[I].id;
        }

        gtsam_wrapper->saveBearingRangeFactor(&cone, matched_id);
    }
    if (debug) {
        // printDebug(bounding_boxes_list, cones, cones_world_frame);
    }

    if(keyframe_handler->keyframe_strategy->isKeyframeInvalid()){
        auto start = std::chrono::high_resolution_clock::now();
        auto estimate = gtsam_wrapper->updateAndCalculate();
        auto landmarks = estimate.filter(gtsam::Symbol::ChrTest('l'));
        for (const auto & val : landmarks){
            gtsam::Point3 p = estimate.at<gtsam::Point3>(val.key);
            // std::cout << "Key:" << gtsam::Symbol(val.key).index()<< " value:" << p << std::endl;
            auto& c = this->cones[gtsam::symbolIndex(val.key)];
            c.position_world_frame.x = p.x();
            c.position_world_frame.y = p.y();
            c.position_world_frame.z = p.z();
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
        std::cout << "gtsam duration: " << duration.count() << std::endl;
    }
    ros_handler->publish_cones(this->cones);
    // quando faccio isam update pulisco il grafo che stavo mantenendo qua (viene aggiunto tramite isam update)
    // RCLCPP_INFO(this->get_logger(), "%d", cone_idx);
}

void DepthConeMapNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    image_processor->saveKMatrixAsCvMat(camera_info->k);
    ros_handler->camera_info_unsubscribe();
}

void DepthConeMapNode::parameterDeclaration() {
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("camera_frame", "zed_left_camera_optical_frame");
    this->declare_parameter("percentile", 0.2);
    this->declare_parameter("debug", false);
    this->declare_parameter("cone_dist_threshold", 3.0);
    this->declare_parameter("car_cone_dist_threshold", 20.0);
}

void DepthConeMapNode::parameterInitialization() {
    map_frame_name = this->get_parameter("world_frame").as_string();
    camera_frame_name = this->get_parameter("camera_frame").as_string();
    percentile = this->get_parameter("percentile").as_double();
    debug = this->get_parameter("debug").as_bool();
    cone_dist_threshold = this->get_parameter("cone_dist_threshold").as_double();
    car_cone_dist_threshold = this->get_parameter("car_cone_dist_threshold").as_double();
}

void DepthConeMapNode::printDebug(const std::list<std::pair<cv::Point, cv::Point>>& bounding_boxes_list,
                                  const std::vector<cv::Point3f>& cones,
                                  const std::vector<Cone>& cones_world_frame) {
    std::ostringstream ss;
    ss << "JSON parsed: [\n";
    for (const auto& point : bounding_boxes_list) {
        ss << point.first << ", " << point.second << "\n";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    ss.str("");
    ss.clear();

    ss << "Cones in camera frame: [\n";
    for (const auto& cone_camera : cones) {
        ss << cone_camera << "\n";
    }
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    ss.str("");
    ss.clear();

    ss << "Cones in world frame, size " << cones_world_frame.size() << ": [\n";
    for (const auto& cone_world : cones_world_frame) {
        ss << "[" << cone_world.position_world_frame.x << "," << cone_world.position_world_frame.y << ", "
           << cone_world.position_world_frame.z << "],";
    }
    ss.seekp(-1, std::ios_base::end);  // mi cancella l'ultima virgola
    ss << "]";
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
}

RCLCPP_COMPONENTS_REGISTER_NODE(DepthConeMapNode);

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthConeMapNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
