#include "depth_cone_map/DepthConeMapNode.hpp"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <chrono>
#include <functional>
#include <nanoflann.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>

#include "depth_cone_map/Cone.hpp"
#include "depth_cone_map/ConeAdaptor.hpp"
#include "depth_cone_map/MessageContainer.hpp"

// TODO: togliere pose che tanto non serve e usare message container come membro delle classi
DepthConeMapNode::DepthConeMapNode(const rclcpp::NodeOptions& options) : Node("depth_cone_map", options), cone_adaptor(cones), kd_tree_cones(3, cone_adaptor) {
    parameterDeclaration();
    parameterInitialization();
    message_container = std::make_shared<MessageContainer>();
    ros_handler = std::make_unique<RosHandler>(
        this,
        this->map_frame_name,
        std::bind(&DepthConeMapNode::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
        std::bind(&DepthConeMapNode::cameraInfoCallback, this, std::placeholders::_1)
    );
    image_processor = std::make_unique<ImageProcessor>(this->get_logger(), this->percentile, message_container);
    image_transformer = std::make_unique<ImageTransformer>(this->get_clock(), this->map_frame_name,
                                                           this->camera_frame_name, this->get_logger());
    keyframe_handler = std::make_shared<KeyframeHandler>(std::make_unique<TemporalKeyframeStrategy>(std::chrono::seconds(1))); //è shared ptr in caso volessi mettere superpoint-glue in un thread
    //TODO: fare switch per istanziare la strategy da parametro launch (+ settare parametro tempo)
    // TODO: implementa l'altra strategia

    odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 0.002, 0.002, 0.002,  // Incertezza rotazionale per passo
                                  0.02,  0.02,  0.02   // Incertezza traslazionale per passo
            ).finished()
    );

    // infinitesima perchè deve essere stabile
    prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6,   // Rotazione roll, pitch, yaw (rad)
                                  1e-6, 1e-6, 1e-6   // Traslazione x, y, z (metri)
            ).finished()
    );
    landmark_noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.05, 0.05, 0.3))
    );
    cones.reserve(500);
}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const sensor_msgs::msg::Image::ConstSharedPtr& image_left, const driverless_msgs::msg::PoseStamped::ConstSharedPtr& pose) {

    static int cone_idx = 0;
    static int pose_idx = 0;
    // questi non andrebbero ridefiniti ogni volta;
    static gtsam::NonlinearFactorGraph new_factors;
    static gtsam::Values new_values;
    static gtsam::Pose3 last_pose;
    // ---------------------------------------------

    // un problema che potrei avere è che o chiamo isam update spesso e vedo i coni che mi caca fuori
    // oppure pubblico il cono appena lo vedo e lo raffino dopo
    // se sono operazioni abbastanza veloci ho una mappa solida anche per l'autox, altrimenti solo trackdrive

    gtsam::Pose3 gtsam_pose(
        gtsam::Rot3::Quaternion(
            pose->pose.orientation.w,
            pose->pose.orientation.x,
            pose->pose.orientation.y,
            pose->pose.orientation.z
        ),
        gtsam::Point3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z)
    );
    auto key = gtsam::Symbol('p', pose_idx++);
    if (first_pose){
        new_factors.addPrior(key, gtsam_pose, this->prior_noise);
        new_values.insert(key, gtsam_pose);
        last_pose = gtsam_pose;
        first_pose = false;
    }
    else {
        gtsam::Pose3 relative_pose = last_pose.between(gtsam_pose);
        new_factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
            gtsam::Symbol('p', pose_idx - 2),
            key,
            relative_pose,
            this->odom_noise
        );
        new_values.insert(key, gtsam_pose);
        last_pose = gtsam_pose;
    }
    RCLCPP_INFO(this->get_logger(), "Received messages");
    this->message_container->saveMessages(bounding_boxes, depth_image, image_left);
    const auto bounding_boxes_list = image_processor->getBBInJSON();
    const auto cones = image_processor->getConeInCameraFrame(bounding_boxes_list);
    auto cones_world_frame = image_transformer->cameraToWorld(cones);

    if (this->cones.empty()){
        Cone& first_cone = cones_world_frame[0];
        this->cones.push_back(cones_world_frame[0]);
        kd_tree_cones.addPoints(cones.size()-1, cones.size());
        new_values.insert(gtsam::Symbol('l', cone_idx++), gtsam::Point3(first_cone.position_world_frame.x, first_cone.position_world_frame.y, first_cone.position_world_frame.z));
    }

    for (const auto& cone : cones_world_frame){
        // ho seguito l'esempio in nanoflann/example/dynamic_pointcloud_example.cpp
        static const double DIST_THRESHOLD = 3.0;
        size_t ret_idx;
        float out_dist_square;
        auto start = std::chrono::high_resolution_clock::now();
        nanoflann::KNNResultSet<float> result_set(1); //1 come magic number perchè voglio il primo vicino più vicino
        result_set.init(&ret_idx, &out_dist_square);
        float query_pt[3] = {cone.position_world_frame.x, cone.position_world_frame.y, cone.position_world_frame.z};
        kd_tree_cones.findNeighbors(result_set, query_pt, {10});
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
        int matched_id;
        Eigen::Map<const Eigen::Vector3f> a(&cone.position_world_frame.x);
        Eigen::Map<const Eigen::Vector3f> b(&this->cones[ret_idx].position_world_frame.x);

        // non ho gia visto il cono in esame
        if ((a-b).norm()>DIST_THRESHOLD){
            this->cones.emplace_back(cone.position_world_frame, "colore", cone_idx++);
            // kd_tree_cones.addPoints(cone_idx-1, cone_idx); //sono gli indici nel contenitore
            kd_tree_cones.addPoints(this->cones.size()-1, this->cones.size());
            //va aggiunto a isam come values
            new_values.insert(
                gtsam::Symbol('l', cone_idx-1),
                gtsam::Point3(cone.position_world_frame.x, cone.position_world_frame.y, cone.position_world_frame.z)
            );
            matched_id = cone_idx-1;
        }
        else
            matched_id = this->cones[ret_idx].id;

        gtsam::Point3 cone_point(cone.position_world_frame.x, cone.position_world_frame.y, cone.position_world_frame.z);
        double range = gtsam_pose.range(cone_point);
        gtsam::Unit3 bearing = gtsam_pose.bearing(cone_point);
        new_factors.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(
            key,
            gtsam::Symbol('l', matched_id),
            bearing,
            range,
            this->landmark_noise
        );
    }
    if (debug) {
        printDebug(bounding_boxes_list, cones, cones_world_frame);
    }
    // isam2.update()
    isam2.update(new_factors, new_values);
    new_factors.resize(0);
    new_values.clear();
    //dovrei pubblicare quelli che mi dà isam
    // proviamoci
    // if(keyframe_handler->keyframe_strategy->isKeyframeInvalid()){
        gtsam::Values estimate = isam2.calculateEstimate();
        auto landmarks = estimate.filter(gtsam::Symbol::ChrTest('l'));
        for (const auto & val : landmarks){
            gtsam::Point3 p = estimate.at<gtsam::Point3>(val.key);
            std::cout << "Key:" << gtsam::Symbol(val.key).index()<< " value:" << p << std::endl;
            auto& c = this->cones[gtsam::symbolIndex(val.key)];
            c.position_world_frame.x = p.x();
            c.position_world_frame.y = p.y();
            c.position_world_frame.z = p.z();
        }
    // }
    ros_handler->publish_cones(this->cones);
    // quando faccio isam update pulisco il grafo che stavo mantenendo qua (viene aggiunto tramite isam update)
    RCLCPP_INFO(this->get_logger(), "%d", cone_idx);
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
}

void DepthConeMapNode::parameterInitialization() {
    map_frame_name = this->get_parameter("world_frame").as_string();
    camera_frame_name = this->get_parameter("camera_frame").as_string();
    percentile = this->get_parameter("percentile").as_double();
    debug = this->get_parameter("debug").as_bool();
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
