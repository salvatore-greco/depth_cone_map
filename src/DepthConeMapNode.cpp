#include "depth_cone_map/DepthConeMapNode.hpp"
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
#include <nanoflann.hpp>
#include <opencv2/core/types.hpp>

#include "depth_cone_map/ConeAdaptor.hpp"
#include "depth_cone_map/MessageContainer.hpp"

// TODO: togliere pose che tanto non serve e usare message container come membro delle classi
DepthConeMapNode::DepthConeMapNode(const rclcpp::NodeOptions& options) : Node("depth_cone_map", options), cone_adaptor(cones), kd_tree_cones(3, cone_adaptor) {
    parameterDeclaration();
    parameterInitialization();
    message_container = std::make_shared<MessageContainer>();
    ros_handler = std::make_unique<RosHandler>(this, *this);
    image_processor = std::make_unique<ImageProcessor>(this->get_logger(), this->percentile, message_container);
    image_transformer = std::make_unique<ImageTransformer>(this->get_clock(), this->map_frame_name,
                                                           this->camera_frame_name, this->get_logger());
    keyframe_handler = std::make_shared<KeyframeHandler>(std::make_unique<TemporalKeyframeStrategy>(std::chrono::seconds(1))); //è shared ptr in caso volessi mettere superpoint-glue in un thread
    //TODO: fare switch per istanziare la strategy da parametro launch (+ settare parametro tempo)
    // TODO: implementa l'altra strategia

    //ok dopo un colloquio con Claude la mia intuizione era corretta, posso usare la prima posa come prior pose.
    // prior noise e odom noise le posso mettere qua

    //FIXME: metti dei valori di rumore decenti
    prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1,1,1));
    odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(1,1,1));
    cones.reserve(500);
}

void DepthConeMapNode::callback(const driverless_msgs::msg::BoundingBoxes::ConstSharedPtr& bounding_boxes, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image, const sensor_msgs::msg::Image::ConstSharedPtr& image_left, const driverless_msgs::msg::PoseStamped::ConstSharedPtr& pose) {

    static int cone_idx = 0;
    static int pose_idx = 0;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    // un problema che potrei avere è che o chiamo isam update spesso e vedo i coni che mi caca fuori
    // oppure pubblico il cono appena lo vedo e lo raffino dopo
    // se sono operazioni abbastanza veloci ho una mappa solida anche per l'autox, altrimenti solo trackdrive

    if (first_pose){
        gtsam::Pose3 prior_pose(
            gtsam::Rot3::Quaternion(
                pose->pose.orientation.w,
                pose->pose.orientation.x,
                pose->pose.orientation.y,
                pose->pose.orientation.z
            ),
            gtsam::Point3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z)
        );
        new_factors.addPrior(gtsam::Symbol('p', pose_idx++), prior_pose, this->prior_noise);
        first_pose = false;
    }
    else{
        // creare oggetto posa come between factor + landmark al grafo + da.
        // va relativizzata la posa rispetto all'ultima
    }
    RCLCPP_INFO(this->get_logger(), "Received messages");
    this->message_container->saveMessages(bounding_boxes, depth_image, image_left);
    RCLCPP_INFO(this->get_logger(), "after saving messages");
    const auto bounding_boxes_list = image_processor->getBBInJSON();
    const auto cones = image_processor->getConeInCameraFrame(bounding_boxes_list);
    auto marker_array_cones = image_transformer->cameraToWorld(cones);

    for (const cv::Point3f& cone : cones){
        //facciamolo brutto, devo cambiare cosa mi ritornano getConeInCameraFrame e camera to world
        // ho seguito l'esempio in nanoflann/example/dynamic_pointcloud_example.cpp
        const double DIST_THRESHOLD = 1.0;
        size_t ret_idx;
        float out_dist_square;
        nanoflann::KNNResultSet<float> result_set(1); //1 come magic number perchè voglio il primo vicino più vicino
        result_set.init(&ret_idx, &out_dist_square);
        kd_tree_cones.findNeighbors(result_set, cone, {10}); //TODO: cone deve essere del tipo definito in cone adaptor(quindi struct cone).
        if (cv::norm(cone-(this->cones[ret_idx].position_world_frame))>DIST_THRESHOLD){
            Cone cone_to_add(cone, "colore", cone_idx++);
            this->cones.push_back(cone_to_add);
            kd_tree_cones.addPoints(cone_idx-1, cone_idx); //sono gli indici nel contenitore
            //va aggiunto a isam come values
            new_values.insert(gtsam::Symbol('l', cone_to_add.id), gtsam::Point2(cone_to_add.position_world_frame.x, cone_to_add.position_world_frame.y));
        }
        // va aggiunto a isam come factor
    }
    if (debug) {
        printDebug(bounding_boxes_list, cones, marker_array_cones.markers);
    }
    // isam2.update()
    ros_handler->publish_cones(std::move(marker_array_cones));
    // quando faccio isam update pulisco il grafo che stavo mantenendo qua (viene aggiunto tramite isam update)
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
                                  const std::vector<visualization_msgs::msg::Marker>& marker_array_cones) {
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

    ss << "Cones in world frame, size " << marker_array_cones.size() << ": [\n";
    for (const auto& cone_world : marker_array_cones) {
        ss << "[" << cone_world.pose.position.x << "," << cone_world.pose.position.y << ", "
           << cone_world.pose.position.z << "],";
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
