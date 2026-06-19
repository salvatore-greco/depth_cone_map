#include "depth_cone_map/ImageProcessor.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>
#include "depth_cone_map/Cone.hpp"

std::list<std::pair<cv::Rect, ConeColor>> ImageProcessor::getBBInJSON() {
    /* JSON structure
     * [{"color": "color_string", "BB": [[x_l, y_l],[x_r, y_r]]}, ... ]
     * dove x_l, y_l sono le coordinate del punto top left; x_r, y_r le coordinate del punto bottom right
     */

    auto bb_msg = message_container->getBB();
    //json arriva senza terminazione, richiesta da simdjson.
    simdjson::padded_string json_bb(bb_msg->json);

    std::list<std::pair<cv::Rect, ConeColor>> bb_points;
    try {
        auto doc = parser.iterate(json_bb);
        for (simdjson::ondemand::object object: doc) {
            std::string_view color = object["color"].get_string();
            ConeColor cone_color = colorStringToEnum(color);
            simdjson::ondemand::array bb_points_array = object.find_field("BB").get_array();
            auto points_it = bb_points_array.begin();
            simdjson::ondemand::array top_left = *points_it;
            cv::Point top_left_point = extractBoundingBox(top_left);
            simdjson::ondemand::array bottom_right = *(++points_it);
            cv::Point bottom_right_point = extractBoundingBox(bottom_right);
            bb_points.emplace_back(cv::Rect(top_left_point, bottom_right_point), cone_color);
        }
    } catch (const simdjson::simdjson_error &e) {
        RCLCPP_ERROR(logger,
                     "An error occurred while parsing bounding boxes JSON %s, error code %d", e.what(), e.error());
        //error is an enum, check https://simdjson.github.io/simdjson/structsimdjson_1_1simdjson__error.html
    }
    return bb_points;
}


std::vector<Cone> ImageProcessor::getConeInCameraFrame(const std::list<std::pair<cv::Rect, ConeColor>> &bb_points) {
    const std::shared_ptr<const sensor_msgs::msg::Image> image = message_container->getDepthImage();
    cv_bridge::CvImageConstPtr image_converted;
    cv::Mat cvImage;
    try{
        image_converted = cv_bridge::toCvShare(image, image->encoding);
        cvImage = image_converted->image;
    }
    catch(const cv_bridge::Exception& e){
        RCLCPP_ERROR(logger, "[cv_bridge] Error when converting sensor_msgs::msg::Image to cv::Mat: %s", e.what());
    }

    // Calcolo dell'orientamento per identificare la "vera base" del cono (Gravity-Aware)
    auto pose = message_container->getPose();
    Eigen::Quaterniond camera_quaternion(pose->pose.orientation.w, pose->pose.orientation.x,
                                          pose->pose.orientation.y, pose->pose.orientation.z);
    camera_quaternion.normalize();
    Eigen::Matrix3d rotation_matrix = camera_quaternion.toRotationMatrix();

    // Matrice di rotazione Optical Frame (C -> W)
    // ROS Link: X=forward, Y=left, Z=up
    // Optical: Z=forward, X=right, Y=down
    Eigen::Matrix3d rotation_matrix_optical;
    rotation_matrix_optical.col(0) = -rotation_matrix.col(1); // X_opt = -Y_ros
    rotation_matrix_optical.col(1) = -rotation_matrix.col(2); // Y_opt = -Z_ros
    rotation_matrix_optical.col(2) =  rotation_matrix.col(0); // Z_opt =  X_ros

    // Proiezione della gravità del mondo [0, 0, -1] nel piano immagine (X, Y dell'optical frame)
    double gx = -rotation_matrix_optical(2, 0);
    double gy = -rotation_matrix_optical(2, 1);

    double mag_sq = gx*gx + gy*gy;
    double u, v;
    const double EPSILON = 1e-6;

    if (mag_sq < EPSILON) {
        // Singolarità (camera verticale): fallback al basso naturale dell'immagine
        u = 0.0;
        v = 1.0;
    } else {
        double mag = std::sqrt(mag_sq);
        u = gx / mag;
        v = gy / mag;
    }

    std::vector<Cone> cones;
    for (const auto &bounding_box: bb_points) {
        cv::Mat roi_image = cvImage(bounding_box.first).clone();
        const size_t percentile = roi_image.total() * PERCENTILE;
        auto perc_it = roi_image.begin<float>() + percentile;
        std::nth_element(roi_image.begin<float>(), perc_it, roi_image.end<float>());
        const float cone_distance = *perc_it;
        if (isDepthValueInvalid(cone_distance))
            continue;

        // Centro della Bounding Box
        cv::Point2f center(bounding_box.first.x + bounding_box.first.width / 2.0f,
                           bounding_box.first.y + bounding_box.first.height / 2.0f);

        // Spostamento dal centro verso la base seguendo il versore di gravità
        // Usiamo un offset del 40% dell'altezza (equivalente al vecchio 0.7 * rows dal top)
        float offset = bounding_box.first.height * 0.4f;

        cv::Vec3d pixel(center.x + u * offset, center.y + v * offset, 1);
        cv::Mat point_camera_frame = backProjection(std::move(pixel), cone_distance);

        cones.emplace_back(cv::Point3f(point_camera_frame.at<double>(0,0),
                                      point_camera_frame.at<double>(1,0),
                                      point_camera_frame.at<double>(2,0)),
                           bounding_box.second, -1);
    }
    return cones;
}


cv::Mat ImageProcessor::backProjection(cv::Vec3d pixel, float depth) {
    //source: https://math.stackexchange.com/questions/4382437/back-projecting-a-2d-pixel-from-an-image-to-its-corresponding-3d-point
    cv::Mat result = depth*k_matrix.inv()*pixel;
    return result;
}

bool ImageProcessor::isDepthValueInvalid(const float& value) const{
    return (value == 0 || std::isnan(value) || std::isinf(value));
}

cv::Point ImageProcessor::extractBoundingBox(simdjson::ondemand::array& bounding_box) {
    auto bounding_box_it = bounding_box.begin();
    const int x = static_cast<int>((*bounding_box_it).get_int64());
    const int y = static_cast<int>((*(++bounding_box_it)).get_int64());
    return cv::Point(x, y);
}
