#include "depth_cone_map/ImageProcessor.hpp"

#include <rclcpp/logging.hpp>


std::list<std::pair<cv::Point, cv::Point>> ImageProcessor::getPixelInBB(const MessageContainer &messages) {
    auto bb_msg = messages.getBB();
    //json arriva senza terminazione, richiesta da simdjson.
    simdjson::padded_string json_bb(bb_msg->json); //non ha il costruttore di copia

    std::list<std::pair<cv::Point, cv::Point> > bb_points;
    try {
        simdjson::ondemand::parser parser;
        auto doc = parser.iterate(json_bb);
        for (simdjson::ondemand::object object: doc) {
            simdjson::ondemand::array bb_points_array = object["BB"].get_array();
            for (auto point: bb_points_array) {
                auto point_iterator = point.begin();
                bb_points.emplace_back((int) *point_iterator, (int) *(++point_iterator));
            }
        }
    } catch (const simdjson::simdjson_error &e) {
        RCLCPP_ERROR(rclcpp::get_logger("depth_cone_map"),
                     "An error occurred while parsing bounding boxes JSON %s, error code %d", e.what(), e.error());
        //error is an enum, check https://simdjson.github.io/simdjson/structsimdjson_1_1simdjson__error.html
    }
    return bb_points;
}

//va chiamata usando move semantic
std::vector<cv::Point3f> ImageProcessor::coneFinder(const MessageContainer &messages,
                                                    const std::list<std::pair<cv::Point, cv::Point> > &bb_points) {
    const std::shared_ptr<const sensor_msgs::msg::Image> image = messages.getImage();
    const cv_bridge::CvImageConstPtr image_converted = cv_bridge::toCvShare(image, image->encoding);
    const cv::Mat cvImage = image_converted->image;

    std::vector<cv::Point3f> cones(bb_points.size());
    for (const auto& bounding_box : bb_points) {
        cv::Rect roi(bounding_box.first, bounding_box.second);
        cv::Mat roi_image = cvImage(roi);
        std::vector<float> roi_image_vector = roi_image.reshape(0,1);
        const size_t percentile = roi_image_vector.size() * PERCENTILE;
        std::nth_element(roi_image_vector.begin(), roi_image_vector.begin()+percentile, roi_image_vector.end());
        const float cone_distance = roi_image_vector[percentile];

        const int x = roi_image.cols / 2;
        const int y = roi_image.rows - (roi_image.rows * 0.3);
        cones.emplace_back(x,y,cone_distance);
    }
    return cones;
}
