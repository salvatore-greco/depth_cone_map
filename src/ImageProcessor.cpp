#include "depth_cone_map/ImageProcessor.hpp"

#include <rclcpp/logging.hpp>

std::list<std::pair<cv::Point, cv::Point>> ImageProcessor::parseBoundingBoxesJson(simdjson::padded_string json_bb) {
    std::list<std::pair<cv::Point, cv::Point>> bb_points; //lista con le bb: top left, bottom right
    simdjson::ondemand::parser parser;
    auto doc = parser.iterate(json_bb);
    for (simdjson::ondemand::object object: doc) {
        simdjson::ondemand::array bb_points_array = object["BB"].get_array();
        for (auto point : bb_points_array) {
            auto point_iterator = point.begin();
            bb_points.emplace_back((int)*point_iterator, (int)*(++point_iterator));
        }
    }
    return bb_points;
}


cv::Mat ImageProcessor::getPixelInBB(const MessageContainer &messages) {
    //converting ros image into opencv image
    const std::shared_ptr<const sensor_msgs::msg::Image> image = messages.getImage();
    const cv_bridge::CvImagePtr image_converted = cv_bridge::toCvCopy(image, image->encoding);
    cv::Mat cvImage = image_converted->image;

    auto bb_msg = messages.getBB();
    //json arriva senza terminazione, richiesta da simdjson.
    simdjson::padded_string json_bb(bb_msg->json); //non ha il costruttore di copia

    std::list<std::pair<cv::Point, cv::Point>> bb_points;
    try {
    bb_points = parseBoundingBoxesJson(std::move(json_bb));
    }
    catch (const simdjson::simdjson_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("depth_cone_map"), "An error occurred while parsing bounding boxes JSON %s, error code %d", e.what(), e.error());
        //error is an enum, check https://simdjson.github.io/simdjson/structsimdjson_1_1simdjson__error.html
        return cv::Mat::zeros(cvImage.size(), cvImage.type());
    }
    //da qui nella lista ho tutte le bounding box
    cv::Mat img_masked = cv::Mat::zeros(cvImage.size(), cvImage.type()); //faccio un immagine nera

    for (auto bounding_box : bb_points) {
        cv::Rect bb_rect(bounding_box.first, bounding_box.second);
        cvImage(bb_rect).copyTo(img_masked(bb_rect));
    }
    return img_masked;
}
