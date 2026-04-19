#include "depth_cone_map/ImageProcessor.hpp"
#include <opencv2/core/types.hpp>

//TODO: salvarsi colore e fare una struct (o una classe) Cone.

std::list<std::pair<cv::Point, cv::Point> > ImageProcessor::getBBInJSON(const MessageContainer &messages) {
    /* JSON structure
     * [{"color": "color_string", "BB": [[x_l, y_l],[x_r, y_r]]}, ... ]
     * dove x_l, y_l sono le coordinate del punto top left; x_r, y_r le coordinate del punto bottom right
     */

    auto bb_msg = messages.getBB();
    //json arriva senza terminazione, richiesta da simdjson.
    simdjson::padded_string json_bb(bb_msg->json);

    std::list<std::pair<cv::Point, cv::Point> > bb_points;
    try {
        simdjson::ondemand::parser parser;
        auto doc = parser.iterate(json_bb);
        for (simdjson::ondemand::object object: doc) {
            simdjson::ondemand::array bb_points_array = object.find_field("BB").get_array();
            auto points_it = bb_points_array.begin();
            simdjson::ondemand::array top_left = *points_it;
            cv::Point top_left_point = extractBoundingBox(top_left);
            simdjson::ondemand::array bottom_right = *(++points_it);
            cv::Point bottom_right_point = extractBoundingBox(bottom_right);
            bb_points.emplace_back(top_left_point,bottom_right_point);
        }
    } catch (const simdjson::simdjson_error &e) {
        RCLCPP_ERROR(logger,
                     "An error occurred while parsing bounding boxes JSON %s, error code %d", e.what(), e.error());
        //error is an enum, check https://simdjson.github.io/simdjson/structsimdjson_1_1simdjson__error.html
    }
    return bb_points;
}


std::vector<cv::Point3f> ImageProcessor::coneFinder(const MessageContainer &messages,
                                                    const std::list<std::pair<cv::Point, cv::Point> > &bb_points) {
    const std::shared_ptr<const sensor_msgs::msg::Image> image = messages.getImage();
    cv_bridge::CvImageConstPtr image_converted;
    cv::Mat cvImage;
    try{
    image_converted = cv_bridge::toCvShare(image, image->encoding);
    cvImage = image_converted->image;
    }
    catch(const cv_bridge::Exception& e){
        RCLCPP_ERROR(logger, "[cv_bridge] Error when converting sensor_msgs::msg::Image to cv::Mat: %s", e.what());
    }

    std::vector<cv::Point3f> cones;
    for (const auto &bounding_box: bb_points) {
        cv::Rect roi(bounding_box.first, bounding_box.second);
        cv::Mat roi_image = cvImage(roi).clone();
        const size_t percentile = roi_image.total() * PERCENTILE;
        auto perc_it = roi_image.begin<float>() + percentile;
        std::nth_element(roi_image.begin<float>(), perc_it, roi_image.end<float>());
        const float cone_distance = *perc_it;
        if (isDepthValueInvalid(cone_distance))
            continue;
        const int x = (roi_image.cols / 2) + bounding_box.first.x;
        const int y = roi_image.rows - roi_image.rows * 0.3 + bounding_box.first.y;
        cv::Vec3d pixel(x,y,1);
        cv::Mat point_camera_frame = backProjection(std::move(pixel), cone_distance);
        cones.emplace_back(point_camera_frame.at<double>(0,0), point_camera_frame.at<double>(1,0), point_camera_frame.at<double>(2,0));
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



