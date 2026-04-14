#include "depth_cone_map/ImageProcessor.hpp"

#include <rclcpp/logging.hpp>

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
            auto top_left_it = top_left.begin();
            const int top_left_x = static_cast<int>((*top_left_it).get_int64());
            const int top_left_y = static_cast<int>((*(++top_left_it)).get_int64());
            simdjson::ondemand::array bottom_right = *(++points_it);
            auto bottom_right_it = bottom_right.begin();
            const int bottom_right_x = static_cast<int>((*bottom_right_it).get_int64());
            const int bottom_right_y = static_cast<int>((*(++bottom_right_it)).get_int64());
            bb_points.emplace_back(cv::Point(top_left_x, top_left_y),
                cv::Point(bottom_right_x, bottom_right_y));
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
    for (const auto &bounding_box: bb_points) {
        cv::Rect roi(bounding_box.first, bounding_box.second);
        cv::Mat roi_image = cvImage(roi);
        std::vector<float> roi_image_vector = roi_image.reshape(0, 1);
        const size_t percentile = roi_image_vector.size() * PERCENTILE;
        std::nth_element(roi_image_vector.begin(), roi_image_vector.begin() + percentile, roi_image_vector.end());
        const float cone_distance = roi_image_vector[percentile];


        cv::Size whole_size;
        cv::Point ROI_offset;
        roi_image.locateROI(whole_size, ROI_offset);
        const int x = (roi_image.cols / 2) + ROI_offset.x;
        const int y = (roi_image.rows - (roi_image.rows * 0.3)) + ROI_offset.y;
        cones.emplace_back(x, y, cone_distance);
    }
    return cones;
}

driverless_msgs::msg::MarkerArrayStamped ImageProcessor::cameraToWorld(const std::vector<cv::Point3f> &cones) {
    //FIXME: quando farai il launch metti questi parametri là (remap o semplice param)
    //TODO: assumendo che ce la faccia a trasformare fra un frame e l'altro uso tf2::TimePointZero. In caso non sia così c'è da ripensare come messageContainer viene gestito
    //Magari usando uno shared ptr o roba simile? Cambia ad ogni callback
    std::vector<visualization_msgs::msg::Marker> markers;
    geometry_msgs::msg::TransformStamped transformation = tf2_buffer.lookupTransform(
        "map", "zed_camera_link", tf2::TimePointZero);
    for (const auto &cone: cones) {
        //marker non ha le funzioni per la trasformazione
        geometry_msgs::msg::Point point_to_transform = cvPoint3fToGeometryMsgsPoint(cone);
        geometry_msgs::msg::Point point_trasformed;
        tf2_buffer.transform(point_to_transform, point_trasformed, "map");
        //se questa non funziona da tf2_geometry_msgs: tf2::do_transform(point_to_transform, point_trasformed, transformation);
        visualization_msgs::msg::Marker marker;
        //copiando come viene fatto da clustering_node
        marker.type = marker.CYLINDER;
        marker.header.frame_id = "map";
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.7;
        marker.pose.position.x = point_trasformed.x;
        marker.pose.position.y = point_trasformed.y;
        marker.pose.position.z = point_trasformed.z;
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.z = 0.0;
        markers.push_back(std::move(marker));
    }
    driverless_msgs::msg::MarkerArrayStamped marker_array_stamped;
    marker_array_stamped.markers = markers;
    return marker_array_stamped;
}

geometry_msgs::msg::Point ImageProcessor::cvPoint3fToGeometryMsgsPoint(cv::Point3f point) {
    //geometry_msgs/Point non ha il costruttore, assurdo lo so
    geometry_msgs::msg::Point geometry_msgs_point;
    geometry_msgs_point.x = point.x;
    geometry_msgs_point.y = point.y;
    geometry_msgs_point.z = point.z;
    return geometry_msgs_point;
}
